/* supervisor: starts hub, imu_proc and cam_proc, restarts them when they
 * exit or stop sending heartbeats, and stops them all cleanly on SIGTERM.
 * systemd keeps the supervisor itself running (Step 2).
 *
 * - Crash: SIGCHLD -> waitpid -> log exit status -> restart after a backoff
 *   (1 s, doubling to 30 s for repeated quick failures).
 * - Hang: each child writes heartbeat bytes to a pipe on fd 3 (the fd number
 *   is also in POSTURE_HEARTBEAT_FD). No first heartbeat within 5 s, or
 *   3 s of silence after that, -> SIGKILL -> reaped and restarted as above.
 *   A sensor that is alive but ABSENT keeps sending heartbeats, so it is
 *   not restarted. --no-watchdog disables this (children without heartbeat
 *   support).
 * - Every child gets PR_SET_PDEATHSIG(SIGTERM), so a dead supervisor leaves
 *   no orphans; a restarted supervisor starts fresh children.
 * - Events go to hub over <run_dir>/sup.sock (queued while hub is down, so a
 *   hub crash and restart still reach the log) and to stderr (journal).
 */
#define _GNU_SOURCE /* pipe2 */
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/prctl.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ipc_client.h"
#include "msg.h"
#include "sup_policy.h"
#include "timeutil.h"

#define SUP_TICK_NS        (100 * NS_PER_MS) /* watchdog and restart checks */
#define SUP_SENSOR_DELAY_NS (500 * NS_PER_MS) /* start sensors after hub */
#define SUP_STOP_GRACE_NS  (3 * NS_PER_S)    /* SIGTERM, then SIGKILL (team decision) */
#define SUP_QUEUE_MAX      32                /* events held while hub is down */
#define SUP_DEFAULT_LOG    "posture.log"
#define HEARTBEAT_FD       3
#define NCHILD             3

enum { TAG_SIGNAL = 0x100, TAG_TIMER = 0x200, TAG_HB = 0x300 /* | child index */ };

struct child {
    const char *name;
    char path[PATH_MAX];
    pid_t pid;              /* 0 when not running */
    int hb_fd;              /* read end of the heartbeat pipe, -1 if none */
    int64_t started_ns;
    int64_t last_hb_ns;
    int got_hb;
    int watchdog_killed;    /* we sent SIGKILL for silence */
    int64_t start_at_ns;    /* pending (re)start time, -1 if none */
    unsigned failures;      /* consecutive quick failures, for the backoff */
    unsigned restarts;      /* total restarts since the supervisor started */
};

struct sup {
    struct ipc_client ipc;
    int epfd, sfd, tfd;
    const char *run_dir;
    const char *log_path;
    int watchdog;
    pid_t self;
    struct child kids[NCHILD];

    struct msg_event queue[SUP_QUEUE_MAX];
    unsigned q_head, q_len;
    uint64_t q_dropped;

    int stopping;
    int64_t stop_deadline_ns;
    int killed_on_stop;
};

/* Supervisor events carry their own CLOCK_MONOTONIC time in the body, since
 * a queued event reaches hub's log later than it happened. */
static void sup_event(struct sup *s, const char *name, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void flush_queue(struct sup *s)
{
    while (s->q_len > 0 && ipc_client_ready(&s->ipc)) {
        struct msg_event *e = &s->queue[s->q_head];

        if (ipc_send(&s->ipc, &e->h, sizeof *e) < 0)
            return; /* hub busy or just went away; try again next tick */
        s->q_head = (s->q_head + 1) % SUP_QUEUE_MAX;
        s->q_len--;
    }
}

static void sup_event(struct sup *s, const char *name, const char *fmt, ...)
{
    struct msg_event *e;
    struct timespec t;
    char body[MSG_EVENT_BODY_MAX];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(body, sizeof body, fmt, ap);
    va_end(ap);
    if (n < 0)
        body[0] = '\0';

    (void)clock_gettime(CLOCK_MONOTONIC, &t);
    if (s->q_len == SUP_QUEUE_MAX) {
        s->q_head = (s->q_head + 1) % SUP_QUEUE_MAX; /* drop the oldest */
        s->q_len--;
        s->q_dropped++;
    }
    e = &s->queue[(s->q_head + s->q_len) % SUP_QUEUE_MAX];
    s->q_len++;

    memset(e, 0, sizeof *e);
    msg_init_hdr(&e->h, MSG_EVENT, SENSOR_OK);
    snprintf(e->name, sizeof e->name, "%s", name);
    n = snprintf(e->body, sizeof e->body, "mono=%lld.%09ld %s",
                 (long long)t.tv_sec, (long)t.tv_nsec, body);
    if (n < 0 || (size_t)n >= sizeof e->body)
        snprintf(e->body, sizeof e->body, "mono=%lld.%09ld truncated=1",
                 (long long)t.tv_sec, (long)t.tv_nsec);

    fprintf(stderr, "supervisor %s %s\n", e->name, e->body);
    flush_queue(s);
}

static struct child *find_child(struct sup *s, pid_t pid)
{
    for (int i = 0; i < NCHILD; i++)
        if (s->kids[i].pid == pid)
            return &s->kids[i];
    return NULL;
}

static int epoll_add(int epfd, int fd, uint32_t tag)
{
    struct epoll_event ev;

    memset(&ev, 0, sizeof ev);
    ev.events = EPOLLIN;
    ev.data.u32 = tag;
    return epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
}

static void close_hb(struct sup *s, struct child *c)
{
    if (c->hb_fd < 0)
        return;
    (void)epoll_ctl(s->epfd, EPOLL_CTL_DEL, c->hb_fd, NULL);
    close(c->hb_fd);
    c->hb_fd = -1;
}

/* Runs in the forked child; never returns. */
static void exec_child(const struct sup *s, const struct child *c, int hb_wfd)
    __attribute__((noreturn));
static void exec_child(const struct sup *s, const struct child *c, int hb_wfd)
{
    char *argv[8];
    sigset_t none;
    int k = 0;

    /* The supervisor blocks signals for its signalfd; a blocked mask
     * survives exec, so undo it before the child program starts. */
    sigemptyset(&none);
    sigprocmask(SIG_SETMASK, &none, NULL);
    signal(SIGPIPE, SIG_DFL);

    if (prctl(PR_SET_PDEATHSIG, SIGTERM) < 0 || getppid() != s->self)
        _exit(126); /* supervisor already gone */

    if (hb_wfd == HEARTBEAT_FD) {
        int fl = fcntl(hb_wfd, F_GETFD);

        if (fl < 0 || fcntl(hb_wfd, F_SETFD, fl & ~FD_CLOEXEC) < 0)
            _exit(126);
    } else if (dup2(hb_wfd, HEARTBEAT_FD) < 0) { /* the copy has no FD_CLOEXEC */
        _exit(126);
    }
    if (setenv("POSTURE_HEARTBEAT_FD", "3", 1) < 0)
        _exit(126);

    argv[k++] = (char *)c->path;
    argv[k++] = "--run-dir";
    argv[k++] = (char *)s->run_dir;
    if (strcmp(c->name, "hub") == 0) {
        argv[k++] = "--log";
        argv[k++] = (char *)s->log_path;
    }
    argv[k] = NULL;
    execv(c->path, argv);
    fprintf(stderr, "supervisor: exec %s: %s\n", c->path, strerror(errno));
    _exit(127);
}

static void schedule_restart(struct sup *s, struct child *c, int64_t now, int64_t ran_ns)
{
    int64_t delay;

    c->failures = sup_next_failures(c->failures, ran_ns);
    delay = sup_backoff_ns(c->failures);
    c->start_at_ns = now + delay;
    c->restarts++;
    sup_event(s, "SUP_RESTART", "child=%s n=%u delay_ms=%lld consecutive=%u",
              c->name, c->restarts, (long long)(delay / NS_PER_MS), c->failures);
}

static void spawn(struct sup *s, struct child *c, int64_t now)
{
    int p[2];
    pid_t pid;

    c->start_at_ns = -1;
    if (pipe2(p, O_CLOEXEC) < 0) {
        sup_event(s, "SUP_ERROR", "op=pipe2 child=%s errno=%d", c->name, errno);
        schedule_restart(s, c, now, 0);
        return;
    }
    pid = fork();
    if (pid < 0) {
        sup_event(s, "SUP_ERROR", "op=fork child=%s errno=%d", c->name, errno);
        close(p[0]);
        close(p[1]);
        schedule_restart(s, c, now, 0);
        return;
    }
    if (pid == 0)
        exec_child(s, c, p[1]);

    close(p[1]);
    if (fcntl(p[0], F_SETFL, O_NONBLOCK) < 0 ||
        epoll_add(s->epfd, p[0], TAG_HB | (uint32_t)(c - s->kids)) < 0) {
        /* No way to hear heartbeats; the watchdog will restart it. */
        sup_event(s, "SUP_ERROR", "op=heartbeat_pipe child=%s errno=%d", c->name, errno);
        close(p[0]);
        p[0] = -1;
    }
    c->pid = pid;
    c->hb_fd = p[0];
    c->started_ns = now;
    c->got_hb = 0;
    c->watchdog_killed = 0;
    sup_event(s, "SUP_CHILD_START", "child=%s pid=%d restart=%u", c->name, (int)pid,
              c->restarts);
}

static void on_exit_status(struct sup *s, struct child *c, int status, int64_t now)
{
    int64_t ran_ns = now - c->started_ns;
    const char *cause = s->stopping ? "stop" : c->watchdog_killed ? "watchdog" : "exit";

    if (WIFSIGNALED(status))
        sup_event(s, "SUP_CHILD_EXIT", "child=%s pid=%d signal=%d cause=%s ran_ms=%lld",
                  c->name, (int)c->pid, WTERMSIG(status), cause,
                  (long long)(ran_ns / NS_PER_MS));
    else
        sup_event(s, "SUP_CHILD_EXIT", "child=%s pid=%d exit=%d cause=%s ran_ms=%lld",
                  c->name, (int)c->pid, WEXITSTATUS(status), cause,
                  (long long)(ran_ns / NS_PER_MS));
    close_hb(s, c);
    c->pid = 0;
    if (!s->stopping)
        schedule_restart(s, c, now, ran_ns);
}

static void reap(struct sup *s)
{
    for (;;) {
        int status;
        pid_t pid = waitpid(-1, &status, WNOHANG);
        struct child *c;

        if (pid <= 0) {
            if (pid < 0 && errno == EINTR)
                continue;
            return; /* 0: none left to reap; ECHILD: no children */
        }
        c = find_child(s, pid);
        if (c)
            on_exit_status(s, c, status, mono_ns());
    }
}

static void read_heartbeat(struct sup *s, struct child *c)
{
    char buf[64];
    ssize_t n;

    while ((n = read(c->hb_fd, buf, sizeof buf)) > 0) {
        c->last_hb_ns = mono_ns();
        c->got_hb = 1;
    }
    if (n == 0)
        close_hb(s, c); /* child closed fd 3 or exited; waitpid reports which */
}

static void begin_stop(struct sup *s, int64_t now)
{
    s->stopping = 1;
    s->stop_deadline_ns = now + SUP_STOP_GRACE_NS;
    sup_event(s, "SUP_STOPPING", "grace_ms=%lld", (long long)(SUP_STOP_GRACE_NS / NS_PER_MS));
    for (int i = 0; i < NCHILD; i++) {
        s->kids[i].start_at_ns = -1;
        if (s->kids[i].pid > 0)
            kill(s->kids[i].pid, SIGTERM);
    }
}

/* Returns 1 once stopping has finished. */
static int on_tick(struct sup *s, int64_t now)
{
    int running = 0;

    for (int i = 0; i < NCHILD; i++) {
        struct child *c = &s->kids[i];

        if (c->pid > 0) {
            running++;
            if (s->watchdog && !s->stopping && !c->watchdog_killed &&
                now > sup_watchdog_deadline(c->started_ns, c->got_hb, c->last_hb_ns)) {
                int64_t silent = now - (c->got_hb ? c->last_hb_ns : c->started_ns);

                c->watchdog_killed = 1;
                kill(c->pid, SIGKILL);
                sup_event(s, "SUP_WATCHDOG", "child=%s pid=%d silent_ms=%lld first_hb=%s",
                          c->name, (int)c->pid, (long long)(silent / NS_PER_MS),
                          c->got_hb ? "yes" : "no");
            }
        } else if (c->start_at_ns >= 0 && now >= c->start_at_ns && !s->stopping) {
            spawn(s, c, now);
        }
    }

    if (s->stopping) {
        if (running == 0)
            return 1;
        if (!s->killed_on_stop && now >= s->stop_deadline_ns) {
            s->killed_on_stop = 1;
            for (int i = 0; i < NCHILD; i++)
                if (s->kids[i].pid > 0) {
                    sup_event(s, "SUP_STOP_KILL", "child=%s pid=%d", s->kids[i].name,
                              (int)s->kids[i].pid);
                    kill(s->kids[i].pid, SIGKILL);
                }
        }
    }
    flush_queue(s);
    return 0;
}

/* Directory of this executable, so children are found next to it. */
static int exe_dir(char *out, size_t cap)
{
    ssize_t n = readlink("/proc/self/exe", out, cap - 1);
    char *slash;

    if (n < 0)
        return -1;
    out[n] = '\0';
    slash = strrchr(out, '/');
    if (!slash)
        return -1;
    *slash = '\0';
    return 0;
}

static int setup(struct sup *s, const char *bin_dir)
{
    struct itimerspec its;
    sigset_t mask;

    for (int i = 0; i < NCHILD; i++) {
        struct child *c = &s->kids[i];
        int n = snprintf(c->path, sizeof c->path, "%s/%s", bin_dir, c->name);

        if (n < 0 || (size_t)n >= sizeof c->path) {
            errno = ENAMETOOLONG;
            return -1;
        }
        if (access(c->path, X_OK) < 0) {
            fprintf(stderr, "supervisor: %s: %s (run make first?)\n", c->path, strerror(errno));
            return -1;
        }
    }

    s->epfd = epoll_create1(EPOLL_CLOEXEC);
    if (s->epfd < 0)
        return -1;

    sigemptyset(&mask);
    sigaddset(&mask, SIGCHLD);
    sigaddset(&mask, SIGTERM);
    sigaddset(&mask, SIGINT);
    if (sigprocmask(SIG_BLOCK, &mask, NULL) < 0)
        return -1;
    s->sfd = signalfd(-1, &mask, SFD_CLOEXEC | SFD_NONBLOCK);
    if (s->sfd < 0 || epoll_add(s->epfd, s->sfd, TAG_SIGNAL) < 0)
        return -1;

    memset(&its, 0, sizeof its);
    its.it_value.tv_nsec = SUP_TICK_NS;
    its.it_interval.tv_nsec = SUP_TICK_NS;
    s->tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK);
    if (s->tfd < 0 || timerfd_settime(s->tfd, 0, &its, NULL) < 0 ||
        epoll_add(s->epfd, s->tfd, TAG_TIMER) < 0)
        return -1;
    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr, "usage: %s [--run-dir DIR] [--log PATH] [--bin-dir DIR] [--no-watchdog]\n"
                    "  --run-dir DIR   socket directory, passed to all children (default %s)\n"
                    "  --log PATH      log file, passed to hub (default ./%s)\n"
                    "  --bin-dir DIR   where hub, imu_proc, cam_proc are (default: this program's dir)\n"
                    "  --no-watchdog   don't kill children for missing heartbeats\n",
            argv0, IPC_DEFAULT_RUN_DIR, SUP_DEFAULT_LOG);
}

int main(int argc, char **argv)
{
    static const struct option opts[] = {
        { "run-dir", required_argument, NULL, 'r' },
        { "log", required_argument, NULL, 'l' },
        { "bin-dir", required_argument, NULL, 'b' },
        { "no-watchdog", no_argument, NULL, 'n' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };
    static const char *names[NCHILD] = { "hub", "imu_proc", "cam_proc" };
    static struct sup s; /* large (event queue); keep off the stack */
    char self_dir[PATH_MAX];
    const char *bin_dir = NULL;
    int64_t now;
    int done = 0, exit_code = 0;
    int opt;

    s.run_dir = IPC_DEFAULT_RUN_DIR;
    s.log_path = SUP_DEFAULT_LOG;
    s.watchdog = 1;
    s.epfd = s.sfd = s.tfd = -1;

    while ((opt = getopt_long(argc, argv, "r:l:b:nh", opts, NULL)) != -1) {
        switch (opt) {
        case 'r':
            s.run_dir = optarg;
            break;
        case 'l':
            s.log_path = optarg;
            break;
        case 'b':
            bin_dir = optarg;
            break;
        case 'n':
            s.watchdog = 0;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 2;
        }
    }
    if (!bin_dir) {
        if (exe_dir(self_dir, sizeof self_dir) < 0) {
            perror("supervisor: /proc/self/exe");
            return 1;
        }
        bin_dir = self_dir;
    }

    s.self = getpid();
    if (ipc_client_init(&s.ipc, s.run_dir, "sup.sock", "supervisor") < 0) {
        fprintf(stderr, "supervisor: bad --run-dir %s: %s\n", s.run_dir, strerror(errno));
        return 1;
    }
    for (int i = 0; i < NCHILD; i++) {
        s.kids[i].name = names[i];
        s.kids[i].hb_fd = -1;
        s.kids[i].start_at_ns = -1;
    }
    if (setup(&s, bin_dir) < 0) {
        fprintf(stderr, "supervisor: setup failed: %s\n", strerror(errno));
        return 1;
    }

    now = mono_ns();
    sup_event(&s, "SUP_START", "ver=%d pid=%ld bin_dir=%s run_dir=%s watchdog=%s",
              MSG_VERSION, (long)s.self, bin_dir, s.run_dir, s.watchdog ? "on" : "off");
    /* hub first, so the sensors' first events have somewhere to go. */
    s.kids[0].start_at_ns = now;
    s.kids[1].start_at_ns = now + SUP_SENSOR_DELAY_NS;
    s.kids[2].start_at_ns = now + SUP_SENSOR_DELAY_NS;
    spawn(&s, &s.kids[0], now);

    while (!done) {
        struct epoll_event evs[8];
        int n = epoll_wait(s.epfd, evs, 8, -1);

        if (n < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "supervisor: epoll_wait: %s\n", strerror(errno));
            exit_code = 1;
            begin_stop(&s, mono_ns());
            /* Fall back to polling the tick by hand. */
            while (!on_tick(&s, mono_ns())) {
                struct timespec ts = { 0, SUP_TICK_NS };

                nanosleep(&ts, NULL);
                reap(&s);
            }
            break;
        }
        for (int k = 0; k < n; k++) {
            uint32_t tag = evs[k].data.u32;

            if (tag == TAG_SIGNAL) {
                struct signalfd_siginfo si;

                while (read(s.sfd, &si, sizeof si) == (ssize_t)sizeof si) {
                    if (si.ssi_signo == SIGCHLD)
                        reap(&s);
                    else if (!s.stopping)
                        begin_stop(&s, mono_ns());
                }
                reap(&s); /* SIGCHLDs can merge; reap everything that's ready */
            } else if (tag == TAG_TIMER) {
                uint64_t expirations;

                if (read(s.tfd, &expirations, sizeof expirations) < 0 && errno != EAGAIN)
                    fprintf(stderr, "supervisor: timerfd read: %s\n", strerror(errno));
                done = on_tick(&s, mono_ns());
            } else if ((tag & 0xff00u) == TAG_HB) {
                unsigned i = tag & 0xffu;

                if (i < NCHILD && s.kids[i].hb_fd >= 0)
                    read_heartbeat(&s, &s.kids[i]);
            }
        }
    }

    sup_event(&s, "SUP_STOP", "reason=%s restarts=%u/%u/%u events_dropped=%llu",
              exit_code ? "fatal" : "signal", s.kids[0].restarts, s.kids[1].restarts,
              s.kids[2].restarts, (unsigned long long)s.q_dropped);
    ipc_client_close(&s.ipc);
    close(s.tfd);
    close(s.sfd);
    close(s.epfd);
    return exit_code;
}
