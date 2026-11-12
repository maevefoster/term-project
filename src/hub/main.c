/* hub (Step 1, minimal): receives IMU_DATA, CAM_DATA and EVENT messages from
 * imu_proc and cam_proc, decides once per second which sensors are usable,
 * and is the only process that writes the application log.
 *
 * One SOCK_SEQPACKET listening socket per source (<run_dir>/imu.sock and
 * cam.sock); the socket a message arrives on identifies its sender. A new
 * connection from a source replaces the old one, so a restarted sensor
 * process simply reconnects. Posture decisions (fusion) come later.
 */
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/un.h>
#include <unistd.h>

#include "applog.h"
#include "hub_state.h"
#include "ipc_client.h"
#include "msg.h"
#include "procstat.h"
#include "timeutil.h"

#define HUB_DEFAULT_LOG  "posture.log"
#define HUB_SUMMARY_S    60   /* design: 1/min summary */
#define HUB_HEALTH_S     3600 /* design: hourly heartbeat */
#define HUB_MAX_EVENTS   8
#define HUB_RECV_BATCH   64   /* messages per wakeup per client, so one can't starve the rest */

enum {
    TAG_LISTEN = 0x100, /* | link index */
    TAG_CONN = 0x200,   /* | link index */
    TAG_TIMER = 0x300,
    TAG_SIGNAL = 0x400,
};
#define TAG_KIND(t)  ((t) & 0xff00u)
#define TAG_INDEX(t) ((t) & 0x00ffu)

struct hub_link {
    const char *sock_name;
    const char *proc;  /* sender's name in the log */
    const char *label; /* short name in hub records */
    enum msg_source src;
    char path[IPC_PATH_MAX];
    int listen_fd;
    int fd;
    int reject_logged; /* first reject on this connection already logged */
    struct hub_source st;
    struct msg_imu_data last_imu; /* latest data, kept for fusion later */
    struct msg_cam_data last_cam;
};

struct hub {
    struct applog log;
    int epfd, tfd, sfd;
    struct hub_link links[2];
    enum hub_mode mode;
    uint64_t mode_changes;
};

static int epoll_add(int epfd, int fd, uint32_t tag)
{
    struct epoll_event ev;

    memset(&ev, 0, sizeof ev);
    ev.events = EPOLLIN;
    ev.data.u32 = tag;
    return epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
}

/* Binds <run_dir>/<sock_name>, removing a leftover socket from an earlier
 * run but never any other kind of file. */
static int listen_on(struct hub_link *l, const char *run_dir)
{
    struct sockaddr_un addr;
    struct stat st;
    int n, fd, saved;

    n = snprintf(l->path, sizeof l->path, "%s/%s", run_dir, l->sock_name);
    if (n < 0 || (size_t)n >= sizeof l->path) {
        errno = ENAMETOOLONG;
        return -1;
    }
    if (lstat(l->path, &st) == 0) {
        if (!S_ISSOCK(st.st_mode)) {
            errno = EEXIST;
            return -1;
        }
        if (unlink(l->path) < 0)
            return -1;
    }

    fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (fd < 0)
        return -1;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, l->path, sizeof l->path);
    if (bind(fd, (const struct sockaddr *)&addr, sizeof addr) < 0 || listen(fd, 4) < 0)
        goto fail;
    l->listen_fd = fd;
    return 0;

fail:
    saved = errno;
    close(fd);
    errno = saved;
    return -1;
}

static void drop_conn(struct hub *h, int i, const char *reason)
{
    struct hub_link *l = &h->links[i];

    if (l->fd < 0)
        return;
    (void)epoll_ctl(h->epfd, EPOLL_CTL_DEL, l->fd, NULL);
    if (close(l->fd) < 0)
        applog_write(&h->log, "HUB_ERROR", "op=close source=%s errno=%d", l->label, errno);
    l->fd = -1;
    hub_source_disconnect(&l->st);
    applog_write(&h->log, "HUB_CLIENT", "source=%s state=disconnected reason=%s",
                 l->label, reason);
}

static void accept_conn(struct hub *h, int i)
{
    struct hub_link *l = &h->links[i];
    int fd = accept4(l->listen_fd, NULL, NULL, SOCK_CLOEXEC | SOCK_NONBLOCK);

    if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR &&
            errno != ECONNABORTED)
            applog_write(&h->log, "HUB_ERROR", "op=accept source=%s errno=%d",
                         l->label, errno);
        return;
    }
    drop_conn(h, i, "replaced");
    if (epoll_add(h->epfd, fd, TAG_CONN | (uint32_t)i) < 0) {
        applog_write(&h->log, "HUB_ERROR", "op=epoll_add source=%s errno=%d",
                     l->label, errno);
        close(fd);
        return;
    }
    l->fd = fd;
    l->reject_logged = 0;
    hub_source_connect(&l->st);
    applog_write(&h->log, "HUB_CLIENT", "source=%s state=connected", l->label);
}

static void handle_msg(struct hub *h, struct hub_link *l, const void *buf, size_t len,
                       int64_t now)
{
    const char *why = len > MSG_MAX_SIZE ? "length" : msg_validate(buf, len, l->src);
    struct msg_hdr hdr;

    if (why) {
        l->st.rejects++;
        if (!l->reject_logged) {
            /* Once per connection; later rejects are only counted. */
            applog_write(&h->log, "HUB_REJECT", "source=%s reason=%s len=%zu",
                         l->label, why, len);
            l->reject_logged = 1;
        }
        return;
    }
    memcpy(&hdr, buf, sizeof hdr);
    hub_source_note_seq(&l->st, hdr.seq);
    l->st.msgs++;

    switch (hdr.type) {
    case MSG_IMU_DATA:
        memcpy(&l->last_imu, buf, sizeof l->last_imu);
        /* pitch_valid isn't required until the filter exists. */
        hub_source_data(&l->st, now, (enum sensor_status)hdr.status,
                        hdr.status == SENSOR_OK);
        break;
    case MSG_CAM_DATA:
        memcpy(&l->last_cam, buf, sizeof l->last_cam);
        hub_source_data(&l->st, now, (enum sensor_status)hdr.status,
                        hdr.status == SENSOR_OK && l->last_cam.cva_valid);
        break;
    case MSG_EVENT: {
        struct msg_event e;

        memcpy(&e, buf, sizeof e);
        applog_write_as(&h->log, l->proc, e.name, "%s", e.body);
        break;
    }
    default:
        break; /* unreachable: msg_validate checked the type */
    }
}

static void read_conn(struct hub *h, int i)
{
    struct hub_link *l = &h->links[i];
    union {
        struct msg_event ev; /* largest message; also sets alignment */
        unsigned char raw[MSG_MAX_SIZE];
    } buf;

    for (int k = 0; k < HUB_RECV_BATCH && l->fd >= 0; k++) {
        /* MSG_TRUNC: returns the real length, so an oversized message is
         * seen as oversized rather than silently cut. */
        ssize_t n = recv(l->fd, &buf, sizeof buf, MSG_DONTWAIT | MSG_TRUNC);

        if (n == 0) {
            drop_conn(h, i, "eof");
            return;
        }
        if (n < 0) {
            if (errno == EINTR)
                continue;
            if (errno != EAGAIN && errno != EWOULDBLOCK)
                drop_conn(h, i, "recv_error");
            return;
        }
        handle_msg(h, l, &buf, (size_t)n, mono_ns());
    }
}

static void on_tick(struct hub *h, int64_t now, int64_t start, int64_t *next_summary,
                    int64_t *next_health)
{
    struct hub_link *imu = &h->links[MSG_SRC_IMU];
    struct hub_link *cam = &h->links[MSG_SRC_CAM];
    enum hub_mode m = hub_mode_eval(hub_source_fresh(&imu->st, now),
                                    hub_source_fresh(&cam->st, now));

    if (m != h->mode) {
        applog_write(&h->log, "HUB_MODE", "from=%s to=%s imu=%s cam=%s",
                     hub_mode_name(h->mode), hub_mode_name(m),
                     hub_source_why(&imu->st, now), hub_source_why(&cam->st, now));
        h->mode = m;
        h->mode_changes++;
    }
    if (now >= *next_summary) {
        applog_write(&h->log, "HUB_SUMMARY",
                     "mode=%s imu=%s imu_msgs=%llu imu_rejects=%llu imu_gaps=%llu "
                     "cam=%s cam_msgs=%llu cam_rejects=%llu cam_gaps=%llu",
                     hub_mode_name(h->mode), hub_source_why(&imu->st, now),
                     (unsigned long long)imu->st.msgs, (unsigned long long)imu->st.rejects,
                     (unsigned long long)imu->st.seq_gaps, hub_source_why(&cam->st, now),
                     (unsigned long long)cam->st.msgs, (unsigned long long)cam->st.rejects,
                     (unsigned long long)cam->st.seq_gaps);
        *next_summary += HUB_SUMMARY_S * NS_PER_S;
        if (*next_summary <= now)
            *next_summary = now + HUB_SUMMARY_S * NS_PER_S;
    }
    if (now >= *next_health) {
        applog_write(&h->log, "HEALTH",
                     "uptime_s=%lld rss_kb=%ld mode=%s mode_changes=%llu log_failures=%lu",
                     (long long)((now - start) / NS_PER_S), proc_rss_kb(),
                     hub_mode_name(h->mode), (unsigned long long)h->mode_changes,
                     h->log.write_failures);
        *next_health += HUB_HEALTH_S * NS_PER_S;
        if (*next_health <= now)
            *next_health = now + HUB_HEALTH_S * NS_PER_S;
    }
}

static int setup_timer_and_signals(struct hub *h)
{
    struct itimerspec its;
    sigset_t mask;

    memset(&its, 0, sizeof its);
    its.it_value.tv_sec = 1;
    its.it_interval.tv_sec = 1;
    h->tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK);
    if (h->tfd < 0 || timerfd_settime(h->tfd, 0, &its, NULL) < 0)
        return -1;

    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    if (sigprocmask(SIG_BLOCK, &mask, NULL) < 0)
        return -1;
    h->sfd = signalfd(-1, &mask, SFD_CLOEXEC | SFD_NONBLOCK);
    if (h->sfd < 0)
        return -1;

    if (epoll_add(h->epfd, h->tfd, TAG_TIMER) < 0 || epoll_add(h->epfd, h->sfd, TAG_SIGNAL) < 0)
        return -1;
    return 0;
}

static void close_all(struct hub *h)
{
    for (int i = 0; i < 2; i++) {
        struct hub_link *l = &h->links[i];

        if (l->fd >= 0)
            close(l->fd);
        if (l->listen_fd >= 0) {
            close(l->listen_fd);
            if (unlink(l->path) < 0 && errno != ENOENT)
                fprintf(stderr, "hub: unlink %s: %s\n", l->path, strerror(errno));
        }
        l->fd = l->listen_fd = -1;
    }
    if (h->sfd >= 0)
        close(h->sfd);
    if (h->tfd >= 0)
        close(h->tfd);
    if (h->epfd >= 0)
        close(h->epfd);
    h->sfd = h->tfd = h->epfd = -1;
}

static void usage(const char *argv0)
{
    fprintf(stderr, "usage: %s [--run-dir DIR] [--log PATH]\n"
                    "  --run-dir DIR  socket directory (default %s)\n"
                    "  --log PATH     application log (default ./%s)\n",
            argv0, IPC_DEFAULT_RUN_DIR, HUB_DEFAULT_LOG);
}

int main(int argc, char **argv)
{
    static const struct option opts[] = {
        { "run-dir", required_argument, NULL, 'r' },
        { "log", required_argument, NULL, 'l' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };
    struct hub h;
    const char *run_dir = IPC_DEFAULT_RUN_DIR;
    const char *log_path = HUB_DEFAULT_LOG;
    int64_t start, next_summary, next_health;
    int stop = 0, exit_code = 0;
    int opt;

    memset(&h, 0, sizeof h);
    h.epfd = h.tfd = h.sfd = -1;
    h.mode = HUB_MODE_NO_SENSORS;
    h.links[MSG_SRC_IMU] = (struct hub_link){ .sock_name = "imu.sock", .proc = "imu_proc",
                                              .label = "imu", .src = MSG_SRC_IMU,
                                              .listen_fd = -1, .fd = -1 };
    h.links[MSG_SRC_CAM] = (struct hub_link){ .sock_name = "cam.sock", .proc = "cam_proc",
                                              .label = "cam", .src = MSG_SRC_CAM,
                                              .listen_fd = -1, .fd = -1 };

    while ((opt = getopt_long(argc, argv, "r:l:h", opts, NULL)) != -1) {
        switch (opt) {
        case 'r':
            run_dir = optarg;
            break;
        case 'l':
            log_path = optarg;
            break;
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 2;
        }
    }

    if (applog_open(&h.log, log_path, "hub", APPLOG_CAP) < 0) {
        fprintf(stderr, "hub: cannot open log %s: %s\n", log_path, strerror(errno));
        return 1;
    }
    if (mkdir(run_dir, 0750) < 0 && errno != EEXIST) {
        fprintf(stderr, "hub: cannot create %s: %s\n"
                        "  for a run started by hand: sudo mkdir -p %s && sudo chown $USER %s\n",
                run_dir, strerror(errno), run_dir, run_dir);
        applog_write(&h.log, "HUB_FATAL", "op=mkdir run_dir=%s errno=%d", run_dir, errno);
        applog_close(&h.log);
        return 1;
    }

    h.epfd = epoll_create1(EPOLL_CLOEXEC);
    if (h.epfd < 0 || setup_timer_and_signals(&h) < 0) {
        applog_write(&h.log, "HUB_FATAL", "op=setup errno=%d", errno);
        exit_code = 1;
        goto out;
    }
    for (int i = 0; i < 2; i++) {
        struct hub_link *l = &h.links[i];

        if (listen_on(l, run_dir) < 0 ||
            epoll_add(h.epfd, l->listen_fd, TAG_LISTEN | (uint32_t)i) < 0) {
            fprintf(stderr, "hub: cannot listen on %s: %s\n", l->path, strerror(errno));
            applog_write(&h.log, "HUB_FATAL", "op=listen path=%s errno=%d", l->path, errno);
            exit_code = 1;
            goto out;
        }
    }

    start = mono_ns();
    next_summary = start + HUB_SUMMARY_S * NS_PER_S;
    next_health = start + HUB_HEALTH_S * NS_PER_S;
    applog_write(&h.log, "HUB_START", "ver=%d pid=%ld run_dir=%s stale_ms=%lld mode=%s log_cap=%lld",
                 MSG_VERSION, (long)getpid(), run_dir, (long long)(HUB_STALE_NS / NS_PER_MS),
                 hub_mode_name(h.mode), (long long)h.log.cap);

    while (!stop) {
        struct epoll_event evs[HUB_MAX_EVENTS];
        int n = epoll_wait(h.epfd, evs, HUB_MAX_EVENTS, -1);

        if (n < 0) {
            if (errno == EINTR)
                continue;
            applog_write(&h.log, "HUB_FATAL", "op=epoll_wait errno=%d", errno);
            exit_code = 1;
            break;
        }
        for (int k = 0; k < n; k++) {
            uint32_t tag = evs[k].data.u32;
            int i = (int)TAG_INDEX(tag);

            switch (TAG_KIND(tag)) {
            case TAG_LISTEN:
                accept_conn(&h, i);
                break;
            case TAG_CONN:
                read_conn(&h, i);
                break;
            case TAG_TIMER: {
                uint64_t expirations;

                if (read(h.tfd, &expirations, sizeof expirations) < 0 && errno != EAGAIN)
                    applog_write(&h.log, "HUB_ERROR", "op=timerfd_read errno=%d", errno);
                on_tick(&h, mono_ns(), start, &next_summary, &next_health);
                break;
            }
            case TAG_SIGNAL: {
                struct signalfd_siginfo si;

                if (read(h.sfd, &si, sizeof si) == (ssize_t)sizeof si)
                    stop = 1;
                break;
            }
            default:
                break;
            }
        }
    }

    applog_write(&h.log, "HUB_STOP", "reason=%s mode=%s mode_changes=%llu log_failures=%lu",
                 exit_code ? "fatal" : "signal", hub_mode_name(h.mode),
                 (unsigned long long)h.mode_changes, h.log.write_failures);
out:
    close_all(&h);
    applog_close(&h.log);
    return exit_code;
}
