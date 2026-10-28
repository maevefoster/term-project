/* cam_proc (M3): standalone V4L2 capture with bounded logging.
 *
 * Captures 320x240 YUYV at 5 fps into mmap buffers, keeps frames in RAM
 * only, and appends status/summary records to a single log file. An
 * unplugged camera is reported as ABSENT and retried every 5 s; the
 * process never exits for a missing device.
 */
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "applog.h"
#include "frame_stats.h"
#include "markers.h"
#include "msg.h"
#include "timeutil.h"
#include "v4l2_cap.h"

#define CAM_DEFAULT_DEV  "/dev/video0"
#define CAM_DEFAULT_LOG  "posture.log"
#define CAM_WIDTH        320
#define CAM_HEIGHT       240
#define CAM_FPS          5
#define CAM_POLL_MS      1000           /* one dequeue wait; a timeout is a stall */
#define CAM_STALL_LIMIT  5              /* consecutive stalls before reopening */
#define CAM_RETRY_NS     (5 * NS_PER_S) /* ABSENT retry period (CLAUDE.md section 3) */
#define CAM_IDLE_MS      250            /* sleep slice while waiting to retry */
#define CAM_HEALTH_S     3600

/* Placeholder until the team decides (CLAUDE.md section 8). */
#ifndef CAM_SUMMARY_S
#define CAM_SUMMARY_S 10
#endif

struct cam_ctx {
    struct applog log;
    struct cam_dev cam;
    struct frame_stats stats;
    const char *dev_path;
    enum sensor_status status;
    int64_t absent_since_ns; /* -1 while not absent */
    unsigned stall_streak;
    uint64_t open_failures;  /* since the device went absent */
    uint64_t frames_total;
    uint64_t absent_events;
};

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static int install_signals(void)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal; /* no SA_RESTART: poll/nanosleep return EINTR */
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGINT, &sa, NULL) < 0 || sigaction(SIGTERM, &sa, NULL) < 0)
        return -1;
    return 0;
}

static void fourcc_str(uint32_t f, char out[5])
{
    for (int i = 0; i < 4; i++) {
        char c = (char)((f >> (8 * i)) & 0xff);

        out[i] = (c >= 0x20 && c < 0x7f && c != ' ') ? c : '?';
    }
    out[4] = '\0';
}

static double ns_to_ms(int64_t ns)
{
    return ns < 0 ? -1.0 : (double)ns / (double)NS_PER_MS;
}

static long rss_kb(void)
{
    long pages = -1;
    long page = sysconf(_SC_PAGESIZE);
    FILE *f = fopen("/proc/self/statm", "re");

    if (!f)
        return -1;
    if (fscanf(f, "%*s %ld", &pages) != 1)
        pages = -1;
    fclose(f);
    return (pages < 0 || page <= 0) ? -1 : pages * (page / 1024);
}

static void on_frame(const struct cam_frame *f, void *arg)
{
    struct cam_ctx *c = arg;
    int bad = (f->flags & V4L2_BUF_FLAG_ERROR) || f->bytesused < c->cam.sizeimage;

    fstats_frame(&c->stats, f->sequence, ts_ns(&f->mono), bad);
    c->frames_total++;
    c->stall_streak = 0;
    if (!bad)
        markers_on_frame(f->data, f->bytesused, c->cam.width, c->cam.height,
                         c->cam.bytesperline);
}

static void mark_absent(struct cam_ctx *c, int64_t now)
{
    c->status = SENSOR_ABSENT;
    if (c->absent_since_ns >= 0)
        return; /* already reported; retries are counted, not logged */
    c->absent_since_ns = now;
    c->absent_events++;
    c->open_failures = 0;
    applog_write(&c->log, "CAM_ABSENT", "dev=%s op=%s errno=%d",
                 c->dev_path, c->cam.last_op, c->cam.last_errno);
}

/* Returns 0 when streaming, 1 when absent (retry later), -1 when fatal. */
static int try_open(struct cam_ctx *c)
{
    enum cam_result r = cam_open(&c->cam, c->dev_path, CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
    int64_t now = mono_ns();
    char fcc[5];

    if (r == CAM_GONE) {
        mark_absent(c, now);
        c->open_failures++;
        return 1;
    }
    if (r != CAM_OK) {
        applog_write(&c->log, "CAM_FATAL", "dev=%s op=%s errno=%d",
                     c->dev_path, c->cam.last_op, c->cam.last_errno);
        return -1;
    }

    if (c->absent_since_ns >= 0)
        applog_write(&c->log, "CAM_RECONNECTED", "dev=%s down_ms=%lld open_failures=%llu",
                     c->dev_path, (long long)((now - c->absent_since_ns) / NS_PER_MS),
                     (unsigned long long)c->open_failures);
    fourcc_str(c->cam.pixfmt, fcc);
    applog_write(&c->log, "CAM_OPEN",
                 "dev=%s fmt=%s w=%u h=%u bpl=%u sizeimage=%u tpf=%u/%u nbufs=%u",
                 c->dev_path, fcc, (unsigned)c->cam.width, (unsigned)c->cam.height,
                 (unsigned)c->cam.bytesperline, (unsigned)c->cam.sizeimage,
                 (unsigned)c->cam.tpf_num, (unsigned)c->cam.tpf_den, c->cam.n_bufs);
    if (c->cam.width != CAM_WIDTH || c->cam.height != CAM_HEIGHT)
        applog_write(&c->log, "CAM_FORMAT_MISMATCH", "want=%ux%u got=%ux%u",
                     (unsigned)CAM_WIDTH, (unsigned)CAM_HEIGHT,
                     (unsigned)c->cam.width, (unsigned)c->cam.height);

    c->status = SENSOR_OK;
    c->absent_since_ns = -1;
    c->stall_streak = 0;
    fstats_new_stream(&c->stats);
    return 0;
}

static void log_summary(struct cam_ctx *c, int64_t now)
{
    struct frame_summary s;

    fstats_summary(&c->stats, now, &s);
    applog_write(&c->log, "CAM_SUMMARY",
                 "status=%s window_ms=%lld frames=%llu fps=%.2f iv_min_ms=%.1f "
                 "iv_p50_ms=%.1f iv_max_ms=%.1f dropped=%llu stalls=%llu bad=%llu "
                 "open_failures=%llu",
                 sensor_status_name(c->status), (long long)(s.window_ns / NS_PER_MS),
                 (unsigned long long)s.frames, s.fps, ns_to_ms(s.iv_min_ns),
                 ns_to_ms(s.iv_p50_ns), ns_to_ms(s.iv_max_ns),
                 (unsigned long long)s.dropped, (unsigned long long)s.stalls,
                 (unsigned long long)s.bad_frames, (unsigned long long)c->open_failures);
}

static void log_health(struct cam_ctx *c, int64_t uptime_ns)
{
    applog_write(&c->log, "HEALTH",
                 "uptime_s=%lld rss_kb=%ld status=%s frames_total=%llu "
                 "absent_events=%llu log_failures=%lu",
                 (long long)(uptime_ns / NS_PER_S), rss_kb(), sensor_status_name(c->status),
                 (unsigned long long)c->frames_total, (unsigned long long)c->absent_events,
                 c->log.write_failures);
}

static int64_t next_deadline(int64_t deadline, int64_t now, int64_t period)
{
    deadline += period;
    return deadline > now ? deadline : now + period;
}

static void sleep_ms(long ms)
{
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * NS_PER_MS };

    (void)nanosleep(&ts, NULL); /* EINTR is fine: the loop rechecks g_stop */
}

static void usage(const char *argv0)
{
    fprintf(stderr, "usage: %s [--dev PATH] [--log PATH]\n"
                    "  --dev PATH  V4L2 capture device (default %s)\n"
                    "  --log PATH  application log (default ./%s)\n",
            argv0, CAM_DEFAULT_DEV, CAM_DEFAULT_LOG);
}

int main(int argc, char **argv)
{
    static const struct option opts[] = {
        { "dev", required_argument, NULL, 'd' },
        { "log", required_argument, NULL, 'l' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };
    struct cam_ctx c;
    const char *log_path = CAM_DEFAULT_LOG;
    int64_t start, next_summary, next_health, next_retry = 0;
    int exit_code = 0;
    int opt;

    memset(&c, 0, sizeof c);
    c.dev_path = CAM_DEFAULT_DEV;
    c.status = SENSOR_ABSENT;
    c.absent_since_ns = -1;
    cam_init(&c.cam);

    while ((opt = getopt_long(argc, argv, "d:l:h", opts, NULL)) != -1) {
        switch (opt) {
        case 'd':
            c.dev_path = optarg;
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

    if (install_signals() < 0) {
        perror("cam_proc: sigaction");
        return 1;
    }
    if (applog_open(&c.log, log_path, "cam_proc", APPLOG_CAP) < 0) {
        fprintf(stderr, "cam_proc: cannot open log %s: %s\n", log_path, strerror(errno));
        return 1;
    }

    start = mono_ns();
    fstats_new_window(&c.stats, start);
    next_summary = start + CAM_SUMMARY_S * NS_PER_S;
    next_health = start + CAM_HEALTH_S * NS_PER_S;
    applog_write(&c.log, "CAM_START",
                 "ver=%d pid=%ld dev=%s want=%ux%u@%u summary_s=%d log_cap=%lld",
                 MSG_VERSION, (long)getpid(), c.dev_path, (unsigned)CAM_WIDTH,
                 (unsigned)CAM_HEIGHT, (unsigned)CAM_FPS, CAM_SUMMARY_S,
                 (long long)c.log.cap);

    while (!g_stop) {
        int64_t now = mono_ns();

        if (c.cam.fd < 0) {
            if (now >= next_retry) {
                int r = try_open(&c);

                if (r < 0) {
                    exit_code = 1;
                    break;
                }
                if (r > 0)
                    next_retry = now + CAM_RETRY_NS;
            } else {
                sleep_ms(CAM_IDLE_MS);
            }
        } else {
            switch (cam_next_frame(&c.cam, CAM_POLL_MS, on_frame, &c)) {
            case CAM_OK:
            case CAM_AGAIN:
                break;
            case CAM_TIMEOUT:
                fstats_stall(&c.stats);
                if (++c.stall_streak == 1)
                    applog_write(&c.log, "CAM_STALL", "dev=%s timeout_ms=%d",
                                 c.dev_path, CAM_POLL_MS);
                if (c.stall_streak >= CAM_STALL_LIMIT) {
                    /* Open but silent: report ABSENT and reopen right away. */
                    cam_close(&c.cam);
                    c.cam.last_op = "stall";
                    c.cam.last_errno = 0;
                    mark_absent(&c, mono_ns());
                    next_retry = 0;
                }
                break;
            case CAM_GONE:
            case CAM_FATAL:
                mark_absent(&c, mono_ns());
                next_retry = mono_ns() + CAM_RETRY_NS;
                break;
            }
        }

        now = mono_ns();
        if (now >= next_summary) {
            log_summary(&c, now);
            fstats_new_window(&c.stats, now);
            next_summary = next_deadline(next_summary, now, CAM_SUMMARY_S * NS_PER_S);
        }
        if (now >= next_health) {
            log_health(&c, now - start);
            next_health = next_deadline(next_health, now, CAM_HEALTH_S * NS_PER_S);
        }
    }

    cam_close(&c.cam);
    applog_write(&c.log, "CAM_STOP", "reason=%s frames_total=%llu absent_events=%llu log_failures=%lu",
                 exit_code ? "fatal" : "signal", (unsigned long long)c.frames_total,
                 (unsigned long long)c.absent_events, c.log.write_failures);
    applog_close(&c.log);
    return exit_code;
}
