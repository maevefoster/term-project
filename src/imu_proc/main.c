/* imu_proc (basic): reads the MPU-6050 at 100 Hz, shows the current values
 * on the console, sends IMU_DATA to hub at 10 Hz and reports status and
 * summaries to hub as EVENTs (hub writes the log; each event is also printed
 * to stderr).
 *
 * Runs under the default scheduler for now; SCHED_FIFO and the
 * complementary filter come later, so IMU_DATA carries pitch_valid=0. After
 * 5 consecutive failed reads (I2C errors or all-zero data) the sensor is
 * reported ABSENT and retried with a backoff; the process never exits for a
 * missing sensor or a missing hub.
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

#include "ipc_client.h"
#include "mpu6050.h"
#include "msg.h"
#include "procstat.h"
#include "timeutil.h"

#define IMU_DEFAULT_DEV "/dev/i2c-1"
#define IMU_RATE_HZ     100
#define IMU_PERIOD_NS   (NS_PER_S / IMU_RATE_HZ)
#define IMU_REPORT_DIV  10 /* ticks per console update and IMU_DATA: 10 Hz */
#define IMU_ERR_LIMIT   5  /* consecutive failed reads before ABSENT */
#define IMU_HEALTH_S    3600

/* Placeholder until the team decides (CLAUDE.md section 8). */
#ifndef IMU_SUMMARY_S
#define IMU_SUMMARY_S 10
#endif

/* ABSENT retry delays; the last one repeats. Capped at 5 s so a replugged
 * sensor is back well inside the 10 s recovery target. */
static const int64_t retry_delay_s[] = { 1, 2, 4, 5 };
#define IMU_RETRY_STEPS (sizeof retry_delay_s / sizeof retry_delay_s[0])

struct imu_ctx {
    struct ipc_client ipc;
    struct mpu_dev mpu;
    const char *dev_path;
    enum sensor_status status;
    int64_t absent_since_ns; /* -1 while not absent */
    int64_t next_retry_ns;
    size_t retry_step;
    unsigned err_streak;     /* consecutive failed reads */
    uint64_t open_failures;  /* since the sensor went absent */
    uint64_t absent_events;

    struct mpu_reading last; /* valid only while have_last is set */
    int64_t last_sample_ns;  /* CLOCK_MONOTONIC right after the read */
    int have_last;

    /* Current summary window. */
    int64_t window_start_ns;
    uint64_t window_samples;
    uint64_t window_errors;

    uint64_t samples_total;
    uint64_t errors_total;
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
    sa.sa_handler = on_signal; /* no SA_RESTART: clock_nanosleep returns EINTR */
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGINT, &sa, NULL) < 0 || sigaction(SIGTERM, &sa, NULL) < 0)
        return -1;
    return 0;
}

static void schedule_retry(struct imu_ctx *c, int64_t now)
{
    c->next_retry_ns = now + retry_delay_s[c->retry_step] * NS_PER_S;
    if (c->retry_step + 1 < IMU_RETRY_STEPS)
        c->retry_step++;
}

static void mark_absent(struct imu_ctx *c, int64_t now)
{
    c->status = SENSOR_ABSENT;
    c->have_last = 0; /* never show or send a stale reading as current */
    if (c->absent_since_ns >= 0)
        return; /* already reported; retries are counted, not logged */
    c->absent_since_ns = now;
    c->absent_events++;
    c->open_failures = 0;
    c->retry_step = 0;
    ipc_event(&c->ipc, "IMU_ABSENT", "dev=%s i2c_err=%u op=%s errno=%d who_am_i=0x%02x",
              c->dev_path, c->err_streak, c->mpu.last_op, c->mpu.last_errno,
              (unsigned)c->mpu.who_am_i);
}

/* Returns 0 when reading, 1 when absent (retry later), -1 when fatal. */
static int try_open(struct imu_ctx *c)
{
    enum mpu_result r = mpu_open(&c->mpu, c->dev_path, MPU_DEFAULT_ADDR);
    int64_t now = mono_ns();

    if (r == MPU_GONE) {
        mark_absent(c, now);
        c->open_failures++;
        schedule_retry(c, now);
        return 1;
    }
    if (r != MPU_OK) {
        ipc_event(&c->ipc, "IMU_FATAL", "dev=%s op=%s errno=%d",
                  c->dev_path, c->mpu.last_op, c->mpu.last_errno);
        fprintf(stderr, "imu_proc: %s: %s failed: %s\n", c->dev_path,
                c->mpu.last_op, strerror(c->mpu.last_errno));
        return -1;
    }

    if (c->absent_since_ns >= 0)
        ipc_event(&c->ipc, "IMU_RECONNECTED", "dev=%s down_ms=%lld open_failures=%llu",
                  c->dev_path, (long long)((now - c->absent_since_ns) / NS_PER_MS),
                  (unsigned long long)c->open_failures);
    ipc_event(&c->ipc, "IMU_OPEN",
              "dev=%s addr=0x%02x who_am_i=0x%02x accel_fs_g=%d gyro_fs_dps=%d",
              c->dev_path, (unsigned)MPU_DEFAULT_ADDR, (unsigned)c->mpu.who_am_i,
              MPU_ACCEL_FS_G, MPU_GYRO_FS_DPS);

    c->status = SENSOR_OK;
    c->absent_since_ns = -1;
    c->err_streak = 0;
    return 0;
}

static void take_sample(struct imu_ctx *c)
{
    struct mpu_raw raw;

    if (mpu_read_raw(&c->mpu, &raw) < 0) {
        c->window_errors++;
        c->errors_total++;
        if (++c->err_streak >= IMU_ERR_LIMIT) {
            int64_t now = mono_ns();

            mpu_close(&c->mpu);
            mark_absent(c, now);
            c->err_streak = 0;
            schedule_retry(c, now);
        }
        return;
    }
    c->last_sample_ns = mono_ns();
    mpu_scale(&raw, &c->last);
    c->have_last = 1;
    c->err_streak = 0;
    c->window_samples++;
    c->samples_total++;
}

static void send_data(struct imu_ctx *c)
{
    struct msg_imu_data m;

    memset(&m, 0, sizeof m);
    msg_init_hdr(&m.h, MSG_IMU_DATA, c->status);
    m.pitch_valid = 0; /* no complementary filter yet */
    ipc_send(&c->ipc, &m.h, sizeof m);
}

static void show_console(const struct imu_ctx *c, int tty)
{
    const char *end = tty ? "\033[K\r" : "\n";

    if (c->have_last)
        printf("accel[g] %+7.3f %+7.3f %+7.3f  gyro[dps] %+8.2f %+8.2f %+8.2f  temp %5.1fC%s",
               c->last.ax_g, c->last.ay_g, c->last.az_g,
               c->last.gx_dps, c->last.gy_dps, c->last.gz_dps, c->last.temp_c, end);
    else
        printf("IMU %s: no current reading%s", sensor_status_name(c->status), end);
    fflush(stdout);
}

static void log_summary(struct imu_ctx *c, int64_t now)
{
    long long window_ms = (now - c->window_start_ns) / NS_PER_MS;

    if (c->have_last && c->window_samples > 0)
        ipc_event(&c->ipc, "IMU_SUMMARY",
                  "status=%s window_ms=%lld samples=%llu i2c_errors=%llu "
                  "open_failures=%llu age_ms=%lld ax_g=%.3f ay_g=%.3f az_g=%.3f "
                  "gx_dps=%.2f gy_dps=%.2f gz_dps=%.2f temp_c=%.1f",
                  sensor_status_name(c->status), window_ms,
                  (unsigned long long)c->window_samples,
                  (unsigned long long)c->window_errors,
                  (unsigned long long)c->open_failures,
                  (long long)((now - c->last_sample_ns) / NS_PER_MS),
                  c->last.ax_g, c->last.ay_g, c->last.az_g,
                  c->last.gx_dps, c->last.gy_dps, c->last.gz_dps, c->last.temp_c);
    else
        ipc_event(&c->ipc, "IMU_SUMMARY",
                  "status=%s window_ms=%lld samples=%llu i2c_errors=%llu open_failures=%llu",
                  sensor_status_name(c->status), window_ms,
                  (unsigned long long)c->window_samples,
                  (unsigned long long)c->window_errors,
                  (unsigned long long)c->open_failures);
    c->window_start_ns = now;
    c->window_samples = 0;
    c->window_errors = 0;
}

static void log_health(struct imu_ctx *c, int64_t uptime_ns)
{
    ipc_event(&c->ipc, "HEALTH",
              "uptime_s=%lld rss_kb=%ld status=%s samples_total=%llu i2c_errors_total=%llu "
              "absent_events=%llu ipc_dropped=%llu",
              (long long)(uptime_ns / NS_PER_S), proc_rss_kb(), sensor_status_name(c->status),
              (unsigned long long)c->samples_total, (unsigned long long)c->errors_total,
              (unsigned long long)c->absent_events, (unsigned long long)c->ipc.dropped);
}

static int64_t next_deadline(int64_t deadline, int64_t now, int64_t period)
{
    deadline += period;
    return deadline > now ? deadline : now + period;
}

static void sleep_until(int64_t deadline_ns)
{
    struct timespec ts = {
        .tv_sec = (time_t)(deadline_ns / NS_PER_S),
        .tv_nsec = (long)(deadline_ns % NS_PER_S),
    };

    /* EINTR is fine: the loop rechecks g_stop. */
    (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

static void usage(const char *argv0)
{
    fprintf(stderr, "usage: %s [--dev PATH] [--run-dir DIR]\n"
                    "  --dev PATH     I2C bus device (default %s)\n"
                    "  --run-dir DIR  hub socket directory (default %s)\n",
            argv0, IMU_DEFAULT_DEV, IPC_DEFAULT_RUN_DIR);
}

int main(int argc, char **argv)
{
    static const struct option opts[] = {
        { "dev", required_argument, NULL, 'd' },
        { "run-dir", required_argument, NULL, 'r' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };
    struct imu_ctx c;
    const char *run_dir = IPC_DEFAULT_RUN_DIR;
    int64_t start, next_tick, next_summary, next_health;
    unsigned report_tick = 0;
    int exit_code = 0;
    int tty = isatty(STDOUT_FILENO);
    int opt;

    memset(&c, 0, sizeof c);
    c.dev_path = IMU_DEFAULT_DEV;
    c.status = SENSOR_ABSENT;
    c.absent_since_ns = -1;
    mpu_init(&c.mpu);

    while ((opt = getopt_long(argc, argv, "d:r:h", opts, NULL)) != -1) {
        switch (opt) {
        case 'd':
            c.dev_path = optarg;
            break;
        case 'r':
            run_dir = optarg;
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
        perror("imu_proc: sigaction");
        return 1;
    }
    if (ipc_client_init(&c.ipc, run_dir, "imu.sock", "imu_proc") < 0) {
        fprintf(stderr, "imu_proc: bad --run-dir %s: %s\n", run_dir, strerror(errno));
        return 1;
    }

    start = mono_ns();
    c.window_start_ns = start;
    next_tick = start;
    next_summary = start + IMU_SUMMARY_S * NS_PER_S;
    next_health = start + IMU_HEALTH_S * NS_PER_S;
    ipc_event(&c.ipc, "IMU_START", "ver=%d pid=%ld dev=%s addr=0x%02x rate_hz=%d summary_s=%d",
              MSG_VERSION, (long)getpid(), c.dev_path, (unsigned)MPU_DEFAULT_ADDR,
              IMU_RATE_HZ, IMU_SUMMARY_S);

    while (!g_stop) {
        int64_t now;

        sleep_until(next_tick);
        if (g_stop)
            break;

        if (c.mpu.fd >= 0) {
            take_sample(&c);
        } else if (mono_ns() >= c.next_retry_ns && try_open(&c) < 0) {
            exit_code = 1;
            break;
        }

        if (++report_tick >= IMU_REPORT_DIV) {
            report_tick = 0;
            send_data(&c);
            show_console(&c, tty);
        }

        now = mono_ns();
        if (now >= next_summary) {
            log_summary(&c, now);
            next_summary = next_deadline(next_summary, now, IMU_SUMMARY_S * NS_PER_S);
        }
        if (now >= next_health) {
            log_health(&c, now - start);
            next_health = next_deadline(next_health, now, IMU_HEALTH_S * NS_PER_S);
        }

        /* After an overrun, carry on from now rather than bursting to catch up. */
        next_tick = next_deadline(next_tick, now, IMU_PERIOD_NS);
    }

    if (tty)
        putchar('\n');
    mpu_close(&c.mpu);
    ipc_event(&c.ipc, "IMU_STOP",
              "reason=%s samples_total=%llu i2c_errors_total=%llu absent_events=%llu "
              "ipc_sent=%llu ipc_dropped=%llu",
              exit_code ? "fatal" : "signal", (unsigned long long)c.samples_total,
              (unsigned long long)c.errors_total, (unsigned long long)c.absent_events,
              (unsigned long long)c.ipc.sent, (unsigned long long)c.ipc.dropped);
    ipc_client_close(&c.ipc);
    return exit_code;
}
