/* Host unit tests for src/common/heartbeat.c, using a real pipe in place of
 * the supervisor. */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "heartbeat.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

/* Bytes waiting in the (non-blocking) read end. */
static int drain(int rfd)
{
    char buf[64];
    int total = 0;
    ssize_t n;

    while ((n = read(rfd, buf, sizeof buf)) > 0)
        total += (int)n;
    return total;
}

static void test_disabled(void)
{
    struct heartbeat hb;

    unsetenv(HEARTBEAT_ENV);
    CHECK(heartbeat_init(&hb, "test") == 0);
    CHECK(hb.fd == -1);
    heartbeat_beat(&hb, 0); /* must be a no-op */
    CHECK(hb.sent == 0);

    setenv(HEARTBEAT_ENV, "abc", 1);
    CHECK(heartbeat_init(&hb, "test") == 0);
    setenv(HEARTBEAT_ENV, "1", 1); /* stdout is never a heartbeat fd */
    CHECK(heartbeat_init(&hb, "test") == 0);
    setenv(HEARTBEAT_ENV, "999", 1); /* not open */
    CHECK(heartbeat_init(&hb, "test") == 0);
    unsetenv(HEARTBEAT_ENV);
}

static void test_beats(void)
{
    struct heartbeat hb;
    char num[16];
    int p[2];

    CHECK(pipe(p) == 0);
    CHECK(fcntl(p[0], F_SETFL, O_NONBLOCK) == 0);
    snprintf(num, sizeof num, "%d", p[1]);
    setenv(HEARTBEAT_ENV, num, 1);

    CHECK(heartbeat_init(&hb, "test") == 1);
    CHECK(hb.fd == p[1]);

    heartbeat_beat(&hb, 10 * NS_PER_S);
    CHECK(drain(p[0]) == 1);
    heartbeat_beat(&hb, 10 * NS_PER_S + 500 * NS_PER_MS); /* too soon */
    CHECK(drain(p[0]) == 0);
    heartbeat_beat(&hb, 10 * NS_PER_S + HEARTBEAT_PERIOD_NS);
    CHECK(drain(p[0]) == 1);
    CHECK(hb.sent == 2);

    /* Supervisor gone: the write fails with EPIPE (SIGPIPE is ignored after
     * heartbeat_init), heartbeats switch off, and the process lives on. */
    close(p[0]);
    heartbeat_beat(&hb, 20 * NS_PER_S);
    CHECK(hb.fd == -1);
    heartbeat_beat(&hb, 30 * NS_PER_S);
    CHECK(hb.sent == 2);
    unsetenv(HEARTBEAT_ENV);
}

int main(void)
{
    test_disabled();
    test_beats();

    if (failures) {
        fprintf(stderr, "test_heartbeat: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_heartbeat: all checks passed\n");
    return 0;
}
