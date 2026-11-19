/* Host unit tests for src/supervisor/sup_policy.c (no processes, no clocks). */
#include <stdio.h>

#include "sup_policy.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static void test_backoff(void)
{
    CHECK(sup_backoff_ns(0) == NS_PER_S);  /* never below the first delay */
    CHECK(sup_backoff_ns(1) == NS_PER_S);
    CHECK(sup_backoff_ns(2) == 2 * NS_PER_S);
    CHECK(sup_backoff_ns(3) == 4 * NS_PER_S);
    CHECK(sup_backoff_ns(4) == 8 * NS_PER_S);
    CHECK(sup_backoff_ns(5) == 16 * NS_PER_S);
    CHECK(sup_backoff_ns(6) == 30 * NS_PER_S); /* 32 s capped */
    CHECK(sup_backoff_ns(1000) == 30 * NS_PER_S);
}

static void test_failures(void)
{
    unsigned f = 0;

    /* Quick crashes keep counting up. */
    f = sup_next_failures(f, 2 * NS_PER_S);
    CHECK(f == 1);
    f = sup_next_failures(f, 2 * NS_PER_S);
    CHECK(f == 2);
    f = sup_next_failures(f, SUP_STABLE_NS - 1);
    CHECK(f == 3);

    /* A child that ran long enough starts over at the first delay. */
    f = sup_next_failures(f, SUP_STABLE_NS);
    CHECK(f == 1);
    CHECK(sup_backoff_ns(f) == NS_PER_S);

    /* The count stays bounded. */
    for (int i = 0; i < 1000; i++)
        f = sup_next_failures(f, 0);
    CHECK(f >= 6 && f <= 64);
    CHECK(sup_backoff_ns(f) == 30 * NS_PER_S);
}

static void test_watchdog(void)
{
    const int64_t start = 100 * NS_PER_S;

    /* Before the first heartbeat: 5 s from start. */
    CHECK(sup_watchdog_deadline(start, 0, 0) == start + 5 * NS_PER_S);
    /* After it: 3 s from the latest heartbeat. */
    CHECK(sup_watchdog_deadline(start, 1, start + NS_PER_S) == start + 4 * NS_PER_S);
    CHECK(sup_watchdog_deadline(start, 1, start + 60 * NS_PER_S) == start + 63 * NS_PER_S);
}

int main(void)
{
    test_backoff();
    test_failures();
    test_watchdog();

    if (failures) {
        fprintf(stderr, "test_sup_policy: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_sup_policy: all checks passed\n");
    return 0;
}
