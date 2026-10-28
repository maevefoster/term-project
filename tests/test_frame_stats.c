/* Host unit tests for src/cam_proc/frame_stats.c (no hardware). */
#include <stdio.h>

#include "frame_stats.h"
#include "timeutil.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static struct frame_stats g_s; /* large; keep off the stack */

static void test_steady_5fps(void)
{
    struct frame_summary sum;
    const int64_t iv = 200 * NS_PER_MS;

    fstats_new_stream(&g_s);
    fstats_new_window(&g_s, 0);
    for (uint32_t i = 0; i < 50; i++)
        fstats_frame(&g_s, i, (int64_t)i * iv, 0);
    fstats_summary(&g_s, 10 * NS_PER_S, &sum);

    CHECK(sum.frames == 50);
    CHECK(sum.dropped == 0);
    CHECK(sum.bad_frames == 0);
    CHECK(sum.n_intervals == 49);
    CHECK(sum.iv_min_ns == iv);
    CHECK(sum.iv_p50_ns == iv);
    CHECK(sum.iv_max_ns == iv);
    CHECK(sum.fps > 4.99 && sum.fps < 5.01);
}

static void test_drops_and_bad(void)
{
    struct frame_summary sum;

    fstats_new_stream(&g_s);
    fstats_new_window(&g_s, 0);
    fstats_frame(&g_s, 10, 0, 0);
    fstats_frame(&g_s, 11, 200 * NS_PER_MS, 1);
    fstats_frame(&g_s, 14, 800 * NS_PER_MS, 0); /* 12, 13 dropped */
    fstats_frame(&g_s, 3, 1000 * NS_PER_MS, 0); /* backwards: not a drop */
    fstats_stall(&g_s);
    fstats_summary(&g_s, NS_PER_S, &sum);

    CHECK(sum.frames == 4);
    CHECK(sum.dropped == 2);
    CHECK(sum.bad_frames == 1);
    CHECK(sum.stalls == 1);
    CHECK(sum.iv_min_ns == 200 * NS_PER_MS);
    CHECK(sum.iv_max_ns == 600 * NS_PER_MS);
    /* sorted {200, 200, 600}: element n/2 */
    CHECK(sum.iv_p50_ns == 200 * NS_PER_MS);
}

static void test_empty_window(void)
{
    struct frame_summary sum;

    fstats_new_stream(&g_s);
    fstats_new_window(&g_s, 5 * NS_PER_S);
    fstats_summary(&g_s, 15 * NS_PER_S, &sum);
    CHECK(sum.frames == 0);
    CHECK(sum.fps == 0.0);
    CHECK(sum.n_intervals == 0);
    CHECK(sum.iv_min_ns == -1);
    CHECK(sum.iv_p50_ns == -1);
    CHECK(sum.iv_max_ns == -1);
}

static void test_window_keeps_prev_stream_does_not(void)
{
    struct frame_summary sum;

    fstats_new_stream(&g_s);
    fstats_new_window(&g_s, 0);
    fstats_frame(&g_s, 7, 900 * NS_PER_MS, 0);

    /* New window: the boundary-spanning interval still counts. */
    fstats_new_window(&g_s, NS_PER_S);
    fstats_frame(&g_s, 8, 1100 * NS_PER_MS, 0);
    fstats_summary(&g_s, 2 * NS_PER_S, &sum);
    CHECK(sum.frames == 1);
    CHECK(sum.n_intervals == 1);
    CHECK(sum.iv_p50_ns == 200 * NS_PER_MS);

    /* New stream (device reopened): sequence restarts at 0, no drop or
     * interval across the reconnect. */
    fstats_new_stream(&g_s);
    fstats_frame(&g_s, 0, 9 * NS_PER_S, 0);
    fstats_summary(&g_s, 10 * NS_PER_S, &sum);
    CHECK(sum.frames == 2);
    CHECK(sum.dropped == 0);
    CHECK(sum.n_intervals == 1);
}

static void test_interval_overflow(void)
{
    struct frame_summary sum;

    fstats_new_stream(&g_s);
    fstats_new_window(&g_s, 0);
    for (uint32_t i = 0; i < FSTATS_MAX_INTERVALS + 50; i++)
        fstats_frame(&g_s, i, (int64_t)i * 10 * NS_PER_MS, 0);
    /* One long gap past the sample buffer still reaches max. */
    fstats_frame(&g_s, FSTATS_MAX_INTERVALS + 50, 100 * NS_PER_S, 0);
    fstats_summary(&g_s, 101 * NS_PER_S, &sum);

    CHECK(sum.n_intervals == FSTATS_MAX_INTERVALS);
    CHECK(g_s.iv_overflow == 50);
    CHECK(sum.iv_min_ns == 10 * NS_PER_MS);
    CHECK(sum.iv_max_ns > 90 * NS_PER_S);
}

int main(void)
{
    test_steady_5fps();
    test_drops_and_bad();
    test_empty_window();
    test_window_keeps_prev_stream_does_not();
    test_interval_overflow();

    if (failures) {
        fprintf(stderr, "test_frame_stats: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_frame_stats: all checks passed\n");
    return 0;
}
