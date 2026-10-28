/* Per-window capture statistics for cam_proc summaries. Pure bookkeeping:
 * no V4L2 or clock calls, so it can be unit-tested on any host. */
#ifndef POSTURE_FRAME_STATS_H
#define POSTURE_FRAME_STATS_H

#include <stddef.h>
#include <stdint.h>

/* Intervals kept per window for the median. 10 s at 5 fps is 50. */
#define FSTATS_MAX_INTERVALS 256

struct frame_stats {
    int64_t window_start_ns;
    uint64_t frames, dropped, stalls, bad_frames;
    int64_t iv_min_ns, iv_max_ns; /* -1 until the first interval */
    size_t n_iv;
    uint64_t iv_overflow;         /* intervals past FSTATS_MAX_INTERVALS */
    int64_t iv_ns[FSTATS_MAX_INTERVALS];

    int have_prev;
    uint32_t prev_seq;
    int64_t prev_ns;
};

struct frame_summary {
    int64_t window_ns;
    uint64_t frames, dropped, stalls, bad_frames;
    double fps;
    size_t n_intervals;
    /* Inter-frame interval from CLOCK_MONOTONIC dequeue times; -1 when the
     * window had no interval. p50 is element n/2 of the sorted samples. */
    int64_t iv_min_ns, iv_p50_ns, iv_max_ns;
};

/* Starts a new summary window. The previous frame is kept, so the interval
 * spanning the window boundary counts in the new window. */
void fstats_new_window(struct frame_stats *s, int64_t now_ns);

/* Forgets the previous frame: call after (re)opening the device, whose
 * sequence numbers restart. Window counters are untouched. */
void fstats_new_stream(struct frame_stats *s);

void fstats_frame(struct frame_stats *s, uint32_t seq, int64_t t_ns, int bad);
void fstats_stall(struct frame_stats *s);
void fstats_summary(const struct frame_stats *s, int64_t now_ns, struct frame_summary *out);

#endif
