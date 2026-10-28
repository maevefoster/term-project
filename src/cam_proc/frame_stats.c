#include "frame_stats.h"

#include <string.h>

void fstats_new_window(struct frame_stats *s, int64_t now_ns)
{
    s->window_start_ns = now_ns;
    s->frames = 0;
    s->dropped = 0;
    s->stalls = 0;
    s->bad_frames = 0;
    s->iv_min_ns = -1;
    s->iv_max_ns = -1;
    s->n_iv = 0;
    s->iv_overflow = 0;
}

void fstats_new_stream(struct frame_stats *s)
{
    s->have_prev = 0;
}

void fstats_frame(struct frame_stats *s, uint32_t seq, int64_t t_ns, int bad)
{
    s->frames++;
    if (bad)
        s->bad_frames++;

    if (s->have_prev) {
        uint32_t gap = seq - s->prev_seq;
        int64_t iv = t_ns - s->prev_ns;

        /* Ignore backwards or wrapped-by-reset sequence numbers. */
        if (gap > 1 && gap < 0x80000000u)
            s->dropped += gap - 1;
        if (iv >= 0) {
            if (s->iv_min_ns < 0 || iv < s->iv_min_ns)
                s->iv_min_ns = iv;
            if (iv > s->iv_max_ns)
                s->iv_max_ns = iv;
            if (s->n_iv < FSTATS_MAX_INTERVALS)
                s->iv_ns[s->n_iv++] = iv;
            else
                s->iv_overflow++;
        }
    }
    s->have_prev = 1;
    s->prev_seq = seq;
    s->prev_ns = t_ns;
}

void fstats_stall(struct frame_stats *s)
{
    s->stalls++;
}

void fstats_summary(const struct frame_stats *s, int64_t now_ns, struct frame_summary *out)
{
    int64_t sorted[FSTATS_MAX_INTERVALS];
    size_t n = s->n_iv;

    memset(out, 0, sizeof *out);
    out->window_ns = now_ns - s->window_start_ns;
    out->frames = s->frames;
    out->dropped = s->dropped;
    out->stalls = s->stalls;
    out->bad_frames = s->bad_frames;
    out->fps = out->window_ns > 0 ? (double)s->frames * 1e9 / (double)out->window_ns : 0.0;
    out->n_intervals = n;
    out->iv_min_ns = s->iv_min_ns;
    out->iv_max_ns = s->iv_max_ns;
    out->iv_p50_ns = -1;
    if (n == 0)
        return;

    memcpy(sorted, s->iv_ns, n * sizeof sorted[0]);
    for (size_t i = 1; i < n; i++) {
        int64_t v = sorted[i];
        size_t j = i;

        while (j > 0 && sorted[j - 1] > v) {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = v;
    }
    out->iv_p50_ns = sorted[n / 2];
}
