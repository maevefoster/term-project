/* Seam between V4L2 capture and the student-written marker pipeline
 * (posture_detect.c: HSV marker detection, centroids, confidence, CVA).
 * Owner: Tiffany. This file is plumbing only; the algorithm lives in
 * posture_detect.c and is written by the team, not generated.
 *
 * `yuyv` points straight into a V4L2 mmap buffer that is requeued as soon
 * as markers_on_frame returns. Read it in place; never copy it to disk or
 * a log. Only derived numbers (counts, CVA, confidence) leave this module. */
#ifndef POSTURE_MARKERS_H
#define POSTURE_MARKERS_H

#include <stddef.h>
#include <stdint.h>

/* Per-summary-window counters. `last_*` persist across windows. */
struct marker_stats {
    uint64_t frames;        /* frames handed to markers_on_frame */
    uint64_t found;         /* both markers found, CVA valid */
    uint64_t no_user;       /* detector ran, markers missing/rejected */
    uint64_t skipped;       /* not run: padded rows, short buffer, bad size */
    int have_last;          /* last_cva/last_conf valid */
    float last_cva_deg;
    float last_conf;
};

void markers_on_frame(const uint8_t *yuyv, size_t len,
                      uint32_t width, uint32_t height, uint32_t bytesperline);

/* Copies the current window's counters into *out and resets them. */
void markers_take_stats(struct marker_stats *out);

/* Result for the most recent frame passed to markers_on_frame. */
struct marker_obs {
    int ran;        /* markers_on_frame has been called at least once */
    int found;      /* that frame gave a valid CVA */
    float cva_deg;  /* valid only if found */
    float conf;     /* valid only if found */
};

void markers_last_obs(struct marker_obs *out);

#endif
