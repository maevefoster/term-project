/* Seam for the student-written marker pipeline: HSV marker detection,
 * centroids, confidence and CVA. Owner: Tiffany. Not implemented in M3;
 * per CLAUDE.md section 1 this body is written by the team, not generated.
 *
 * `yuyv` points straight into a V4L2 mmap buffer that is requeued as soon
 * as this returns. Read it in place; never copy it to disk or a log. */
#ifndef POSTURE_MARKERS_H
#define POSTURE_MARKERS_H

#include <stddef.h>
#include <stdint.h>

void markers_on_frame(const uint8_t *yuyv, size_t len,
                      uint32_t width, uint32_t height, uint32_t bytesperline);

#endif
