/* posture_detect.h - public types and entry points for the student-written
 * HSV marker detection + CVA in posture_detect.c (owner: Tiffany).
 * Definitions moved here verbatim from posture_detect.c; see that file for
 * the algorithm and the meaning of each field. */
#ifndef POSTURE_DETECT_H
#define POSTURE_DETECT_H

#include <stdint.h>

#define MAX_W 640
#define MAX_H 480

typedef struct {
    float h_lo, h_hi;   /* hue range in degrees [0,360); lo > hi wraps (red) */
    float s_min;        /* min saturation 0..1 */
    float v_min;        /* min value 0..1 */
} hsv_range_t;

typedef struct {
    hsv_range_t ear, c7;
    int   min_pixels;      /* blob needs at least this many pixels to count   */
    int   good_pixels;     /* pixel count that gives full count-confidence    */
    float max_sigma_px;    /* blob spread (std dev) above this = not a sticker */
    float min_sep_px;      /* markers closer than this are rejected           */
    float outlier_sigma;   /* drop pixels farther than this many sigma        */
} posture_cfg_t;

typedef enum { POSTURE_OK = 0, POSTURE_NO_USER = 1 } posture_status_t;

typedef struct {
    int   found;
    float x, y;        /* centroid, pixels (y grows downward) */
    int   npix;
    float sigma;       /* RMS spread of the blob, pixels */
    float conf;        /* 0..1 */
} blob_t;

typedef struct {
    posture_status_t status;
    blob_t ear, c7;
    float cva_deg;     /* valid only if status == POSTURE_OK */
    float confidence;  /* 0..1, overall */
} posture_obs_t;

void posture_cfg_default(posture_cfg_t *c);

/* `yuyv` must be tightly packed (row stride == w * 2). Returns 0 on success
 * (even if NO_USER), -1 on bad arguments. */
int posture_detect(const uint8_t *yuyv, int w, int h,
                   const posture_cfg_t *cfg, posture_obs_t *out);

#endif
