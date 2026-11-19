/* posture_detect.h - public types and entry points for the student-written
 * two-purple-tag marker detection + CVA in posture_detect.c (owner: Tiffany).
 * The types below were moved here verbatim from the submitted detector; see
 * posture_detect.c for the algorithm and the meaning of each field. */
#ifndef POSTURE_DETECT_H
#define POSTURE_DETECT_H

#include <stdint.h>

#define MAX_W 640
#define MAX_H 480

#define PX_PURPLE 1u   /* in the purple range */
#define PX_GOOD   2u   /* member of a cluster that passed the sticker checks */
#define MAX_KEEP  8    /* good clusters remembered (more than 2 is a reject anyway) */

typedef struct {
    float h_lo, h_hi;   /* hue range in degrees [0,360); lo > hi wraps (red) */
    float s_min;        /* min saturation 0..1 */
    float v_min;        /* min value 0..1 */
} hsv_range_t;

typedef struct {
    hsv_range_t marker;    /* purple range, used for BOTH stickers              */
    int   min_pixels;      /* cluster smaller than this is a stray: ignored      */
    int   good_pixels;     /* pixel count that gives full count-confidence       */
    float max_sigma_px;    /* RMS spread above this = not a sticker              */
    float min_fill;        /* cluster area / bounding-box area, at least this    */
    float max_aspect;      /* bounding-box long side / short side, at most this  */
    float min_sep_px;      /* the two tags' centroids must be this far apart     */
    float min_dy_px;       /* ...and this far apart vertically (ear above neck)  */
} posture_cfg_t;

typedef enum { POSTURE_OK = 0, POSTURE_NO_USER = 1 } posture_status_t;

typedef enum {
    REJ_NONE = 0,
    REJ_NOT_TWO,       /* not exactly two good clusters                  */
    REJ_TOO_CLOSE,     /* centroids closer than min_sep_px               */
    REJ_SAME_HEIGHT    /* centroids closer than min_dy_px vertically     */
} posture_reject_t;

typedef struct {
    int   found;
    float x, y;        /* centroid, pixels (y grows downward) */
    int   npix;
    float sigma;       /* RMS spread of the cluster, pixels */
    float conf;        /* 0..1 */
} blob_t;

typedef struct {
    posture_status_t status;
    blob_t ear, c7;          /* ear = upper tag, c7 = the neck (lower) tag */
    float cva_deg;           /* valid only if status == POSTURE_OK */
    float confidence;        /* 0..1, overall */
    int   n_clusters;        /* every purple cluster seen, strays included */
    int   n_good;            /* clusters that passed the sticker checks */
    posture_reject_t reject; /* why status is NO_USER (REJ_NONE when OK) */
} posture_obs_t;

void posture_cfg_default(posture_cfg_t *c);

/* `yuyv` must be tightly packed (row stride == w * 2). Returns 0 on success
 * (even if NO_USER), -1 on bad arguments. */
int posture_detect(const uint8_t *yuyv, int w, int h,
                   const posture_cfg_t *cfg, posture_obs_t *out);

#endif
