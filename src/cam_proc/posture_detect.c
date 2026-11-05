/*
 * posture_detect.c - HSV marker detection + craniovertebral angle (CVA)
 * for cam_proc.
 *
 * Input : one YUYV (YUY2) frame from V4L2 (e.g. 320x240, 153600 bytes).
 * Output: centroids of two colored stickers, CVA in degrees, a confidence,
 *         and a status (POSTURE_OK or POSTURE_NO_USER).
 *
 * Camera must view the user from the SIDE. Stickers:
 *   ear marker (tragus)  - GREEN by default
 *   neck marker (C7)     - BLUE by default
 * Change the hue ranges in posture_cfg_default() to match your stickers.
 *
 * Built into cam_proc via the Makefile; called from markers.c.
 * Unit test: tests/test_posture_detect.c (make test).
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "posture_detect.h"

void posture_cfg_default(posture_cfg_t *c)
{
    /* Green and blue are ~120 deg apart and far from skin hue (~0-25 deg),
     * so neither should match the face or neck. Use neon/fluorescent
     * stickers so saturation stays high in dim or warm light. */
    c->ear = (hsv_range_t){ 90.f, 165.f, 0.50f, 0.25f };   /* neon green */
    c->c7  = (hsv_range_t){ 200.f, 255.f, 0.50f, 0.20f };  /* blue       */
    c->min_pixels   = 12;
    c->good_pixels  = 60;
    c->max_sigma_px = 12.f;
    c->min_sep_px   = 15.f;
    c->outlier_sigma = 2.5f;
}

/* ---------- color conversion ---------- */

static inline uint8_t clamp8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

/* BT.601 YUV -> RGB, then RGB -> HSV (h in degrees, s,v in 0..1) */
static inline void yuv_to_hsv(int y, int u, int v, float *h, float *s, float *val)
{
    int c = y - 16, d = u - 128, e = v - 128;
    int r = clamp8((298 * c + 409 * e + 128) >> 8);
    int g = clamp8((298 * c - 100 * d - 208 * e + 128) >> 8);
    int b = clamp8((298 * c + 516 * d + 128) >> 8);

    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    float delta = (float)(mx - mn);

    *val = mx / 255.f;
    *s = mx ? delta / mx : 0.f;
    if (delta < 1.f) { *h = 0.f; return; }
    float hh;
    if (mx == r)      hh = 60.f * fmodf((g - b) / delta, 6.f);
    else if (mx == g) hh = 60.f * ((b - r) / delta + 2.f);
    else              hh = 60.f * ((r - g) / delta + 4.f);
    if (hh < 0.f) hh += 360.f;
    *h = hh;
}

static inline int in_range(const hsv_range_t *r, float h, float s, float v)
{
    if (s < r->s_min || v < r->v_min) return 0;
    if (r->h_lo <= r->h_hi) return h >= r->h_lo && h <= r->h_hi;
    return h >= r->h_lo || h <= r->h_hi;      /* wraps through 0 (red) */
}

/* ---------- blob centroid ---------- */

/* bit: 1 = ear mask, 2 = c7 mask. Two passes: plain centroid, then
 * recompute after discarding pixels > outlier_sigma away (kills stray
 * pixels from similarly colored clothing or noise). */
static void find_blob(const uint8_t *mask, int w, int h, uint8_t bit,
                      const posture_cfg_t *cfg, blob_t *b)
{
    memset(b, 0, sizeof *b);
    double sx = 0, sy = 0; long n = 0;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (mask[y * w + x] & bit) { sx += x; sy += y; n++; }
    if (n < cfg->min_pixels) return;

    double cx = sx / n, cy = sy / n, var = 0;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (mask[y * w + x] & bit)
                var += (x - cx) * (x - cx) + (y - cy) * (y - cy);
    double sigma = sqrt(var / n);

    /* refine */
    double lim = cfg->outlier_sigma * (sigma > 1.0 ? sigma : 1.0);
    sx = sy = 0; long m = 0; var = 0;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (mask[y * w + x] & bit) {
                double dx = x - cx, dy = y - cy;
                if (dx * dx + dy * dy <= lim * lim) { sx += x; sy += y; m++; }
            }
    if (m < cfg->min_pixels) return;
    cx = sx / m; cy = sy / m;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (mask[y * w + x] & bit) {
                double dx = x - cx, dy = y - cy;
                if (dx * dx + dy * dy <= lim * lim) var += dx * dx + dy * dy;
            }
    sigma = sqrt(var / m);

    float count_conf = (float)m / cfg->good_pixels; if (count_conf > 1.f) count_conf = 1.f;
    float comp_conf = 1.f - (float)sigma / cfg->max_sigma_px;
    if (comp_conf < 0.f) comp_conf = 0.f;

    b->found = comp_conf > 0.f;
    b->x = (float)cx; b->y = (float)cy;
    b->npix = (int)m; b->sigma = (float)sigma;
    b->conf = count_conf * comp_conf;
}

/* ---------- CVA ---------- */

/*
 * CVA = angle between the horizontal line through C7 and the line from C7
 * to the ear marker (tragus). Side view, image y grows downward:
 *
 *     dx  = |ear.x - c7.x|          (horizontal distance, direction-agnostic)
 *     dy  =  c7.y - ear.y           (ear height above C7; > 0 if ear is higher)
 *     CVA = atan2(dy, dx) * 180/pi
 *
 * Upright ~ 50-55 deg; smaller = head further forward. Ear at or below C7
 * (dy <= 0) is physically implausible, so it is rejected.
 */
static int compute_cva(const blob_t *ear, const blob_t *c7, float *cva_deg)
{
    float dx = fabsf(ear->x - c7->x);
    float dy = c7->y - ear->y;
    if (dy <= 0.f) return 0;
    *cva_deg = atan2f(dy, dx) * (180.f / (float)M_PI);
    return 1;
}

/* ---------- public entry point ---------- */

static uint8_t g_mask[MAX_W * MAX_H];

/* Returns 0 on success (even if NO_USER), -1 on bad arguments. */
int posture_detect(const uint8_t *yuyv, int w, int h,
                   const posture_cfg_t *cfg, posture_obs_t *out)
{
    if (!yuyv || !cfg || !out || w <= 0 || h <= 0 || (w & 1) ||
        w > MAX_W || h > MAX_H)
        return -1;

    memset(out, 0, sizeof *out);
    out->status = POSTURE_NO_USER;

    for (int y = 0; y < h; y++) {
        const uint8_t *row = yuyv + (size_t)y * w * 2;
        for (int x = 0; x < w; x += 2) {          /* 2 pixels per 4 bytes */
            int y0 = row[2 * x], u = row[2 * x + 1];
            int y1 = row[2 * x + 2], v = row[2 * x + 3];
            for (int k = 0; k < 2; k++) {
                float hh, s, val;
                yuv_to_hsv(k ? y1 : y0, u, v, &hh, &s, &val);
                uint8_t m = 0;
                if (in_range(&cfg->ear, hh, s, val)) m |= 1;
                if (in_range(&cfg->c7,  hh, s, val)) m |= 2;
                g_mask[y * w + x + k] = m;
            }
        }
    }

    find_blob(g_mask, w, h, 1, cfg, &out->ear);
    find_blob(g_mask, w, h, 2, cfg, &out->c7);

    if (!out->ear.found || !out->c7.found) return 0;       /* NO_USER */

    float sep = hypotf(out->ear.x - out->c7.x, out->ear.y - out->c7.y);
    if (sep < cfg->min_sep_px) return 0;

    float cva;
    if (!compute_cva(&out->ear, &out->c7, &cva)) return 0;

    out->cva_deg = cva;
    out->confidence = out->ear.conf < out->c7.conf ? out->ear.conf : out->c7.conf;
    out->status = POSTURE_OK;
    return 0;
}
