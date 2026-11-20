/*
 * posture_detect.c - two-purple-sticker marker detection using
 * connected clusters, plus craniovertebral angle (CVA).
 *
 * Both stickers are the SAME color (purple), so the detector cannot tell them
 * apart by hue. It tells them apart by position instead:
 *
 *   1. Mark every pixel whose hue/sat/val falls in the purple range.
 *   2. Group touching purple pixels (8-connected) into clusters.
 *   3. Ignore clusters that are too small (strays / noise).
 *   4. Keep clusters that look like a sticker: enough pixels, compact spread,
 *      roughly filled and roughly square bounding box.
 *   5. The frame is accepted ONLY if exactly two such clusters exist.
 *   6. The UPPER cluster (smaller y) is the ear tag, the LOWER one is the
 *      neck (C7) tag. CVA is computed from their centroids as before.
 *
 * Anything else gives POSTURE_NO_USER: zero or one good cluster, three or more
 * (e.g. purple clothing that is also sticker-shaped), tags too close, or tags
 * at the same height (can't say which is the ear).
 *
 * Input : one tightly packed YUYV (YUY2) frame, w*h*2 bytes, w even.
 * Output: posture_obs_t (types in posture_detect.h).
 *
 * g_mask bits: bit 1 (PX_PURPLE) = pixel is in the purple range, bit 2
 * (PX_GOOD) = pixel belongs to a cluster that passed the sticker checks.
 * obs.ear / obs.c7 are only filled (found=1) when the frame is accepted. The
 * debug fields n_clusters, n_good and reject say why a frame was rejected.
 * The old 2.5-sigma outlier pass is gone: strays now form their own tiny
 * clusters and are dropped by min_pixels.
 *
 * Self-test (project flags; also built by `make test` as test_posture_selftest):
 *   cc -std=c17 -Wall -Wextra -Werror -Wvla -D_DEFAULT_SOURCE -Isrc/cam_proc \
 *      -DPOSTURE_TEST -o posture_test src/cam_proc/posture_detect.c -lm
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "posture_detect.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void posture_cfg_default(posture_cfg_t *c)
{
    /* Purple, widened a little toward blue-violet because webcams often render
     * purple a bit bluish. Tune with cam_debug (click a sticker, read H/S/V).
     * Values tuned live with our sticky-note tags (DESIGN.md, items 6 and 7):
     * the brightness minimum 0.40 keeps dark, noisy pixels out, and
     * min_pixels 30 keeps tiny fragments from being taken as a tag (our tags
     * are about 100-150 pixels). */
    c->marker = (hsv_range_t){ 250.f, 310.f, 0.38f, 0.40f };
    c->min_pixels   = 30;
    c->good_pixels  = 60;
    c->max_sigma_px = 12.f;
    c->min_fill     = 0.45f;
    c->max_aspect   = 3.0f;
    c->min_sep_px   = 15.f;
    c->min_dy_px    = 8.f;
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

/* ---------- connected clusters ---------- */

typedef struct {
    int    area;
    int    x0, x1, y0, y1;     /* bounding box, inclusive */
    double cx, cy, sigma;
} cluster_t;

static uint8_t g_mask[MAX_W * MAX_H];
static uint8_t g_seen[MAX_W * MAX_H];
static int     g_queue[MAX_W * MAX_H];

/* Flood-fills (8-connected) the purple cluster containing pixel 'start'.
 * On return g_queue[0..area) holds the indices of its pixels. Iterative, so
 * there is no recursion depth to worry about. */
static void flood(int w, int h, int start, cluster_t *c)
{
    int head = 0, tail = 0;
    double sx = 0, sy = 0, sxx = 0, syy = 0;

    g_seen[start] = 1;
    g_queue[tail++] = start;
    c->x0 = c->x1 = start % w;
    c->y0 = c->y1 = start / w;

    while (head < tail) {
        int idx = g_queue[head++];
        int x = idx % w, y = idx / w;

        sx += x;  sy += y;
        sxx += (double)x * x;  syy += (double)y * y;
        if (x < c->x0) c->x0 = x;
        if (x > c->x1) c->x1 = x;
        if (y < c->y0) c->y0 = y;
        if (y > c->y1) c->y1 = y;

        for (int dy = -1; dy <= 1; dy++) {
            int ny = y + dy;
            if (ny < 0 || ny >= h) continue;
            for (int dx = -1; dx <= 1; dx++) {
                int nx = x + dx;
                if (nx < 0 || nx >= w) continue;
                int ni = ny * w + nx;
                if ((g_mask[ni] & PX_PURPLE) && !g_seen[ni]) {
                    g_seen[ni] = 1;
                    g_queue[tail++] = ni;
                }
            }
        }
    }

    c->area = tail;
    c->cx = sx / tail;
    c->cy = sy / tail;
    double var = sxx / tail - c->cx * c->cx + syy / tail - c->cy * c->cy;
    c->sigma = var > 0.0 ? sqrt(var) : 0.0;
}

/* A cluster is "good" (sticker-like) if it is big enough, compact, mostly
 * filled and not a long thin stripe. Strays fail min_pixels and are ignored. */
static int cluster_is_good(const cluster_t *c, const posture_cfg_t *cfg)
{
    int bw = c->x1 - c->x0 + 1, bh = c->y1 - c->y0 + 1;
    int lo = bw < bh ? bw : bh, hi = bw < bh ? bh : bw;

    if (c->area < cfg->min_pixels) return 0;
    if ((float)c->sigma > cfg->max_sigma_px) return 0;
    if ((float)c->area / (float)(bw * bh) < cfg->min_fill) return 0;
    if ((float)hi / (float)lo > cfg->max_aspect) return 0;
    return 1;
}

static void make_blob(const cluster_t *c, const posture_cfg_t *cfg, blob_t *b)
{
    float count_conf = (float)c->area / (float)cfg->good_pixels;
    float comp_conf = 1.f - (float)c->sigma / cfg->max_sigma_px;
    if (count_conf > 1.f) count_conf = 1.f;
    if (comp_conf < 0.f) comp_conf = 0.f;

    b->found = 1;
    b->x = (float)c->cx;
    b->y = (float)c->cy;
    b->npix = c->area;
    b->sigma = (float)c->sigma;
    b->conf = count_conf * comp_conf;
}

/* ---------- CVA ---------- */

/*
 * CVA = angle between the horizontal line through the neck (C7) tag and the
 * line from the neck tag to the ear tag. Side view, image y grows downward:
 *
 *     dx  = |ear.x - c7.x|          (direction-agnostic)
 *     dy  =  c7.y - ear.y           (> 0: ear tag is above the neck tag)
 *     CVA = atan2(dy, dx) * 180/pi
 *
 * Upright ~ 50-55 deg; smaller = head further forward. Because the ear tag is
 * defined as the upper cluster, dy > 0 here by construction.
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

/* Returns 0 on success (even if NO_USER), -1 on bad arguments. */
int posture_detect(const uint8_t *yuyv, int w, int h,
                   const posture_cfg_t *cfg, posture_obs_t *out)
{
    cluster_t good[MAX_KEEP];

    if (!yuyv || !cfg || !out || w <= 0 || h <= 0 || (w & 1) ||
        w > MAX_W || h > MAX_H)
        return -1;

    memset(out, 0, sizeof *out);
    out->status = POSTURE_NO_USER;
    out->reject = REJ_NOT_TWO;
    memset(g_seen, 0, (size_t)w * (size_t)h);

    /* pass 1: purple mask */
    for (int y = 0; y < h; y++) {
        const uint8_t *row = yuyv + (size_t)y * w * 2;
        for (int x = 0; x < w; x += 2) {          /* 2 pixels per 4 bytes */
            int y0 = row[2 * x], u = row[2 * x + 1];
            int y1 = row[2 * x + 2], v = row[2 * x + 3];
            for (int k = 0; k < 2; k++) {
                float hh, s, val;
                yuv_to_hsv(k ? y1 : y0, u, v, &hh, &s, &val);
                g_mask[y * w + x + k] = in_range(&cfg->marker, hh, s, val) ? PX_PURPLE : 0;
            }
        }
    }

    /* pass 2: connected clusters */
    for (int i = 0; i < w * h; i++) {
        cluster_t c;
        if (!(g_mask[i] & PX_PURPLE) || g_seen[i]) continue;
        flood(w, h, i, &c);
        out->n_clusters++;
        if (!cluster_is_good(&c, cfg)) continue;
        for (int k = 0; k < c.area; k++) g_mask[g_queue[k]] |= PX_GOOD;
        if (out->n_good < MAX_KEEP) good[out->n_good] = c;
        out->n_good++;
    }

    if (out->n_good != 2) return 0;                       /* NO_USER */

    /* the upper cluster is the ear tag, the lower one the neck tag */
    const cluster_t *up = &good[0], *low = &good[1];
    if (up->cy > low->cy) { up = &good[1]; low = &good[0]; }

    if ((float)(low->cy - up->cy) < cfg->min_dy_px) { out->reject = REJ_SAME_HEIGHT; return 0; }
    if (hypotf((float)(low->cx - up->cx), (float)(low->cy - up->cy)) < cfg->min_sep_px) {
        out->reject = REJ_TOO_CLOSE;
        return 0;
    }

    blob_t ear, c7;
    float cva;
    make_blob(up, cfg, &ear);
    make_blob(low, cfg, &c7);
    if (!compute_cva(&ear, &c7, &cva)) { out->reject = REJ_SAME_HEIGHT; return 0; }

    out->ear = ear;
    out->c7 = c7;
    out->cva_deg = cva;
    out->confidence = ear.conf < c7.conf ? ear.conf : c7.conf;
    out->status = POSTURE_OK;
    out->reject = REJ_NONE;
    return 0;
}

#ifdef POSTURE_TEST
/* ------------------------------------------------------------------ */
/* Self-test: synthetic frames. These define the intended behavior.   */
/* Rectangles use even x origins and even widths so YUYV chroma        */
/* sharing does not blur their edges.                                  */
/* ------------------------------------------------------------------ */
#define W 320
#define H 240

static uint8_t F[W * H * 2];
static int g_fail, g_checks;

#define CHECK(cond, msg) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("FAIL: %s  (line %d)\n", msg, __LINE__); } } while (0)

static void rgb_to_yuv(int r, int g, int b, uint8_t yuv[3])
{
    double y = 16 + (65.738 * r + 129.057 * g + 25.064 * b) / 256.0;
    double u = 128 + (-37.945 * r - 74.494 * g + 112.439 * b) / 256.0;
    double v = 128 + (112.439 * r - 94.154 * g - 18.285 * b) / 256.0;
    yuv[0] = clamp8((int)(y + 0.5)); yuv[1] = clamp8((int)(u + 0.5)); yuv[2] = clamp8((int)(v + 0.5));
}

static uint8_t BG[3], PURPLE[3], GREEN[3], BLUE[3];

static void put(int x, int y, const uint8_t *c)
{
    uint8_t *p = &F[((size_t)y * W + (size_t)(x & ~1)) * 2];
    p[(x & 1) ? 2 : 0] = c[0];
    p[1] = c[1];
    p[3] = c[2];
}

static void rect(int x0, int y0, int w, int h, const uint8_t *c)
{
    for (int y = y0; y < y0 + h; y++)
        for (int x = x0; x < x0 + w; x++)
            put(x, y, c);
}

static void clear_frame(void) { rect(0, 0, W, H, BG); }

static posture_obs_t run(void)
{
    posture_cfg_t cfg;
    posture_obs_t o;
    posture_cfg_default(&cfg);
    if (posture_detect(F, W, H, &cfg, &o) != 0) { g_fail++; printf("FAIL: posture_detect returned -1\n"); }
    return o;
}

/* Standard scene: neck tag centroid (154.5,164.5), ear tag (194.5,104.5).
 * dx=40, dy=60 -> 56.31 deg. */
static void base_scene(void)
{
    clear_frame();
    rect(190, 100, 10, 10, PURPLE);   /* ear  (upper) */
    rect(150, 160, 10, 10, PURPLE);   /* neck (lower) */
}

static int near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

int main(void)
{
    posture_obs_t o;
    const float EXPECT = atan2f(60.f, 40.f) * 180.f / (float)M_PI;

    rgb_to_yuv(110, 110, 110, BG);
    rgb_to_yuv(160, 50, 220, PURPLE);
    rgb_to_yuv(30, 220, 60, GREEN);
    rgb_to_yuv(40, 80, 230, BLUE);

    /* 1. two purple squares: ear on top, neck below, correct angle */
    base_scene();
    o = run();
    CHECK(o.status == POSTURE_OK, "two squares -> OK");
    CHECK(o.ear.y < o.c7.y, "ear is the upper tag");
    CHECK(near(o.ear.x, 194.5f, 0.6f) && near(o.ear.y, 104.5f, 0.6f), "ear centroid");
    CHECK(near(o.c7.x, 154.5f, 0.6f) && near(o.c7.y, 164.5f, 0.6f), "neck centroid");
    CHECK(near(o.cva_deg, EXPECT, 0.5f), "CVA ~56.3 deg");
    CHECK(o.confidence > 0.5f, "confidence reasonable");
    CHECK(o.n_good == 2 && o.reject == REJ_NONE, "n_good == 2, no reject");

    /* 2. mirrored (user faces the other way) gives the same angle */
    clear_frame();
    rect(110, 100, 10, 10, PURPLE);
    rect(150, 160, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_OK && near(o.cva_deg, EXPECT, 0.5f), "mirrored scene -> same CVA");

    /* 3. the labels follow position, not draw order */
    clear_frame();
    rect(150, 160, 10, 10, PURPLE);   /* neck drawn first */
    rect(190, 100, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_OK && o.ear.y < o.c7.y, "ear/neck independent of order");

    /* 4. a single purple square is NO_USER */
    clear_frame();
    rect(190, 100, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_NO_USER && o.reject == REJ_NOT_TWO && !o.ear.found && !o.c7.found,
          "single square -> NO_USER");

    /* 5. no purple at all */
    clear_frame();
    o = run();
    CHECK(o.status == POSTURE_NO_USER && o.n_clusters == 0, "empty frame -> NO_USER");

    /* 6. isolated stray pixels do not change the result */
    base_scene();
    for (int x = 10; x < W; x += 20) { rect(x, 20, 2, 2, PURPLE); rect(x, 215, 2, 2, PURPLE); }
    o = run();
    CHECK(o.status == POSTURE_OK && near(o.cva_deg, EXPECT, 0.2f), "strays ignored, CVA unchanged");
    CHECK(o.n_clusters > 2 && o.n_good == 2, "strays counted as clusters but not good");

    /* 7. a small bump touching a tag barely moves the centroid */
    base_scene();
    rect(200, 100, 2, 2, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_OK && near(o.cva_deg, EXPECT, 0.7f), "touching bump: CVA within 0.7 deg");

    /* 8. green and blue squares are ignored (purple only) */
    base_scene();
    rect(60, 60, 10, 10, GREEN);
    rect(240, 60, 10, 10, BLUE);
    o = run();
    CHECK(o.status == POSTURE_OK && o.n_good == 2, "non-purple squares ignored");

    /* 9. a third sticker-like purple object -> not exactly two -> NO_USER */
    base_scene();
    rect(60, 40, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_NO_USER && o.reject == REJ_NOT_TWO && o.n_good == 3, "three squares -> NO_USER");

    /* 10. tags touching merge into one cluster -> NO_USER */
    clear_frame();
    rect(150, 150, 10, 10, PURPLE);
    rect(150, 160, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_NO_USER && o.n_good == 1, "touching tags merge -> NO_USER");

    /* 11. tags too close (clear gap, but centroids < min_sep_px apart) */
    clear_frame();
    rect(150, 140, 6, 6, PURPLE);
    rect(150, 152, 6, 6, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_NO_USER && o.reject == REJ_TOO_CLOSE, "tags too close -> NO_USER");

    /* 12. same height: cannot tell which is the ear */
    clear_frame();
    rect(110, 120, 10, 10, PURPLE);
    rect(190, 120, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_NO_USER && o.reject == REJ_SAME_HEIGHT, "same height -> NO_USER");

    /* 13. one tag below min_pixels is a stray: only one good tag -> NO_USER */
    clear_frame();
    rect(190, 100, 4, 2, PURPLE);      /* 8 px */
    rect(150, 160, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_NO_USER && o.n_good == 1, "undersized tag -> NO_USER");

    /* 14. a long thin purple stripe (clothing seam) is not a sticker */
    base_scene();
    rect(40, 30, 40, 4, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_OK && o.n_good == 2, "thin stripe ignored");

    /* 15. a big purple area (shirt) is too spread out to be a sticker */
    base_scene();
    rect(240, 20, 40, 40, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_OK && o.n_good == 2, "large purple blob ignored");

    /* 16. head forward -> smaller CVA */
    clear_frame();
    rect(210, 100, 10, 10, PURPLE);    /* ear further forward: dx=60, dy=60 -> 45 deg */
    rect(150, 160, 10, 10, PURPLE);
    o = run();
    CHECK(o.status == POSTURE_OK && near(o.cva_deg, 45.f, 0.5f), "head forward -> ~45 deg");

    /* 17. bad arguments */
    {
        posture_cfg_t cfg; posture_cfg_default(&cfg);
        CHECK(posture_detect(NULL, W, H, &cfg, &o) == -1, "NULL frame rejected");
        CHECK(posture_detect(F, 321, H, &cfg, &o) == -1, "odd width rejected");
    }

    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
#endif
