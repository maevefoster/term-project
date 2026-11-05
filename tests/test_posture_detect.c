/* Host unit tests for src/cam_proc/posture_detect.c and the markers.c seam.
 * Synthetic frames only (built in memory, never read from or written to
 * disk); colors and geometry match the original POSTURE_TEST self-test. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "markers.h"
#include "posture_detect.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

enum { W = 320, H = 240, PAD = 64 };

/* BT.601 limited-range YUV of pure green / pure blue */
static const uint8_t GY = 145, GU = 54, GV = 34;   /* green (ear) */
static const uint8_t BY = 41, BU = 240, BV = 110;  /* blue (C7) */

static uint8_t g_frame[(W * 2 + PAD) * H]; /* large; keep off the stack */

static void fill_grey(uint8_t *f, size_t stride)
{
    for (size_t y = 0; y < H; y++)
        for (size_t x = 0; x < stride; x += 2) {
            f[y * stride + x] = 110;
            f[y * stride + x + 1] = 128;
        }
}

/* 9x9-ish square centered on (cx, cy), written as YUYV pairs at even x. */
static void square(uint8_t *f, size_t stride, int cx, int cy,
                   uint8_t yy, uint8_t u, uint8_t v)
{
    for (int dy = -4; dy <= 4; dy++)
        for (int dx = -4; dx <= 4; dx += 2) {
            uint8_t *p = &f[(size_t)(cy + dy) * stride + (size_t)(cx + dx) * 2];

            p[0] = yy; p[1] = u; p[2] = yy; p[3] = v;
        }
}

static const float EXPECT_CVA = 56.31f; /* atan2(60, 40) in degrees */

static void test_upright_pair(void)
{
    posture_cfg_t cfg;
    posture_obs_t o;

    posture_cfg_default(&cfg);
    fill_grey(g_frame, W * 2);
    square(g_frame, W * 2, 190, 100, GY, GU, GV); /* ear */
    square(g_frame, W * 2, 150, 160, BY, BU, BV); /* C7 */

    CHECK(posture_detect(g_frame, W, H, &cfg, &o) == 0);
    CHECK(o.status == POSTURE_OK);
    CHECK(o.ear.found && o.c7.found);
    CHECK(fabsf(o.ear.x - 190.5f) < 1.0f && fabsf(o.ear.y - 100.0f) < 1.0f);
    CHECK(fabsf(o.c7.x - 150.5f) < 1.0f && fabsf(o.c7.y - 160.0f) < 1.0f);
    CHECK(fabsf(o.cva_deg - EXPECT_CVA) < 0.5f);
    CHECK(o.confidence > 0.0f && o.confidence <= 1.0f);
}

static void test_mirrored_pair(void)
{
    posture_cfg_t cfg;
    posture_obs_t o;

    /* Ear to the left of C7: CVA is documented as direction-agnostic. */
    posture_cfg_default(&cfg);
    fill_grey(g_frame, W * 2);
    square(g_frame, W * 2, 110, 100, GY, GU, GV);
    square(g_frame, W * 2, 150, 160, BY, BU, BV);

    CHECK(posture_detect(g_frame, W, H, &cfg, &o) == 0);
    CHECK(o.status == POSTURE_OK);
    CHECK(fabsf(o.cva_deg - EXPECT_CVA) < 0.5f);
}

static void test_empty_frame(void)
{
    posture_cfg_t cfg;
    posture_obs_t o;

    posture_cfg_default(&cfg);
    fill_grey(g_frame, W * 2);

    CHECK(posture_detect(g_frame, W, H, &cfg, &o) == 0);
    CHECK(o.status == POSTURE_NO_USER);
    CHECK(!o.ear.found && !o.c7.found);
}

static void test_one_marker(void)
{
    posture_cfg_t cfg;
    posture_obs_t o;

    posture_cfg_default(&cfg);
    fill_grey(g_frame, W * 2);
    square(g_frame, W * 2, 190, 100, GY, GU, GV); /* ear only */

    CHECK(posture_detect(g_frame, W, H, &cfg, &o) == 0);
    CHECK(o.status == POSTURE_NO_USER);
    CHECK(o.ear.found && !o.c7.found);
}

static void test_ear_below_c7(void)
{
    posture_cfg_t cfg;
    posture_obs_t o;

    posture_cfg_default(&cfg);
    fill_grey(g_frame, W * 2);
    square(g_frame, W * 2, 190, 160, GY, GU, GV); /* ear lower than C7 */
    square(g_frame, W * 2, 150, 100, BY, BU, BV);

    CHECK(posture_detect(g_frame, W, H, &cfg, &o) == 0);
    CHECK(o.status == POSTURE_NO_USER);
}

static void test_bad_args(void)
{
    posture_cfg_t cfg;
    posture_obs_t o;

    posture_cfg_default(&cfg);
    CHECK(posture_detect(NULL, W, H, &cfg, &o) == -1);
    CHECK(posture_detect(g_frame, W, H, NULL, &o) == -1);
    CHECK(posture_detect(g_frame, W, H, &cfg, NULL) == -1);
    CHECK(posture_detect(g_frame, 0, H, &cfg, &o) == -1);
    CHECK(posture_detect(g_frame, W + 1, H, &cfg, &o) == -1); /* odd width */
    CHECK(posture_detect(g_frame, MAX_W + 2, H, &cfg, &o) == -1);
    CHECK(posture_detect(g_frame, W, MAX_H + 1, &cfg, &o) == -1);
}

static void test_markers_seam(void)
{
    struct marker_stats m;
    const size_t packed = (size_t)W * 2 * H;

    markers_take_stats(&m); /* reset */

    fill_grey(g_frame, W * 2);
    square(g_frame, W * 2, 190, 100, GY, GU, GV);
    square(g_frame, W * 2, 150, 160, BY, BU, BV);
    markers_on_frame(g_frame, packed, W, H, W * 2);      /* found */
    markers_on_frame(g_frame, packed - 1, W, H, W * 2);  /* short buffer */
    markers_on_frame(g_frame, sizeof g_frame, W, H, W * 2 + PAD); /* padded rows */
    fill_grey(g_frame, W * 2);
    markers_on_frame(g_frame, packed, W, H, W * 2);      /* no markers */

    markers_take_stats(&m);
    CHECK(m.frames == 4);
    CHECK(m.found == 1);
    CHECK(m.skipped == 2);
    CHECK(m.no_user == 1);
    CHECK(m.have_last);
    CHECK(fabsf(m.last_cva_deg - EXPECT_CVA) < 0.5f);

    /* Counters reset; last observation persists. */
    markers_take_stats(&m);
    CHECK(m.frames == 0 && m.found == 0 && m.skipped == 0 && m.no_user == 0);
    CHECK(m.have_last);
}

int main(void)
{
    test_upright_pair();
    test_mirrored_pair();
    test_empty_frame();
    test_one_marker();
    test_ear_below_c7();
    test_bad_args();
    test_markers_seam();

    if (failures) {
        fprintf(stderr, "test_posture_detect: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_posture_detect: all checks passed\n");
    return 0;
}
