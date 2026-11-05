#include "markers.h"

#include "posture_detect.h"

static posture_cfg_t g_cfg;
static int g_cfg_ready;
static struct marker_stats g_stats;

void markers_on_frame(const uint8_t *yuyv, size_t len,
                      uint32_t width, uint32_t height, uint32_t bytesperline)
{
    posture_obs_t obs;

    if (!g_cfg_ready) {
        posture_cfg_default(&g_cfg);
        g_cfg_ready = 1;
    }
    g_stats.frames++;

    /* posture_detect expects tightly packed rows (stride == width * 2). */
    if (!yuyv || width == 0 || height == 0 ||
        (uint64_t)bytesperline != (uint64_t)width * 2u ||
        (uint64_t)len < (uint64_t)bytesperline * height) {
        g_stats.skipped++;
        return;
    }
    if (posture_detect(yuyv, (int)width, (int)height, &g_cfg, &obs) < 0) {
        g_stats.skipped++; /* size outside MAX_W x MAX_H or odd width */
        return;
    }
    if (obs.status != POSTURE_OK) {
        g_stats.no_user++;
        return;
    }
    g_stats.found++;
    g_stats.have_last = 1;
    g_stats.last_cva_deg = obs.cva_deg;
    g_stats.last_conf = obs.confidence;
}

void markers_take_stats(struct marker_stats *out)
{
    *out = g_stats;
    g_stats.frames = 0;
    g_stats.found = 0;
    g_stats.no_user = 0;
    g_stats.skipped = 0;
}
