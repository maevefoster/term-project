#include "hub_state.h"

void hub_source_connect(struct hub_source *s)
{
    s->connected = 1;
    s->have_msg = 0;
    s->usable = 0;
    s->have_seq = 0;
}

void hub_source_disconnect(struct hub_source *s)
{
    s->connected = 0;
    s->usable = 0;
}

void hub_source_note_seq(struct hub_source *s, uint32_t seq)
{
    if (s->have_seq) {
        uint32_t gap = seq - s->last_seq;

        if (gap > 1 && gap < 0x80000000u)
            s->seq_gaps += gap - 1;
    }
    s->have_seq = 1;
    s->last_seq = seq;
}

void hub_source_data(struct hub_source *s, int64_t now_ns, enum sensor_status status,
                     int usable)
{
    s->have_msg = 1;
    s->last_rx_ns = now_ns;
    s->status = status;
    s->usable = usable;
}

int hub_source_fresh(const struct hub_source *s, int64_t now_ns)
{
    return s->connected && s->have_msg && now_ns - s->last_rx_ns <= HUB_STALE_NS &&
           s->usable;
}

const char *hub_source_why(const struct hub_source *s, int64_t now_ns)
{
    if (!s->connected)
        return "disconnected";
    if (!s->have_msg)
        return "no_data";
    if (now_ns - s->last_rx_ns > HUB_STALE_NS)
        return "stale";
    if (s->status != SENSOR_OK)
        return sensor_status_name(s->status);
    if (!s->usable)
        return "no_cva";
    return "ok";
}

enum hub_mode hub_mode_eval(int imu_fresh, int cam_fresh)
{
    if (imu_fresh && cam_fresh)
        return HUB_MODE_OK;
    if (cam_fresh)
        return HUB_MODE_DEGRADED_CAM_ONLY;
    if (imu_fresh)
        return HUB_MODE_DEGRADED_IMU_ONLY;
    return HUB_MODE_NO_SENSORS;
}

const char *hub_mode_name(enum hub_mode m)
{
    switch (m) {
    case HUB_MODE_OK:                return "OK";
    case HUB_MODE_DEGRADED_CAM_ONLY: return "DEGRADED(cam-only)";
    case HUB_MODE_DEGRADED_IMU_ONLY: return "DEGRADED(imu-only)";
    case HUB_MODE_NO_SENSORS:        return "NO_SENSORS";
    }
    return "UNKNOWN";
}
