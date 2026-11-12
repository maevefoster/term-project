#include "msg.h"

#include <math.h>
#include <string.h>

#include "timeutil.h"

void msg_init_hdr(struct msg_hdr *h, enum msg_type type, enum sensor_status status)
{
    memset(h, 0, sizeof *h);
    h->version = MSG_VERSION;
    h->type = (uint16_t)type;
    h->mono_ns = mono_ns();
    h->status = (uint8_t)status;
}

/* True if `s` has a NUL within `cap` bytes. */
static int terminated(const char *s, size_t cap)
{
    return memchr(s, '\0', cap) != NULL;
}

static const char *check_event(const struct msg_event *e)
{
    if (!terminated(e->name, sizeof e->name) || !terminated(e->body, sizeof e->body))
        return "event_unterminated";
    if (e->name[0] == '\0')
        return "event_name";
    for (const char *p = e->name; *p; p++)
        if (!((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9') || *p == '_'))
            return "event_name";
    if (e->h.status != SENSOR_OK)
        return "status";
    return NULL;
}

const char *msg_validate(const void *buf, size_t len, enum msg_source src)
{
    struct msg_hdr h;

    if (len < sizeof h)
        return "short";
    memcpy(&h, buf, sizeof h);
    if (h.version != MSG_VERSION)
        return "version";

    switch (h.type) {
    case MSG_IMU_DATA: {
        struct msg_imu_data m;

        if (src != MSG_SRC_IMU)
            return "type";
        if (len != sizeof m)
            return "length";
        memcpy(&m, buf, sizeof m);
        if (h.status != SENSOR_OK && h.status != SENSOR_ABSENT)
            return "status";
        if (m.pitch_valid > 1 || (m.pitch_valid && !isfinite(m.pitch_deg)))
            return "pitch";
        return NULL;
    }
    case MSG_CAM_DATA: {
        struct msg_cam_data m;

        if (src != MSG_SRC_CAM)
            return "type";
        if (len != sizeof m)
            return "length";
        memcpy(&m, buf, sizeof m);
        if (h.status > SENSOR_NO_MARKERS)
            return "status";
        if (m.cva_valid > 1 ||
            (m.cva_valid && (h.status != SENSOR_OK || !isfinite(m.cva_deg) ||
                             !(m.confidence >= 0.0f && m.confidence <= 1.0f))))
            return "cva";
        return NULL;
    }
    case MSG_EVENT: {
        struct msg_event m;

        if (len != sizeof m)
            return "length";
        memcpy(&m, buf, sizeof m);
        return check_event(&m);
    }
    default:
        return "type";
    }
}
