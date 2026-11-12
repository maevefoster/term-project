/* Messages between posture-coach processes.
 *
 * Sensor processes send these to hub over SOCK_SEQPACKET Unix sockets, one
 * listening socket per source (imu.sock, cam.sock), so each recv() is
 * exactly one message. All processes run on the same Pi and are built by the
 * same compiler, so structs go over the socket as-is; every struct has a
 * fixed size and explicit padding, checked below. Bump MSG_VERSION on any
 * change to a layout.
 */
#ifndef POSTURE_MSG_H
#define POSTURE_MSG_H

#include <stddef.h>
#include <stdint.h>

#define MSG_VERSION 2

enum sensor_status {
    SENSOR_OK = 0,
    SENSOR_ABSENT = 1,     /* device missing or unreadable; process stays alive */
    SENSOR_NO_MARKERS = 2, /* camera only: frames arrive but no markers found */
};

static inline const char *sensor_status_name(enum sensor_status s)
{
    switch (s) {
    case SENSOR_OK:         return "OK";
    case SENSOR_ABSENT:     return "ABSENT";
    case SENSOR_NO_MARKERS: return "NO_MARKERS";
    }
    return "UNKNOWN";
}

enum msg_type {
    MSG_IMU_DATA = 1, /* imu_proc, 10 Hz */
    MSG_CAM_DATA = 2, /* cam_proc, 5 Hz */
    MSG_EVENT = 3,    /* any sender; hub writes it to the log */
};

/* Which socket a message arrived on; decides the types allowed. */
enum msg_source {
    MSG_SRC_IMU = 0,
    MSG_SRC_CAM = 1,
};

#define MSG_EVENT_NAME_MAX 32  /* including NUL */
#define MSG_EVENT_BODY_MAX 200 /* including NUL */

struct msg_hdr {
    uint16_t version;
    uint16_t type;    /* enum msg_type */
    uint32_t seq;     /* per sender, +1 per message sent or dropped */
    int64_t mono_ns;  /* sender's CLOCK_MONOTONIC when the message was built */
    uint8_t status;   /* enum sensor_status; SENSOR_OK for events */
    uint8_t pad[7];
};

struct msg_imu_data {
    struct msg_hdr h;
    float pitch_deg;     /* valid only if pitch_valid */
    uint8_t pitch_valid; /* 0 until the complementary filter exists */
    uint8_t pad[3];
};

struct msg_cam_data {
    struct msg_hdr h;
    int64_t obs_mono_ns; /* CLOCK_MONOTONIC of the frame behind cva_deg */
    float cva_deg;       /* valid only if cva_valid */
    float confidence;    /* 0..1, valid only if cva_valid */
    uint8_t cva_valid;
    uint8_t pad[7];
};

struct msg_event {
    struct msg_hdr h;
    char name[MSG_EVENT_NAME_MAX]; /* [A-Z0-9_]+, NUL-terminated */
    char body[MSG_EVENT_BODY_MAX]; /* key=value text, NUL-terminated */
};

_Static_assert(sizeof(struct msg_hdr) == 24, "msg_hdr layout");
_Static_assert(sizeof(struct msg_imu_data) == 32, "msg_imu_data layout");
_Static_assert(sizeof(struct msg_cam_data) == 48, "msg_cam_data layout");
_Static_assert(sizeof(struct msg_event) == 256, "msg_event layout");

/* Largest message, for receive buffers. */
#define MSG_MAX_SIZE sizeof(struct msg_event)

/* Zeroes the header and fills version, type, status and the current
 * CLOCK_MONOTONIC time. The sender sets seq. */
void msg_init_hdr(struct msg_hdr *h, enum msg_type type, enum sensor_status status);

/* Checks a received message of `len` bytes from `src`. Returns NULL if it is
 * well formed, otherwise a short reason for the reject counter and log. */
const char *msg_validate(const void *buf, size_t len, enum msg_source src);

#endif
