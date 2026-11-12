/* Host unit tests for message validation in src/common/msg.c. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "msg.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

/* Expects msg_validate to return `want` (NULL = accepted). */
#define EXPECT(buf, len, src, want)                                          \
    do {                                                                     \
        const char *got_ = msg_validate((buf), (len), (src));                \
        const char *want_ = (want);                                          \
        if (!(got_ == NULL ? want_ == NULL                                   \
                           : want_ != NULL && strcmp(got_, want_) == 0)) {   \
            fprintf(stderr, "%s:%d: msg_validate gave %s, wanted %s\n",      \
                    __FILE__, __LINE__, got_ ? got_ : "NULL",                \
                    want_ ? want_ : "NULL");                                 \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static struct msg_imu_data imu(enum sensor_status st)
{
    struct msg_imu_data m;

    memset(&m, 0, sizeof m);
    msg_init_hdr(&m.h, MSG_IMU_DATA, st);
    return m;
}

static struct msg_cam_data cam(enum sensor_status st)
{
    struct msg_cam_data m;

    memset(&m, 0, sizeof m);
    msg_init_hdr(&m.h, MSG_CAM_DATA, st);
    return m;
}

static struct msg_event event(const char *name, const char *body)
{
    struct msg_event m;

    memset(&m, 0, sizeof m);
    msg_init_hdr(&m.h, MSG_EVENT, SENSOR_OK);
    snprintf(m.name, sizeof m.name, "%s", name);
    snprintf(m.body, sizeof m.body, "%s", body);
    return m;
}

static void test_init_hdr(void)
{
    struct msg_imu_data m = imu(SENSOR_ABSENT);

    CHECK(m.h.version == MSG_VERSION);
    CHECK(m.h.type == MSG_IMU_DATA);
    CHECK(m.h.status == SENSOR_ABSENT);
    CHECK(m.h.mono_ns > 0);
}

static void test_imu(void)
{
    struct msg_imu_data m = imu(SENSOR_OK);

    EXPECT(&m, sizeof m, MSG_SRC_IMU, NULL);
    m = imu(SENSOR_ABSENT);
    EXPECT(&m, sizeof m, MSG_SRC_IMU, NULL);

    EXPECT(&m, sizeof m, MSG_SRC_CAM, "type");       /* wrong socket */
    EXPECT(&m, sizeof m - 1, MSG_SRC_IMU, "length");
    EXPECT(&m, sizeof m.h - 1, MSG_SRC_IMU, "short");

    m = imu(SENSOR_NO_MARKERS);                       /* camera-only status */
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "status");

    m = imu(SENSOR_OK);
    m.pitch_valid = 2;
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "pitch");
    m.pitch_valid = 1;
    m.pitch_deg = NAN;
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "pitch");
    m.pitch_deg = 12.5f;
    EXPECT(&m, sizeof m, MSG_SRC_IMU, NULL);

    m.h.version = MSG_VERSION + 1;
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "version");
    m.h.version = MSG_VERSION;
    m.h.type = 99;
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "type");
}

static void test_cam(void)
{
    struct msg_cam_data m = cam(SENSOR_NO_MARKERS);

    EXPECT(&m, sizeof m, MSG_SRC_CAM, NULL);
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "type");

    m = cam(SENSOR_OK);
    m.cva_valid = 1;
    m.cva_deg = 48.0f;
    m.confidence = 0.9f;
    EXPECT(&m, sizeof m, MSG_SRC_CAM, NULL);

    m.confidence = 1.5f;
    EXPECT(&m, sizeof m, MSG_SRC_CAM, "cva");
    m.confidence = 0.9f;
    m.cva_deg = INFINITY;
    EXPECT(&m, sizeof m, MSG_SRC_CAM, "cva");

    m = cam(SENSOR_ABSENT);                           /* valid CVA needs status OK */
    m.cva_valid = 1;
    m.confidence = 0.5f;
    EXPECT(&m, sizeof m, MSG_SRC_CAM, "cva");

    m = cam(SENSOR_OK);
    m.h.status = 7;
    EXPECT(&m, sizeof m, MSG_SRC_CAM, "status");
}

static void test_event(void)
{
    struct msg_event m = event("IMU_ABSENT", "dev=/dev/i2c-1 i2c_err=5");

    EXPECT(&m, sizeof m, MSG_SRC_IMU, NULL);
    EXPECT(&m, sizeof m, MSG_SRC_CAM, NULL);         /* events come from anyone */

    m = event("", "x=1");
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "event_name");
    m = event("bad name", "x=1");
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "event_name");
    m = event("lower", "x=1");
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "event_name");

    m = event("EV", "x=1");
    memset(m.body, 'a', sizeof m.body);               /* no NUL */
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "event_unterminated");

    m = event("EV", "");
    memset(m.name, 'A', sizeof m.name);
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "event_unterminated");

    m = event("EV", "");
    m.h.status = SENSOR_ABSENT;
    EXPECT(&m, sizeof m, MSG_SRC_IMU, "status");
}

int main(void)
{
    test_init_hdr();
    test_imu();
    test_cam();
    test_event();

    if (failures) {
        fprintf(stderr, "test_msg: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_msg: all checks passed\n");
    return 0;
}
