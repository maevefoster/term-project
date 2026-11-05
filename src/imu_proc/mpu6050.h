/* Minimal MPU-6050 access over Linux i2c-dev.
 *
 * Register addresses and scale factors are from the InvenSense "MPU-6000
 * and MPU-6050 Register Map and Descriptions" (RM-MPU-6000A-00, rev 4.2).
 * This module only moves bytes and converts units; no tilt or posture
 * logic belongs here.
 */
#ifndef POSTURE_MPU6050_H
#define POSTURE_MPU6050_H

#include <stddef.h>
#include <stdint.h>

#define MPU_DEFAULT_ADDR 0x68
#define MPU_WHO_AM_I_ID  0x68 /* expected WHO_AM_I contents */
#define MPU_BURST_LEN    14   /* ACCEL_XOUT_H .. GYRO_ZOUT_L */

/* Full-scale ranges written by mpu_open(), for logging. */
#define MPU_ACCEL_FS_G   2
#define MPU_GYRO_FS_DPS  250

enum mpu_result {
    MPU_OK = 0,
    MPU_GONE = -1,  /* bus or chip missing, or wrong chip; retry later */
    MPU_FATAL = -2, /* unusable setup, e.g. no permission on the device */
};

struct mpu_raw {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
};

struct mpu_reading {
    double ax_g, ay_g, az_g;
    double gx_dps, gy_dps, gz_dps;
    double temp_c;
};

struct mpu_dev {
    int fd;
    uint16_t addr;
    uint8_t who_am_i; /* last value read; 0 if the read never succeeded */

    /* Last failure, for logging. Kept across mpu_close(). */
    const char *last_op;
    int last_errno;
};

void mpu_init(struct mpu_dev *d);

/* Opens the bus, checks WHO_AM_I, wakes the chip and sets the ranges, then
 * waits 100 ms and confirms the chip is awake and returning data. Must run
 * again after any reconnect: a power-cycled chip comes back asleep. On
 * failure the device is closed again. */
enum mpu_result mpu_open(struct mpu_dev *d, const char *path, uint16_t addr);

/* Reads all sensors in one I2C_RDWR burst. Returns 0, or -1 with last_op and
 * last_errno set, for a bus error or an all-zero burst (last_errno 0). The
 * device stays open: the caller counts failures. */
int mpu_read_raw(struct mpu_dev *d, struct mpu_raw *out);

void mpu_close(struct mpu_dev *d);

/* True if every byte of a burst is zero, as a sleeping chip returns. An awake
 * chip always shows gravity, so this never happens with real data. */
int mpu_burst_all_zero(const uint8_t burst[MPU_BURST_LEN]);

/* Decodes a burst: big-endian pairs of accel xyz, temperature, gyro xyz. */
void mpu_parse(const uint8_t burst[MPU_BURST_LEN], struct mpu_raw *out);

/* Converts raw counts to g, degrees/s and degrees C for the ranges above. */
void mpu_scale(const struct mpu_raw *raw, struct mpu_reading *out);

#endif
