#include "mpu6050.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define PWR_CLKSEL_PLL_XGYRO 0x01 /* SLEEP clear, PLL with X gyro reference */
#define GYRO_FS_SEL_250      0x00
#define ACCEL_AFS_SEL_2G     0x00

/* Wait before checking setup: covers the longest register start-up time
 * listed in the MPU-6050 product specification (100 ms). */
#define WAKE_SETTLE_NS 100000000L

#define ACCEL_LSB_PER_G    16384.0
#define GYRO_LSB_PER_DPS   131.0
#define TEMP_LSB_PER_C     340.0
#define TEMP_OFFSET_C      36.53

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;

    do
        r = ioctl(fd, req, arg);
    while (r < 0 && errno == EINTR);
    return r;
}

static int reg_read(struct mpu_dev *d, uint8_t reg, uint8_t *buf, uint16_t len)
{
    struct i2c_msg msgs[2] = {
        { .addr = d->addr, .flags = 0, .len = 1, .buf = &reg },
        { .addr = d->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };

    return xioctl(d->fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static int reg_write(struct mpu_dev *d, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { .addr = d->addr, .flags = 0, .len = 2, .buf = buf };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };

    return xioctl(d->fd, I2C_RDWR, &xfer) < 0 ? -1 : 0;
}

static enum mpu_result setup_fail(struct mpu_dev *d, const char *op, int err,
                                  enum mpu_result r)
{
    mpu_close(d);
    d->last_op = op;
    d->last_errno = err;
    return r;
}

void mpu_init(struct mpu_dev *d)
{
    memset(d, 0, sizeof *d);
    d->fd = -1;
    d->last_op = "none";
}

void mpu_close(struct mpu_dev *d)
{
    if (d->fd >= 0 && close(d->fd) < 0)
        fprintf(stderr, "mpu: close: %s\n", strerror(errno));
    d->fd = -1;
}

int mpu_burst_all_zero(const uint8_t burst[MPU_BURST_LEN])
{
    for (size_t i = 0; i < MPU_BURST_LEN; i++)
        if (burst[i] != 0)
            return 0;
    return 1;
}

/* A chip that reset during setup still answers on the bus, but sleeps and
 * returns zeros, so a write being acknowledged doesn't prove it took effect. */
static enum mpu_result verify_awake(struct mpu_dev *d)
{
    const struct timespec settle = { .tv_sec = 0, .tv_nsec = WAKE_SETTLE_NS };
    uint8_t burst[MPU_BURST_LEN];
    uint8_t pwr;

    /* EINTR is fine: a short wait can only fail this check and retry. */
    (void)nanosleep(&settle, NULL);

    if (reg_read(d, REG_PWR_MGMT_1, &pwr, 1) < 0)
        return setup_fail(d, "verify_pwr_mgmt_1", errno, MPU_GONE);
    if (pwr != PWR_CLKSEL_PLL_XGYRO)
        return setup_fail(d, "verify_awake", 0, MPU_GONE);
    if (reg_read(d, REG_ACCEL_XOUT_H, burst, sizeof burst) < 0)
        return setup_fail(d, "verify_burst", errno, MPU_GONE);
    if (mpu_burst_all_zero(burst))
        return setup_fail(d, "verify_zero_data", 0, MPU_GONE);
    return MPU_OK;
}

enum mpu_result mpu_open(struct mpu_dev *d, const char *path, uint16_t addr)
{
    mpu_close(d);
    d->addr = addr;
    d->who_am_i = 0;

    d->fd = open(path, O_RDWR | O_CLOEXEC);
    if (d->fd < 0) {
        int err = errno;

        /* A permission or path-type problem won't fix itself by retrying. */
        if (err == EACCES || err == EPERM || err == ENOTDIR)
            return setup_fail(d, "open", err, MPU_FATAL);
        return setup_fail(d, "open", err, MPU_GONE);
    }

    /* Every bus error below means the chip isn't answering: unplugged. */
    if (reg_read(d, REG_WHO_AM_I, &d->who_am_i, 1) < 0)
        return setup_fail(d, "who_am_i", errno, MPU_GONE);
    if (d->who_am_i != MPU_WHO_AM_I_ID)
        return setup_fail(d, "who_am_i_mismatch", 0, MPU_GONE);

    if (reg_write(d, REG_PWR_MGMT_1, PWR_CLKSEL_PLL_XGYRO) < 0)
        return setup_fail(d, "pwr_mgmt_1", errno, MPU_GONE);
    if (reg_write(d, REG_GYRO_CONFIG, GYRO_FS_SEL_250) < 0)
        return setup_fail(d, "gyro_config", errno, MPU_GONE);
    if (reg_write(d, REG_ACCEL_CONFIG, ACCEL_AFS_SEL_2G) < 0)
        return setup_fail(d, "accel_config", errno, MPU_GONE);
    return verify_awake(d);
}

int mpu_read_raw(struct mpu_dev *d, struct mpu_raw *out)
{
    uint8_t burst[MPU_BURST_LEN];

    if (reg_read(d, REG_ACCEL_XOUT_H, burst, sizeof burst) < 0) {
        d->last_op = "burst_read";
        d->last_errno = errno;
        return -1;
    }
    /* The bus is fine but the chip reset and went back to sleep. */
    if (mpu_burst_all_zero(burst)) {
        d->last_op = "all_zero_burst";
        d->last_errno = 0;
        return -1;
    }
    mpu_parse(burst, out);
    return 0;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void mpu_parse(const uint8_t burst[MPU_BURST_LEN], struct mpu_raw *out)
{
    out->ax = be16(burst + 0);
    out->ay = be16(burst + 2);
    out->az = be16(burst + 4);
    out->temp = be16(burst + 6);
    out->gx = be16(burst + 8);
    out->gy = be16(burst + 10);
    out->gz = be16(burst + 12);
}

void mpu_scale(const struct mpu_raw *raw, struct mpu_reading *out)
{
    out->ax_g = raw->ax / ACCEL_LSB_PER_G;
    out->ay_g = raw->ay / ACCEL_LSB_PER_G;
    out->az_g = raw->az / ACCEL_LSB_PER_G;
    out->gx_dps = raw->gx / GYRO_LSB_PER_DPS;
    out->gy_dps = raw->gy / GYRO_LSB_PER_DPS;
    out->gz_dps = raw->gz / GYRO_LSB_PER_DPS;
    out->temp_c = raw->temp / TEMP_LSB_PER_C + TEMP_OFFSET_C;
}
