/* Host unit tests for burst decoding and unit conversion in
 * src/imu_proc/mpu6050.c (no hardware). */
#include <stdio.h>

#include "mpu6050.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static int near(double a, double b)
{
    double d = a - b;

    return d > -1e-6 && d < 1e-6;
}

static void test_parse_order_and_endianness(void)
{
    static const uint8_t burst[MPU_BURST_LEN] = {
        0x01, 0x02, /* ax */
        0x03, 0x04, /* ay */
        0x05, 0x06, /* az */
        0x07, 0x08, /* temp */
        0x09, 0x0A, /* gx */
        0x0B, 0x0C, /* gy */
        0x0D, 0x0E, /* gz */
    };
    struct mpu_raw r;

    mpu_parse(burst, &r);
    CHECK(r.ax == 0x0102);
    CHECK(r.ay == 0x0304);
    CHECK(r.az == 0x0506);
    CHECK(r.temp == 0x0708);
    CHECK(r.gx == 0x090A);
    CHECK(r.gy == 0x0B0C);
    CHECK(r.gz == 0x0D0E);
}

static void test_parse_negative(void)
{
    static const uint8_t burst[MPU_BURST_LEN] = {
        0xFF, 0xFF, /* ax = -1 */
        0x80, 0x00, /* ay = -32768 */
        0x7F, 0xFF, /* az = 32767 */
    };
    struct mpu_raw r;

    mpu_parse(burst, &r);
    CHECK(r.ax == -1);
    CHECK(r.ay == -32768);
    CHECK(r.az == 32767);
    CHECK(r.temp == 0);
}

static void test_all_zero(void)
{
    static const uint8_t zeros[MPU_BURST_LEN] = { 0 };
    static const uint8_t first[MPU_BURST_LEN] = { 0x01 };
    static const uint8_t last[MPU_BURST_LEN] = { [MPU_BURST_LEN - 1] = 0x01 };

    CHECK(mpu_burst_all_zero(zeros));
    CHECK(!mpu_burst_all_zero(first));
    CHECK(!mpu_burst_all_zero(last));
}

static void test_scale(void)
{
    /* Register map: 16384 LSB/g at +-2 g, 131 LSB/(deg/s) at +-250 deg/s,
     * temperature = raw / 340 + 36.53. */
    struct mpu_raw r = {
        .ax = 16384, .ay = -32768, .az = 0,
        .temp = -340,
        .gx = 131, .gy = -262, .gz = 0,
    };
    struct mpu_reading v;

    mpu_scale(&r, &v);
    CHECK(near(v.ax_g, 1.0));
    CHECK(near(v.ay_g, -2.0));
    CHECK(near(v.az_g, 0.0));
    CHECK(near(v.gx_dps, 1.0));
    CHECK(near(v.gy_dps, -2.0));
    CHECK(near(v.gz_dps, 0.0));
    CHECK(near(v.temp_c, 35.53));
}

int main(void)
{
    test_parse_order_and_endianness();
    test_parse_negative();
    test_all_zero();
    test_scale();

    if (failures) {
        fprintf(stderr, "test_mpu6050: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_mpu6050: all checks passed\n");
    return 0;
}
