/* Host unit tests for src/hub/hub_state.c (no sockets, no clocks). */
#include <stdio.h>
#include <string.h>

#include "hub_state.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

#define T0 (100 * NS_PER_S)

static void test_freshness(void)
{
    struct hub_source s;

    memset(&s, 0, sizeof s);
    CHECK(!hub_source_fresh(&s, T0));
    CHECK(strcmp(hub_source_why(&s, T0), "disconnected") == 0);

    hub_source_connect(&s);
    CHECK(!hub_source_fresh(&s, T0));
    CHECK(strcmp(hub_source_why(&s, T0), "no_data") == 0);

    hub_source_data(&s, T0, SENSOR_OK, 1);
    CHECK(hub_source_fresh(&s, T0));
    CHECK(hub_source_fresh(&s, T0 + HUB_STALE_NS));         /* limit is inclusive */
    CHECK(!hub_source_fresh(&s, T0 + HUB_STALE_NS + 1));
    CHECK(strcmp(hub_source_why(&s, T0 + HUB_STALE_NS + 1), "stale") == 0);
    CHECK(strcmp(hub_source_why(&s, T0), "ok") == 0);

    /* Alive and reporting ABSENT: not usable, but not "disconnected". */
    hub_source_data(&s, T0, SENSOR_ABSENT, 0);
    CHECK(!hub_source_fresh(&s, T0));
    CHECK(strcmp(hub_source_why(&s, T0), "ABSENT") == 0);

    /* Camera OK but no valid CVA. */
    hub_source_data(&s, T0, SENSOR_OK, 0);
    CHECK(!hub_source_fresh(&s, T0));
    CHECK(strcmp(hub_source_why(&s, T0), "no_cva") == 0);

    hub_source_data(&s, T0, SENSOR_OK, 1);
    hub_source_disconnect(&s);
    CHECK(!hub_source_fresh(&s, T0));
    CHECK(strcmp(hub_source_why(&s, T0), "disconnected") == 0);

    /* A new connection must not inherit the old sender's data. */
    hub_source_connect(&s);
    CHECK(!hub_source_fresh(&s, T0));
}

static void test_seq_gaps(void)
{
    struct hub_source s;

    memset(&s, 0, sizeof s);
    hub_source_connect(&s);
    hub_source_note_seq(&s, 10);
    hub_source_note_seq(&s, 11);
    CHECK(s.seq_gaps == 0);
    hub_source_note_seq(&s, 14); /* 12, 13 missing */
    CHECK(s.seq_gaps == 2);
    hub_source_note_seq(&s, 0);  /* sender restarted */
    CHECK(s.seq_gaps == 2);
    hub_source_note_seq(&s, 1);
    CHECK(s.seq_gaps == 2);

    hub_source_note_seq(&s, 0xFFFFFFFFu);
    hub_source_note_seq(&s, 0);  /* wraps: consecutive */
    CHECK(s.seq_gaps == 2);

    hub_source_connect(&s);      /* new connection: sequence starts over */
    hub_source_note_seq(&s, 500);
    CHECK(s.seq_gaps == 2);
}

static void test_mode(void)
{
    CHECK(hub_mode_eval(1, 1) == HUB_MODE_OK);
    CHECK(hub_mode_eval(0, 1) == HUB_MODE_DEGRADED_CAM_ONLY);
    CHECK(hub_mode_eval(1, 0) == HUB_MODE_DEGRADED_IMU_ONLY);
    CHECK(hub_mode_eval(0, 0) == HUB_MODE_NO_SENSORS);
    CHECK(strcmp(hub_mode_name(HUB_MODE_DEGRADED_CAM_ONLY), "DEGRADED(cam-only)") == 0);
    CHECK(strcmp(hub_mode_name(HUB_MODE_DEGRADED_IMU_ONLY), "DEGRADED(imu-only)") == 0);
}

int main(void)
{
    test_freshness();
    test_seq_gaps();
    test_mode();

    if (failures) {
        fprintf(stderr, "test_hub_state: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_hub_state: all checks passed\n");
    return 0;
}
