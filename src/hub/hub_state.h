/* Per-source freshness and the hub's sensor mode. Pure bookkeeping (no
 * sockets, no clocks), so it can be unit-tested on any host.
 *
 * This decides only which sensors are usable. Posture decisions (GOOD,
 * SLOUCH, AWAY) belong to the student-written fusion state machine.
 */
#ifndef POSTURE_HUB_STATE_H
#define POSTURE_HUB_STATE_H

#include <stdint.h>

#include "msg.h"
#include "timeutil.h"

/* A source with no message for longer than this is stale. IMU: M2 design
 * (2 s). Camera: team decision, same value (DESIGN.md, Step 1). */
#define HUB_STALE_NS (2 * NS_PER_S)

enum hub_mode {
    HUB_MODE_OK,                 /* both sensors usable */
    HUB_MODE_DEGRADED_CAM_ONLY,  /* IMU unusable */
    HUB_MODE_DEGRADED_IMU_ONLY,  /* camera unusable */
    HUB_MODE_NO_SENSORS,         /* neither: no coaching, no alerts */
};

struct hub_source {
    int connected;
    int have_msg;            /* a data message arrived on this connection */
    int64_t last_rx_ns;      /* hub's CLOCK_MONOTONIC at the last data message */
    enum sensor_status status;
    int usable;              /* last data message carried usable data */

    int have_seq;
    uint32_t last_seq;
    uint64_t msgs, rejects, seq_gaps;
};

/* New connection: forget the previous sender's data and sequence. */
void hub_source_connect(struct hub_source *s);
void hub_source_disconnect(struct hub_source *s);

/* Counts messages missing between the last seq and this one. A seq at or
 * below the last one means the sender restarted: no gap is counted. */
void hub_source_note_seq(struct hub_source *s, uint32_t seq);

/* Records a data message received at `now_ns`. */
void hub_source_data(struct hub_source *s, int64_t now_ns, enum sensor_status status,
                     int usable);

int hub_source_fresh(const struct hub_source *s, int64_t now_ns);

/* Why a source is or isn't usable: "ok", "disconnected", "no_data", "stale",
 * a status name such as "ABSENT", or "no_cva" (camera OK, no valid CVA). */
const char *hub_source_why(const struct hub_source *s, int64_t now_ns);

enum hub_mode hub_mode_eval(int imu_fresh, int cam_fresh);
const char *hub_mode_name(enum hub_mode m);

#endif
