/* Restart and watchdog policy for the supervisor. Pure functions (no clocks,
 * no processes), so they can be unit-tested on any host. All times are
 * CLOCK_MONOTONIC nanoseconds.
 */
#ifndef POSTURE_SUP_POLICY_H
#define POSTURE_SUP_POLICY_H

#include <stdint.h>

#include "timeutil.h"

/* Restart backoff (M2 design): 1 s, doubling, capped at 30 s. */
#define SUP_BACKOFF_FIRST_NS NS_PER_S
#define SUP_BACKOFF_MAX_NS   (30 * NS_PER_S)

/* A child that ran at least this long before exiting counts as a fresh
 * failure, so its next restart is back to 1 s (team decision). */
#define SUP_STABLE_NS (60 * NS_PER_S)

/* Watchdog: first heartbeat within 5 s of start (team rule), then no more
 * than 3 s between heartbeats (M2 design). */
#define SUP_FIRST_HB_NS   (5 * NS_PER_S)
#define SUP_HB_SILENCE_NS (3 * NS_PER_S)

/* Consecutive failures after a child that ran for `ran_ns` exits. */
unsigned sup_next_failures(unsigned failures, int64_t ran_ns);

/* Delay before restart number `failures` (1 = first): 1 s, 2 s, 4 s, ... 30 s. */
int64_t sup_backoff_ns(unsigned failures);

/* Time after which a silent child is killed: SUP_FIRST_HB_NS after start
 * until its first heartbeat, then SUP_HB_SILENCE_NS after the latest. */
int64_t sup_watchdog_deadline(int64_t started_ns, int got_hb, int64_t last_hb_ns);

#endif
