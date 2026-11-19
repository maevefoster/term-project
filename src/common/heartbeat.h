/* Child side of the supervisor watchdog (Step 2).
 *
 * The supervisor gives each child the write end of a pipe and names its fd
 * in POSTURE_HEARTBEAT_FD. The child calls heartbeat_beat() from its main
 * loop; one byte goes out at most every HEARTBEAT_PERIOD_NS (about 1 Hz).
 * Because the beat comes from the main loop, a loop stuck in a system call
 * stops the heartbeats and the supervisor restarts the process. A sensor
 * that is ABSENT keeps looping, keeps beating, and is not restarted.
 *
 * Started by hand (no POSTURE_HEARTBEAT_FD), everything here does nothing.
 */
#ifndef POSTURE_HEARTBEAT_H
#define POSTURE_HEARTBEAT_H

#include <stdint.h>

#include "timeutil.h"

#define HEARTBEAT_ENV "POSTURE_HEARTBEAT_FD"

/* Slightly under 1 s, so a loop that wakes exactly once a second (hub's
 * 1 Hz timer) still beats on every wakeup. */
#define HEARTBEAT_PERIOD_NS (900 * NS_PER_MS)

struct heartbeat {
    int fd;          /* -1: disabled */
    int64_t next_ns; /* earliest next beat */
    uint64_t sent;
};

/* Reads POSTURE_HEARTBEAT_FD and makes the fd non-blocking. When enabled it
 * also sets SIGPIPE to ignored, so a gone supervisor can't kill the process
 * through the pipe (all sockets here already use MSG_NOSIGNAL). Returns 1 if
 * heartbeats are enabled, 0 if not (a bad value is reported on stderr). */
int heartbeat_init(struct heartbeat *hb, const char *proc);

/* Sends one byte if HEARTBEAT_PERIOD_NS has passed since the last one.
 * A full pipe is skipped quietly; a closed pipe (supervisor gone) disables
 * heartbeats. Never blocks. */
void heartbeat_beat(struct heartbeat *hb, int64_t now_ns);

#endif
