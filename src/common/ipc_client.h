/* Sending side of the hub sockets, used by imu_proc and cam_proc.
 *
 * Never blocks the sensor loop: connects lazily, retries at most once per
 * second, and sends with MSG_DONTWAIT. A message that can't go out right now
 * (hub busy, restarting or not started) is dropped and counted; its seq is
 * still used up, so hub sees the gap.
 */
#ifndef POSTURE_IPC_CLIENT_H
#define POSTURE_IPC_CLIENT_H

#include <stddef.h>
#include <stdint.h>

#include "msg.h"

#define IPC_DEFAULT_RUN_DIR "/run/posture"
#define IPC_PATH_MAX 108 /* sizeof sockaddr_un.sun_path on Linux */

struct ipc_client {
    int fd;
    char path[IPC_PATH_MAX];
    char proc[32];        /* this process's name, for stderr lines */
    uint32_t seq;
    int64_t next_try_ns;  /* earliest next connect attempt */
    int connected_once;   /* for "reconnected" vs "connected" on stderr */
    uint64_t sent, dropped;
};

/* Socket path is "<run_dir>/<sock_name>". Returns 0, or -1 if too long. */
int ipc_client_init(struct ipc_client *cl, const char *run_dir, const char *sock_name,
                    const char *proc);

/* Sets msg->seq and sends `len` bytes. Returns 0 if sent, -1 if dropped. */
int ipc_send(struct ipc_client *cl, struct msg_hdr *msg, size_t len);

/* Sends an EVENT that hub logs as "<proc> <name> <body>", and prints the same
 * line to stderr (the journal) so nothing is lost while hub is down. */
void ipc_event(struct ipc_client *cl, const char *name, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

void ipc_client_close(struct ipc_client *cl);

#endif
