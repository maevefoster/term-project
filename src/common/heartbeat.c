#include "heartbeat.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int heartbeat_init(struct heartbeat *hb, const char *proc)
{
    const char *env = getenv(HEARTBEAT_ENV);
    char *end;
    long v;
    int fl;

    memset(hb, 0, sizeof *hb);
    hb->fd = -1;
    if (!env)
        return 0;

    errno = 0;
    v = strtol(env, &end, 10);
    if (errno != 0 || end == env || *end != '\0' || v < 3 || v > INT_MAX) {
        fprintf(stderr, "%s: ignoring %s=%s (not an fd above stderr)\n", proc,
                HEARTBEAT_ENV, env);
        return 0;
    }
    fl = fcntl((int)v, F_GETFL);
    if (fl < 0 || fcntl((int)v, F_SETFL, fl | O_NONBLOCK) < 0) {
        fprintf(stderr, "%s: heartbeat fd %ld unusable: %s\n", proc, v, strerror(errno));
        return 0;
    }
    if (signal(SIGPIPE, SIG_IGN) == SIG_ERR) {
        fprintf(stderr, "%s: cannot ignore SIGPIPE: %s\n", proc, strerror(errno));
        return 0;
    }
    hb->fd = (int)v;
    return 1;
}

void heartbeat_beat(struct heartbeat *hb, int64_t now_ns)
{
    const char byte = 'h';
    ssize_t n;

    if (hb->fd < 0 || now_ns < hb->next_ns)
        return;
    hb->next_ns = now_ns + HEARTBEAT_PERIOD_NS;

    do
        n = write(hb->fd, &byte, 1);
    while (n < 0 && errno == EINTR);

    if (n == 1) {
        hb->sent++;
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        /* EPIPE: the supervisor is gone (PR_SET_PDEATHSIG will usually stop
         * us first). Stop trying rather than fail every second. */
        fprintf(stderr, "heartbeat: fd %d write failed: %s; heartbeats off\n", hb->fd,
                strerror(errno));
        close(hb->fd);
        hb->fd = -1;
    }
}
