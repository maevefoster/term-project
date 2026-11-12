#include "ipc_client.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "timeutil.h"

#define IPC_RETRY_NS NS_PER_S

int ipc_client_init(struct ipc_client *cl, const char *run_dir, const char *sock_name,
                    const char *proc)
{
    int n;

    memset(cl, 0, sizeof *cl);
    cl->fd = -1;
    n = snprintf(cl->path, sizeof cl->path, "%s/%s", run_dir, sock_name);
    if (n < 0 || (size_t)n >= sizeof cl->path) {
        errno = ENAMETOOLONG;
        return -1;
    }
    n = snprintf(cl->proc, sizeof cl->proc, "%s", proc);
    if (n < 0 || (size_t)n >= sizeof cl->proc) {
        errno = EINVAL;
        return -1;
    }
    return 0;
}

void ipc_client_close(struct ipc_client *cl)
{
    if (cl->fd >= 0 && close(cl->fd) < 0)
        fprintf(stderr, "%s: ipc close: %s\n", cl->proc, strerror(errno));
    cl->fd = -1;
}

static void try_connect(struct ipc_client *cl, int64_t now)
{
    struct sockaddr_un addr;
    int fd;

    if (now < cl->next_try_ns)
        return;
    cl->next_try_ns = now + IPC_RETRY_NS;

    fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        fprintf(stderr, "%s: ipc socket: %s\n", cl->proc, strerror(errno));
        return;
    }
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    memcpy(addr.sun_path, cl->path, sizeof cl->path);
    if (connect(fd, (const struct sockaddr *)&addr, sizeof addr) < 0) {
        /* ENOENT/ECONNREFUSED: hub not up yet. Retry next second, quietly. */
        close(fd);
        return;
    }
    cl->fd = fd;
    fprintf(stderr, "%s: %s hub at %s\n", cl->proc,
            cl->connected_once ? "reconnected to" : "connected to", cl->path);
    cl->connected_once = 1;
}

int ipc_send(struct ipc_client *cl, struct msg_hdr *msg, size_t len)
{
    msg->seq = cl->seq++;

    if (cl->fd < 0)
        try_connect(cl, mono_ns());
    if (cl->fd < 0) {
        cl->dropped++;
        return -1;
    }
    for (;;) {
        ssize_t n = send(cl->fd, msg, len, MSG_DONTWAIT | MSG_NOSIGNAL);

        if (n == (ssize_t)len) {
            cl->sent++;
            return 0;
        }
        if (n < 0 && errno == EINTR)
            continue;
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            /* EPIPE/ECONNRESET/ENOTCONN: hub went away. Reconnect later. */
            fprintf(stderr, "%s: lost hub connection: %s\n", cl->proc, strerror(errno));
            ipc_client_close(cl);
        }
        cl->dropped++;
        return -1;
    }
}

void ipc_event(struct ipc_client *cl, const char *name, const char *fmt, ...)
{
    struct msg_event e;
    va_list ap;
    int n;

    memset(&e, 0, sizeof e);
    msg_init_hdr(&e.h, MSG_EVENT, SENSOR_OK);
    n = snprintf(e.name, sizeof e.name, "%s", name);
    if (n < 0 || (size_t)n >= sizeof e.name) {
        fprintf(stderr, "%s: event name too long: %s\n", cl->proc, name);
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(e.body, sizeof e.body, fmt, ap);
    va_end(ap);
    if (n < 0)
        e.body[0] = '\0';
    else if ((size_t)n >= sizeof e.body)
        snprintf(e.body, sizeof e.body, "truncated=1"); /* same rule as applog */

    fprintf(stderr, "%s %s %s\n", cl->proc, e.name, e.body);
    ipc_send(cl, &e.h, sizeof e);
}
