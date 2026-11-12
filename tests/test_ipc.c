/* Socket test for src/common/ipc_client.c: a real SOCK_SEQPACKET listener in
 * a temporary directory stands in for hub. No hardware needed. */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "ipc_client.h"
#include "msg.h"

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                    #cond);                                                  \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static char g_dir[64];
static char g_path[100]; /* fits sun_path (108) */

static int make_listener(void)
{
    struct sockaddr_un addr;
    int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);

    if (fd < 0)
        return -1;
    memset(&addr, 0, sizeof addr);
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof addr.sun_path, "%s", g_path);
    if (bind(fd, (const struct sockaddr *)&addr, sizeof addr) < 0 || listen(fd, 4) < 0) {
        perror("listener");
        close(fd);
        return -1;
    }
    return fd;
}

static void send_imu(struct ipc_client *cl, int *rc)
{
    struct msg_imu_data m;

    memset(&m, 0, sizeof m);
    msg_init_hdr(&m.h, MSG_IMU_DATA, SENSOR_OK);
    *rc = ipc_send(cl, &m.h, sizeof m);
}

int main(void)
{
    struct ipc_client cl;
    union {
        struct msg_event ev;
        unsigned char raw[MSG_MAX_SIZE];
    } buf;
    struct msg_hdr h;
    int lfd, cfd, rc;
    ssize_t n;

    snprintf(g_dir, sizeof g_dir, "/tmp/ipc_test.XXXXXX");
    if (!mkdtemp(g_dir)) {
        perror("mkdtemp");
        return 1;
    }
    snprintf(g_path, sizeof g_path, "%s/imu.sock", g_dir);

    CHECK(ipc_client_init(&cl, g_dir, "imu.sock", "test_ipc") == 0);

    /* No hub yet: dropped, and the seq is still used up. */
    send_imu(&cl, &rc);
    CHECK(rc == -1);
    CHECK(cl.dropped == 1);
    CHECK(cl.seq == 1);

    lfd = make_listener();
    CHECK(lfd >= 0);
    if (lfd < 0)
        goto out;

    cl.next_try_ns = 0; /* skip the 1 s retry wait */
    send_imu(&cl, &rc);
    CHECK(rc == 0);
    cfd = accept(lfd, NULL, NULL);
    CHECK(cfd >= 0);
    if (cfd < 0)
        goto out_listener;

    n = recv(cfd, &buf, sizeof buf, 0);
    CHECK(n == (ssize_t)sizeof(struct msg_imu_data));
    CHECK(msg_validate(&buf, (size_t)n, MSG_SRC_IMU) == NULL);
    memcpy(&h, &buf, sizeof h);
    CHECK(h.seq == 1); /* seq 0 was the dropped message */

    ipc_event(&cl, "TEST_EV", "a=%d b=%s", 7, "x");
    n = recv(cfd, &buf, sizeof buf, 0);
    CHECK(n == (ssize_t)sizeof(struct msg_event));
    CHECK(msg_validate(&buf, (size_t)n, MSG_SRC_IMU) == NULL);
    CHECK(strcmp(buf.ev.name, "TEST_EV") == 0);
    CHECK(strcmp(buf.ev.body, "a=7 b=x") == 0);
    CHECK(buf.ev.h.seq == 2);

    /* Hub goes away: the next send fails, the client closes its socket and
     * keeps running. */
    close(cfd);
    send_imu(&cl, &rc);
    CHECK(rc == -1);
    CHECK(cl.fd < 0);
    CHECK(cl.sent == 2);
    CHECK(cl.dropped == 2);

out_listener:
    close(lfd);
out:
    ipc_client_close(&cl);
    if (unlink(g_path) < 0 && errno != ENOENT)
        perror(g_path);
    if (rmdir(g_dir) < 0)
        perror(g_dir);
    if (failures) {
        fprintf(stderr, "test_ipc: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_ipc: all checks passed\n");
    return 0;
}
