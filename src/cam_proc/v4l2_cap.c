#include "v4l2_cap.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

static int xioctl(int fd, unsigned long req, void *arg)
{
    int r;

    do
        r = ioctl(fd, req, arg);
    while (r < 0 && errno == EINTR);
    return r;
}

/* Errors that mean the camera is unplugged, not yet enumerated, or busy:
 * report ABSENT and retry. Anything else at setup is a configuration bug. */
static enum cam_result classify(int err)
{
    switch (err) {
    case ENOENT:
    case ENODEV:
    case ENXIO:
    case EIO:
    case EBUSY:
        return CAM_GONE;
    default:
        return CAM_FATAL;
    }
}

static enum cam_result setup_fail(struct cam_dev *d, const char *op, int err)
{
    cam_close(d);
    d->last_op = op;
    d->last_errno = err;
    return classify(err);
}

/* Once streaming, every failure is treated as the device going away; the
 * caller reopens on its retry schedule. */
static enum cam_result runtime_fail(struct cam_dev *d, const char *op, int err)
{
    cam_close(d);
    d->last_op = op;
    d->last_errno = err;
    return CAM_GONE;
}

void cam_init(struct cam_dev *d)
{
    memset(d, 0, sizeof *d);
    d->fd = -1;
    d->last_op = "none";
}

void cam_close(struct cam_dev *d)
{
    if (d->fd >= 0 && d->streaming) {
        enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        /* Fails harmlessly if the device was unplugged. */
        (void)xioctl(d->fd, VIDIOC_STREAMOFF, &t);
    }
    for (unsigned i = 0; i < d->n_bufs; i++)
        if (munmap(d->bufs[i].start, d->bufs[i].length) < 0)
            fprintf(stderr, "cam: munmap buffer %u: %s\n", i, strerror(errno));
    if (d->fd >= 0 && close(d->fd) < 0)
        fprintf(stderr, "cam: close: %s\n", strerror(errno));
    d->fd = -1;
    d->streaming = 0;
    d->n_bufs = 0;
    memset(d->bufs, 0, sizeof d->bufs);
}

static void set_frame_rate(struct cam_dev *d, uint32_t fps)
{
    struct v4l2_streamparm parm;

    d->tpf_num = 0;
    d->tpf_den = 0;
    memset(&parm, 0, sizeof parm);
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(d->fd, VIDIOC_G_PARM, &parm) < 0 ||
        !(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME))
        return; /* keep the driver default; reported as 0/0 */
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (xioctl(d->fd, VIDIOC_S_PARM, &parm) < 0)
        return;
    d->tpf_num = parm.parm.capture.timeperframe.numerator;
    d->tpf_den = parm.parm.capture.timeperframe.denominator;
}

enum cam_result cam_open(struct cam_dev *d, const char *path,
                         uint32_t width, uint32_t height, uint32_t fps)
{
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    uint32_t caps;
    unsigned n;

    cam_close(d);
    d->fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (d->fd < 0)
        return setup_fail(d, "open", errno);

    memset(&cap, 0, sizeof cap);
    if (xioctl(d->fd, VIDIOC_QUERYCAP, &cap) < 0)
        return setup_fail(d, "VIDIOC_QUERYCAP", errno);
    caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE) || !(caps & V4L2_CAP_STREAMING))
        return setup_fail(d, "capabilities", ENOTSUP);

    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(d->fd, VIDIOC_S_FMT, &fmt) < 0)
        return setup_fail(d, "VIDIOC_S_FMT", errno);
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
        return setup_fail(d, "pixelformat", ENOTSUP);
    d->width = fmt.fmt.pix.width;
    d->height = fmt.fmt.pix.height;
    d->pixfmt = fmt.fmt.pix.pixelformat;
    d->bytesperline = fmt.fmt.pix.bytesperline ? fmt.fmt.pix.bytesperline : d->width * 2;
    d->sizeimage = fmt.fmt.pix.sizeimage ? fmt.fmt.pix.sizeimage : d->bytesperline * d->height;

    set_frame_rate(d, fps);

    memset(&req, 0, sizeof req);
    req.count = CAM_NBUFS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(d->fd, VIDIOC_REQBUFS, &req) < 0)
        return setup_fail(d, "VIDIOC_REQBUFS", errno);
    if (req.count < 2)
        return setup_fail(d, "VIDIOC_REQBUFS", ENOMEM);

    n = req.count < CAM_NBUFS ? req.count : CAM_NBUFS;
    for (unsigned i = 0; i < n; i++) {
        struct v4l2_buffer b;
        void *p;

        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (xioctl(d->fd, VIDIOC_QUERYBUF, &b) < 0)
            return setup_fail(d, "VIDIOC_QUERYBUF", errno);
        p = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, d->fd, (off_t)b.m.offset);
        if (p == MAP_FAILED)
            return setup_fail(d, "mmap", errno);
        d->bufs[i].start = p;
        d->bufs[i].length = b.length;
        d->n_bufs = i + 1;
        if (xioctl(d->fd, VIDIOC_QBUF, &b) < 0)
            return setup_fail(d, "VIDIOC_QBUF", errno);
    }

    if (xioctl(d->fd, VIDIOC_STREAMON, &type) < 0)
        return setup_fail(d, "VIDIOC_STREAMON", errno);
    d->streaming = 1;
    return CAM_OK;
}

enum cam_result cam_next_frame(struct cam_dev *d, int timeout_ms,
                               cam_frame_cb cb, void *ctx)
{
    struct pollfd pfd = { .fd = d->fd, .events = POLLIN };
    struct v4l2_buffer b;
    int r = poll(&pfd, 1, timeout_ms);

    if (r < 0) {
        if (errno == EINTR)
            return CAM_AGAIN;
        return runtime_fail(d, "poll", errno);
    }
    if (r == 0)
        return CAM_TIMEOUT;
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        return runtime_fail(d, "poll_revents", 0);

    memset(&b, 0, sizeof b);
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    if (xioctl(d->fd, VIDIOC_DQBUF, &b) < 0) {
        if (errno == EAGAIN)
            return CAM_AGAIN;
        return runtime_fail(d, "VIDIOC_DQBUF", errno);
    }

    if (b.index < d->n_bufs) {
        struct cam_frame f;

        (void)clock_gettime(CLOCK_MONOTONIC, &f.mono);
        f.data = d->bufs[b.index].start;
        f.bytesused = b.bytesused < d->bufs[b.index].length ? b.bytesused : d->bufs[b.index].length;
        f.sequence = b.sequence;
        f.flags = b.flags;
        f.driver_ts = b.timestamp;
        cb(&f, ctx);
    }

    if (xioctl(d->fd, VIDIOC_QBUF, &b) < 0)
        return runtime_fail(d, "VIDIOC_QBUF", errno);
    return CAM_OK;
}
