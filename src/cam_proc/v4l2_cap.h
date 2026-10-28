/* Minimal V4L2 mmap capture for cam_proc.
 *
 * Frames live only in the driver's mmap buffers: the callback sees each
 * one in place, and the buffer is requeued as soon as it returns.
 */
#ifndef POSTURE_V4L2_CAP_H
#define POSTURE_V4L2_CAP_H

#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#define CAM_NBUFS 4

enum cam_result {
    CAM_OK = 0,      /* a frame was delivered */
    CAM_AGAIN = 1,   /* nothing this time (EINTR/EAGAIN); just call again */
    CAM_TIMEOUT = 2, /* no frame within the poll timeout: a stall */
    CAM_GONE = -1,   /* device missing or failed; closed, retry later */
    CAM_FATAL = -2,  /* wrong device or unsupported config; don't retry */
};

struct cam_buf {
    void *start;
    size_t length;
};

struct cam_dev {
    int fd;
    int streaming;
    unsigned n_bufs; /* buffers mapped (and queued) */
    struct cam_buf bufs[CAM_NBUFS];

    /* What the driver actually granted. */
    uint32_t width, height, pixfmt, bytesperline, sizeimage;
    uint32_t tpf_num, tpf_den; /* frame interval; 0/0 if not settable */

    /* Last failure, for logging. Kept across cam_close(). */
    const char *last_op;
    int last_errno;
};

struct cam_frame {
    const void *data;
    size_t bytesused;
    uint32_t sequence;     /* driver frame counter; gaps mean drops */
    uint32_t flags;        /* V4L2_BUF_FLAG_* */
    struct timeval driver_ts;
    struct timespec mono;  /* CLOCK_MONOTONIC at dequeue */
};

typedef void (*cam_frame_cb)(const struct cam_frame *f, void *ctx);

void cam_init(struct cam_dev *d);

/* Opens, configures YUYV width x height at `fps`, maps buffers and starts
 * streaming. On failure the device is fully closed again. */
enum cam_result cam_open(struct cam_dev *d, const char *path,
                         uint32_t width, uint32_t height, uint32_t fps);

/* Waits up to timeout_ms for one frame and passes it to `cb`. Any device
 * error closes the device and returns CAM_GONE. */
enum cam_result cam_next_frame(struct cam_dev *d, int timeout_ms,
                               cam_frame_cb cb, void *ctx);

/* Stops streaming, unmaps and closes. Safe on a partly opened device. */
void cam_close(struct cam_dev *d);

#endif
