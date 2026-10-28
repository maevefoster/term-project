#include "applog.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define SCAN_CHUNK 4096
#define CRC_TAG_LEN 10 /* " *XXXXXXXX" */

uint32_t applog_crc32(const void *buf, size_t len)
{
    const unsigned char *p = buf;
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

int applog_line_valid(const char *line, size_t len)
{
    const char *tag;
    uint32_t want = 0;

    if (len <= CRC_TAG_LEN || len > APPLOG_LINE_MAX)
        return 0;
    tag = line + len - CRC_TAG_LEN;
    if (tag[0] != ' ' || tag[1] != '*')
        return 0;
    for (int i = 2; i < CRC_TAG_LEN; i++) {
        char c = tag[i];
        uint32_t v;

        if (c >= '0' && c <= '9')
            v = (uint32_t)(c - '0');
        else if (c >= 'a' && c <= 'f')
            v = (uint32_t)(c - 'a' + 10);
        else
            return 0;
        want = (want << 4) | v;
    }
    return applog_crc32(line, len - CRC_TAG_LEN) == want;
}

/* Sets *out to the offset of the last '\n' in [0, end), or -1 if none. */
static int find_prev_newline(int fd, off_t end, off_t *out)
{
    char buf[SCAN_CHUNK];

    while (end > 0) {
        size_t want = end > (off_t)sizeof buf ? sizeof buf : (size_t)end;
        off_t start = end - (off_t)want;
        ssize_t got = pread(fd, buf, want, start);

        if (got < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if ((size_t)got != want) {
            errno = EIO;
            return -1;
        }
        for (size_t i = want; i > 0; i--) {
            if (buf[i - 1] == '\n') {
                *out = start + (off_t)(i - 1);
                return 0;
            }
        }
        end = start;
    }
    *out = -1;
    return 0;
}

/* Drops an unterminated tail, then the final complete line if its CRC is
 * bad. Sets *keep_out to the resulting file size. */
static int repair_tail(int fd, off_t size, off_t *keep_out)
{
    off_t keep = size;
    off_t nl;
    char last;

    if (size > 0) {
        if (pread(fd, &last, 1, size - 1) != 1)
            return -1;
        if (last != '\n') {
            if (find_prev_newline(fd, size, &nl) < 0)
                return -1;
            keep = nl + 1;
        }
    }
    if (keep > 0) {
        char line[APPLOG_LINE_MAX];
        off_t start;
        size_t len;
        int ok = 0;

        /* keep - 1 is the final '\n'; check the line that ends there. */
        if (find_prev_newline(fd, keep - 1, &nl) < 0)
            return -1;
        start = nl + 1;
        len = (size_t)(keep - 1 - start);
        if (len <= sizeof line) {
            ssize_t got = pread(fd, line, len, start);

            if (got < 0 || (size_t)got != len)
                return -1;
            ok = applog_line_valid(line, len);
        }
        if (!ok)
            keep = start;
    }
    if (keep != size && ftruncate(fd, keep) < 0)
        return -1;
    *keep_out = keep;
    return 0;
}

static int open_file(struct applog *lg, off_t *removed)
{
    struct stat st;
    off_t keep;
    int saved;
    int fd = open(lg->path, O_RDWR | O_CREAT | O_APPEND | O_CLOEXEC, 0640);

    if (fd < 0)
        return -1;
    if (fstat(fd, &st) < 0)
        goto fail;
    if (repair_tail(fd, st.st_size, &keep) < 0)
        goto fail;
    lg->fd = fd;
    lg->size = keep;
    *removed = st.st_size - keep;
    return 0;

fail:
    saved = errno;
    close(fd);
    errno = saved;
    return -1;
}

static void note_failure(struct applog *lg, const char *what)
{
    lg->write_failures++;
    if (lg->write_failures == 1 || lg->write_failures % 100 == 0)
        fprintf(stderr, "applog: %s %s failed: %s (%lu failures)\n",
                lg->path, what, strerror(errno), lg->write_failures);
}

static int reopen(struct applog *lg)
{
    off_t removed;

    if (open_file(lg, &removed) < 0)
        return -1;
    if (removed > 0)
        applog_write(lg, "LOG_TORN_REPAIRED", "bytes=%lld", (long long)removed);
    return 0;
}

/* Starts a fresh file. If the rename fails the current file is emptied
 * instead, so the size bound holds either way. */
static int rotate(struct applog *lg)
{
    off_t old_size = lg->size;
    int renamed = rename(lg->path, lg->rot_path) == 0;

    if (!renamed) {
        note_failure(lg, "rename");
        if (ftruncate(lg->fd, 0) < 0)
            return -1;
        lg->size = 0;
    } else {
        close(lg->fd);
        lg->fd = -1;
        if (reopen(lg) < 0)
            return -1;
    }
    applog_write(lg, "LOG_ROTATED", "prev_bytes=%lld kept=%s",
                 (long long)old_size, renamed ? lg->rot_path : "none");
    return 0;
}

static int format_line(const struct applog *lg, char *buf, size_t cap,
                       const char *event, const char *fmt, va_list ap)
{
    struct timespec mono, real;
    struct tm tm;
    size_t room = cap - CRC_TAG_LEN - 2; /* tag, '\n', NUL */
    size_t len;
    int n;

    (void)clock_gettime(CLOCK_MONOTONIC, &mono);
    (void)clock_gettime(CLOCK_REALTIME, &real);
    if (!gmtime_r(&real.tv_sec, &tm))
        memset(&tm, 0, sizeof tm);

    n = snprintf(buf, room, "%lld.%09ld %04d-%02d-%02dT%02d:%02d:%02d.%03ldZ %s %s",
                 (long long)mono.tv_sec, (long)mono.tv_nsec,
                 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                 tm.tm_hour, tm.tm_min, tm.tm_sec, (long)(real.tv_nsec / 1000000L),
                 lg->proc, event);
    if (n < 0 || (size_t)n >= room)
        return -1;
    len = (size_t)n;

    if (fmt && fmt[0]) {
        int m;

        buf[len++] = ' ';
        m = vsnprintf(buf + len, room - len, fmt, ap);
        if (m < 0)
            return -1;
        if ((size_t)m >= room - len) {
            /* Body too long for one record: drop it and flag that. */
            len--;
            m = snprintf(buf + len, room - len, " truncated=1");
            if (m < 0 || (size_t)m >= room - len)
                return -1;
        }
        len += (size_t)m;
    }

    /* One record per line: a newline inside a value would forge a record. */
    for (size_t i = 0; i < len; i++)
        if (buf[i] == '\n' || buf[i] == '\r')
            buf[i] = '_';

    n = snprintf(buf + len, cap - len, " *%08x\n", (unsigned)applog_crc32(buf, len));
    if (n != CRC_TAG_LEN + 1)
        return -1;
    return (int)(len + (size_t)n);
}

static int write_all(int fd, const char *buf, size_t len)
{
    while (len > 0) {
        ssize_t w = write(fd, buf, len);

        if (w < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        buf += w;
        len -= (size_t)w;
    }
    return 0;
}

int applog_vwrite(struct applog *lg, const char *event, const char *fmt, va_list ap)
{
    char line[APPLOG_LINE_MAX + 1];
    int len = format_line(lg, line, sizeof line, event, fmt, ap);

    if (len < 0) {
        errno = EOVERFLOW;
        note_failure(lg, "format");
        return -1;
    }
    if (lg->fd < 0 && reopen(lg) < 0) {
        note_failure(lg, "open");
        return -1;
    }
    if (lg->size + APPLOG_LINE_MAX > lg->cap && rotate(lg) < 0) {
        note_failure(lg, "rotate");
        if (lg->fd < 0)
            return -1;
    }
    if (write_all(lg->fd, line, (size_t)len) < 0) {
        int saved = errno;

        /* Don't leave a partial record in the middle of the file. */
        if (ftruncate(lg->fd, lg->size) < 0)
            fprintf(stderr, "applog: %s ftruncate after failed write: %s\n",
                    lg->path, strerror(errno));
        errno = saved;
        note_failure(lg, "write");
        return -1;
    }
    lg->size += len;
    return 0;
}

int applog_write(struct applog *lg, const char *event, const char *fmt, ...)
{
    va_list ap;
    int r;

    va_start(ap, fmt);
    r = applog_vwrite(lg, event, fmt, ap);
    va_end(ap);
    return r;
}

int applog_open(struct applog *lg, const char *path, const char *proc, off_t cap)
{
    int n;

    memset(lg, 0, sizeof *lg);
    lg->fd = -1;
    lg->cap = cap < 4 * APPLOG_LINE_MAX ? 4 * APPLOG_LINE_MAX : cap;

    n = snprintf(lg->path, sizeof lg->path, "%s", path);
    if (n < 0 || (size_t)n >= sizeof lg->path) {
        errno = ENAMETOOLONG;
        return -1;
    }
    n = snprintf(lg->rot_path, sizeof lg->rot_path, "%s.1", lg->path);
    if (n < 0 || (size_t)n >= sizeof lg->rot_path) {
        errno = ENAMETOOLONG;
        return -1;
    }
    n = snprintf(lg->proc, sizeof lg->proc, "%s", proc);
    if (n < 0 || (size_t)n >= sizeof lg->proc) {
        errno = EINVAL;
        return -1;
    }
    return reopen(lg);
}

void applog_close(struct applog *lg)
{
    if (lg->fd < 0)
        return;
    if (fdatasync(lg->fd) < 0)
        fprintf(stderr, "applog: %s fdatasync: %s\n", lg->path, strerror(errno));
    if (close(lg->fd) < 0)
        fprintf(stderr, "applog: %s close: %s\n", lg->path, strerror(errno));
    lg->fd = -1;
}
