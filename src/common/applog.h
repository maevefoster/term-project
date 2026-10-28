/* Append-only application log with per-record CRC framing.
 *
 * Record format, one line each:
 *   <mono_s.ns> <UTC ISO-8601> <proc> <EVENT> key=value ... *<crc32 hex>\n
 * The CRC-32 (IEEE) covers every byte before " *". On open, a final line
 * that lacks '\n' or fails its CRC is truncated away (torn write after a
 * crash or power loss) and a LOG_TORN_REPAIRED record is appended.
 *
 * Size is bounded: before a write that could push the file past `cap`, the
 * file is renamed to "<path>.1" (replacing any older copy) and a new file
 * is started, so disk use stays under about 2 * cap.
 *
 * M3 interim: cam_proc owns this log directly. It moves into hub once hub
 * exists (CLAUDE.md section 1, "Only hub persists application logs").
 */
#ifndef POSTURE_APPLOG_H
#define POSTURE_APPLOG_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/* Placeholder cap until the team decides retention (CLAUDE.md section 8). */
#ifndef APPLOG_CAP
#define APPLOG_CAP (5 * 1024 * 1024)
#endif

#define APPLOG_LINE_MAX 512 /* bytes per record, including '\n' */
#define APPLOG_PATH_MAX 256

struct applog {
    int fd;
    off_t size;
    off_t cap;
    unsigned long write_failures;
    char path[APPLOG_PATH_MAX];
    char rot_path[APPLOG_PATH_MAX + 2];
    char proc[32];
};

/* Opens (creating if needed) and repairs a torn tail. Returns 0, or -1 with
 * errno set. `cap` below 4 * APPLOG_LINE_MAX is raised to that minimum. */
int applog_open(struct applog *lg, const char *path, const char *proc, off_t cap);

/* Appends one record. `fmt` is the key=value body (may be NULL or "").
 * Returns 0, or -1 on failure (counted in write_failures, reported on
 * stderr); callers keep running either way. */
int applog_write(struct applog *lg, const char *event, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
int applog_vwrite(struct applog *lg, const char *event, const char *fmt, va_list ap)
    __attribute__((format(printf, 3, 0)));

void applog_close(struct applog *lg);

uint32_t applog_crc32(const void *buf, size_t len);

/* True if `line` (length `len`, without its '\n') carries a valid CRC tag. */
int applog_line_valid(const char *line, size_t len);

#endif
