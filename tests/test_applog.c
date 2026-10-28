/* Host unit tests for src/common/applog.c (no hardware). */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "applog.h"

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
static char g_buf[1 << 16];

static void path_in_dir(char *out, size_t cap, const char *name)
{
    snprintf(out, cap, "%s/%s", g_dir, name);
}

static size_t read_file(const char *p)
{
    FILE *f = fopen(p, "rb");
    size_t n;

    if (!f)
        return 0;
    n = fread(g_buf, 1, sizeof g_buf - 1, f);
    fclose(f);
    g_buf[n] = '\0';
    return n;
}

static void write_file(const char *p, const char *data, size_t len)
{
    FILE *f = fopen(p, "wb");

    CHECK(f != NULL);
    if (!f)
        return;
    CHECK(fwrite(data, 1, len, f) == len);
    CHECK(fclose(f) == 0);
}

/* Number of records if every line is '\n'-terminated with a valid CRC,
 * otherwise -1. Leaves the file contents in g_buf. */
static int count_valid_lines(const char *p)
{
    size_t n = read_file(p);
    size_t start = 0;
    int lines = 0;

    if (n == 0)
        return 0;
    if (g_buf[n - 1] != '\n')
        return -1;
    for (size_t i = 0; i < n; i++) {
        if (g_buf[i] != '\n')
            continue;
        if (!applog_line_valid(g_buf + start, i - start))
            return -1;
        lines++;
        start = i + 1;
    }
    return lines;
}

static off_t file_size(const char *p)
{
    struct stat st;

    return stat(p, &st) == 0 ? st.st_size : -1;
}

static void test_crc(void)
{
    CHECK(applog_crc32("123456789", 9) == 0xCBF43926u);
    CHECK(applog_crc32("", 0) == 0);
}

static void test_basic(void)
{
    struct applog lg;
    char p[128];

    path_in_dir(p, sizeof p, "basic.log");
    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    CHECK(applog_write(&lg, "EV_ONE", "a=%d", 1) == 0);
    CHECK(applog_write(&lg, "EV_TWO", NULL) == 0);
    CHECK(applog_write(&lg, "EV_THREE", "s=%s", "x") == 0);
    applog_close(&lg);

    CHECK(count_valid_lines(p) == 3);
    CHECK(strstr(g_buf, " cam_test EV_ONE a=1 *") != NULL);
    CHECK(strstr(g_buf, " cam_test EV_TWO *") != NULL);

    /* Reopening a clean file must not report a repair. */
    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    applog_close(&lg);
    CHECK(count_valid_lines(p) == 3);
    CHECK(strstr(g_buf, "LOG_TORN_REPAIRED") == NULL);
}

static void test_line_valid(void)
{
    CHECK(!applog_line_valid("abc", 3));
    CHECK(!applog_line_valid("payload *0000000", 16));
    CHECK(!applog_line_valid("payload *zzzzzzzz", 17));
    CHECK(!applog_line_valid("payload *00000000", 17));
}

static void test_torn_partial(void)
{
    static const char partial[] = "1234.5 partial rec";
    struct applog lg;
    char p[128], want[64];
    int fd;

    path_in_dir(p, sizeof p, "torn.log");
    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    CHECK(applog_write(&lg, "EV", "n=1") == 0);
    CHECK(applog_write(&lg, "EV", "n=2") == 0);
    applog_close(&lg);

    fd = open(p, O_WRONLY | O_APPEND);
    CHECK(fd >= 0);
    CHECK(write(fd, partial, strlen(partial)) == (ssize_t)strlen(partial));
    close(fd);

    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    applog_close(&lg);
    CHECK(count_valid_lines(p) == 3);
    snprintf(want, sizeof want, "LOG_TORN_REPAIRED bytes=%zu ", strlen(partial));
    CHECK(strstr(g_buf, want) != NULL);
    CHECK(strstr(g_buf, "partial rec") == NULL);
}

static void test_bad_crc_final_line(void)
{
    struct applog lg;
    char p[128], want[64];
    size_t n, last_start = 0;

    path_in_dir(p, sizeof p, "badcrc.log");
    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    CHECK(applog_write(&lg, "EV", "n=1") == 0);
    CHECK(applog_write(&lg, "EV", "n=2") == 0);
    applog_close(&lg);

    /* Flip one payload byte in the final line. */
    n = read_file(p);
    CHECK(n > 2);
    for (size_t i = 0; i + 1 < n; i++)
        if (g_buf[i] == '\n')
            last_start = i + 1;
    g_buf[last_start] = g_buf[last_start] == '9' ? '8' : '9';
    write_file(p, g_buf, n);

    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    applog_close(&lg);
    CHECK(count_valid_lines(p) == 2);
    CHECK(strstr(g_buf, "EV n=1 ") != NULL);
    CHECK(strstr(g_buf, "EV n=2 ") == NULL);
    snprintf(want, sizeof want, "LOG_TORN_REPAIRED bytes=%zu ", n - last_start);
    CHECK(strstr(g_buf, want) != NULL);
}

static void test_garbage_only(void)
{
    struct applog lg;
    char p[128];

    path_in_dir(p, sizeof p, "garbage.log");
    write_file(p, "xyz", 3);
    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    applog_close(&lg);
    CHECK(count_valid_lines(p) == 1);
    CHECK(strstr(g_buf, "LOG_TORN_REPAIRED bytes=3 ") != NULL);
}

static void test_newline_injection(void)
{
    struct applog lg;
    char p[128];

    path_in_dir(p, sizeof p, "inject.log");
    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    CHECK(applog_write(&lg, "EV", "path=%s", "a\nb\rc") == 0);
    applog_close(&lg);
    CHECK(count_valid_lines(p) == 1);
    CHECK(strstr(g_buf, "path=a_b_c ") != NULL);
}

static void test_long_body(void)
{
    struct applog lg;
    char p[128], big[1000];

    memset(big, 'z', sizeof big - 1);
    big[sizeof big - 1] = '\0';
    path_in_dir(p, sizeof p, "long.log");
    CHECK(applog_open(&lg, p, "cam_test", APPLOG_CAP) == 0);
    CHECK(applog_write(&lg, "EV", "v=%s", big) == 0);
    applog_close(&lg);
    CHECK(count_valid_lines(p) == 1);
    CHECK(strstr(g_buf, "EV truncated=1 *") != NULL);
    CHECK(file_size(p) <= APPLOG_LINE_MAX);
}

static void test_rotation(void)
{
    const off_t cap = 4096;
    struct applog lg;
    char p[128], rot[132];

    path_in_dir(p, sizeof p, "rot.log");
    snprintf(rot, sizeof rot, "%s.1", p);
    CHECK(applog_open(&lg, p, "cam_test", cap) == 0);
    for (int i = 0; i < 300; i++)
        CHECK(applog_write(&lg, "EV", "i=%d pad=0123456789", i) == 0);
    applog_close(&lg);

    CHECK(access(rot, F_OK) == 0);
    CHECK(file_size(p) <= cap);
    CHECK(file_size(rot) <= cap);
    CHECK(count_valid_lines(rot) > 0);
    CHECK(count_valid_lines(p) > 0);
    CHECK(strstr(g_buf, "LOG_ROTATED") != NULL);
    CHECK(strstr(g_buf, "EV i=299 ") != NULL);
    CHECK(lg.write_failures == 0);
}

static void cleanup_dir(void)
{
    static const char *names[] = {
        "basic.log", "torn.log", "badcrc.log", "garbage.log",
        "inject.log", "long.log", "rot.log", "rot.log.1",
    };
    char p[128];

    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++) {
        path_in_dir(p, sizeof p, names[i]);
        if (unlink(p) < 0 && errno != ENOENT)
            perror(p);
    }
    if (rmdir(g_dir) < 0)
        perror(g_dir);
}

int main(void)
{
    snprintf(g_dir, sizeof g_dir, "/tmp/applog_test.XXXXXX");
    if (!mkdtemp(g_dir)) {
        perror("mkdtemp");
        return 1;
    }

    test_crc();
    test_line_valid();
    test_basic();
    test_torn_partial();
    test_bad_crc_final_line();
    test_garbage_only();
    test_newline_injection();
    test_long_body();
    test_rotation();

    cleanup_dir();
    if (failures) {
        fprintf(stderr, "test_applog: %d check(s) failed\n", failures);
        return 1;
    }
    printf("test_applog: all checks passed\n");
    return 0;
}
