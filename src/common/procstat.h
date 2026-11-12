#ifndef POSTURE_PROCSTAT_H
#define POSTURE_PROCSTAT_H

#include <stdio.h>
#include <unistd.h>

/* Resident set size of this process in KiB, or -1 if unreadable. */
static inline long proc_rss_kb(void)
{
    long pages = -1;
    long page = sysconf(_SC_PAGESIZE);
    FILE *f = fopen("/proc/self/statm", "re");

    if (!f)
        return -1;
    if (fscanf(f, "%*s %ld", &pages) != 1)
        pages = -1;
    fclose(f);
    return (pages < 0 || page <= 0) ? -1 : pages * (page / 1024);
}

#endif
