#ifndef POSTURE_TIMEUTIL_H
#define POSTURE_TIMEUTIL_H

#include <stdint.h>
#include <time.h>

#define NS_PER_S  1000000000LL
#define NS_PER_MS 1000000LL

static inline int64_t ts_ns(const struct timespec *t)
{
    return (int64_t)t->tv_sec * NS_PER_S + t->tv_nsec;
}

static inline int64_t mono_ns(void)
{
    struct timespec t;
    (void)clock_gettime(CLOCK_MONOTONIC, &t);
    return ts_ns(&t);
}

#endif
