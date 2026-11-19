#include "sup_policy.h"

#define SUP_FAILURES_CAP 32 /* past this the backoff is long since capped */

unsigned sup_next_failures(unsigned failures, int64_t ran_ns)
{
    if (ran_ns >= SUP_STABLE_NS)
        return 1;
    return failures < SUP_FAILURES_CAP ? failures + 1 : SUP_FAILURES_CAP;
}

int64_t sup_backoff_ns(unsigned failures)
{
    int64_t d = SUP_BACKOFF_FIRST_NS;

    for (unsigned i = 1; i < failures && d < SUP_BACKOFF_MAX_NS; i++)
        d *= 2;
    return d < SUP_BACKOFF_MAX_NS ? d : SUP_BACKOFF_MAX_NS;
}

int64_t sup_watchdog_deadline(int64_t started_ns, int got_hb, int64_t last_hb_ns)
{
    return got_hb ? last_hb_ns + SUP_HB_SILENCE_NS : started_ns + SUP_FIRST_HB_NS;
}
