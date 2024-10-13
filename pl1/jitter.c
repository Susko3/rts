#include <math.h>

#include "jitter.h"

// Modified from:
// https://github.com/ppy/osu-framework/blob/fbfb0efbd24ef52eabd6a0373e10f0539dff3131/osu.Framework/Timing/FramedClock.cs#L86-L98

void jitter_init(struct jitter_t *s)
{
    s->first = true;
}

void jitter_add_datapoint(struct jitter_t *s, struct timespec *t)
{
    if (s->first)
    {
        s->first = false;
        s->min = *t;
        s->max = *t;
    }

    if (timespec_less_than(t, &s->min))
    {
        s->min = *t;
    }

    if (timespec_greater_than(t, &s->max))
    {
        s->max = *t;
    }
}

struct timespec *jitter_get_min(struct jitter_t *s)
{
    return &s->min;
}

struct timespec *jitter_get_max(struct jitter_t *s)
{
    return &s->max;
}

struct timespec jitter_get(struct jitter_t *s)
{
    struct timespec diff = {};

    if (s->first)
        return diff;

    timespec_diff(&s->max, &s->min, &diff);
    return diff;
}
