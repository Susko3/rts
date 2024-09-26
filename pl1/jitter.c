#include <math.h>

#include "jitter.h"

// Modified from:
// https://github.com/ppy/osu-framework/blob/fbfb0efbd24ef52eabd6a0373e10f0539dff3131/osu.Framework/Timing/FramedClock.cs#L86-L98

void jitter_init(struct jitter_t *s)
{
    s->num = 0;
    s->sum = 0;
    s->sum_of_squares = 0;
}

void jitter_add_datapoint(struct jitter_t *s, struct timespec *t)
{
    double v = t->tv_sec + t->tv_nsec * 1e-9;
    ++s->num;
    s->sum += v;
    s->sum_of_squares += v * v;
}

double jitter_get(struct jitter_t *s)
{
    if (s->num == 0)
        return 0;

    double avg = s->sum / s->num;
    double variance = (s->sum_of_squares / s->num) - (avg * avg);
    return sqrt(variance);
}
