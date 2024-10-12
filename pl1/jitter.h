#ifndef JITTER_H
#define JITTER_H

#include <time.h>

#include "utils.h"

struct jitter_t
{
    bool first;
    struct timespec min;
    struct timespec max;
};

void jitter_init(struct jitter_t *s);
void jitter_add_datapoint(struct jitter_t *s, struct timespec *t);
struct timespec *jitter_get_min(struct jitter_t *s);
struct timespec *jitter_get_max(struct jitter_t *s);
struct timespec jitter_get(struct jitter_t *s);
#endif
