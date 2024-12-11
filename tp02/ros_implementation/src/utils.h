#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include <sys/time.h>

bool increase_clock_resolution(void);

void reset_clock_resolution(void);

bool pin_this_thread(void);

bool set_realtime_priority(void);

void print(struct timespec *t);

void sleep_until(struct timespec *t);

/**
 * @fn timespec_diff(struct timespec *, struct timespec *, struct timespec *)
 * @brief Compute the diff of two timespecs, that is a - b = result.
 * @param a the minuend
 * @param b the subtrahend
 * @param result a - b
 * @cite https://gist.github.com/diabloneo/9619917?permalink_comment_id=3364033#gistcomment-3364033
 */
static inline void timespec_diff(struct timespec *a, struct timespec *b,
                                 struct timespec *result)
{
    result->tv_sec = a->tv_sec - b->tv_sec;
    result->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (result->tv_nsec < 0)
    {
        --result->tv_sec;
        result->tv_nsec += 1000000000L;
    }
}

static inline void timespec_add(struct timespec *a, struct timespec *b,
                                struct timespec *result)
{
    result->tv_sec = a->tv_sec + b->tv_sec;
    result->tv_nsec = a->tv_nsec + b->tv_nsec;
    if (result->tv_nsec >= 1000000000L)
    {
        ++result->tv_sec;
        result->tv_nsec -= 1000000000L;
    }
}

static inline bool timespec_greater_than(struct timespec *a, struct timespec *b)
{
    if (a->tv_sec > b->tv_sec)
    {
        return true;
    }
    else if (a->tv_sec == b->tv_sec)
    {
        return a->tv_nsec > b->tv_nsec;
    }
    return false;
}

static inline bool timespec_less_than(struct timespec *a, struct timespec *b)
{
    if (a->tv_sec < b->tv_sec)
    {
        return true;
    }
    else if (a->tv_sec == b->tv_sec)
    {
        return a->tv_nsec < b->tv_nsec;
    }
    return false;
}
#endif
