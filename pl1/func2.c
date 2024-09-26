#define _GNU_SOURCE

#include <stdlib.h>
#include <time.h>

#include "func.h"
#include "utils.h"

void sleep_for(struct timespec *delta)
{
    struct timespec now, end;

    if (clock_gettime(CLOCK_MONOTONIC, &now))
        exit(1);

    timespec_add(&now, delta, &end);

    while (timespec_greater_than(&end, &now))
    {
        if (clock_gettime(CLOCK_MONOTONIC, &now))
            exit(1);
    }
}

void f1(int i, int j)
{
    struct timespec thirty = {.tv_sec = 0, .tv_nsec = 30000000};
    sleep_for(&thirty);
}

void f2(int i, int j)
{
    struct timespec fifty = {.tv_sec = 0, .tv_nsec = 50000000};
    sleep_for(&fifty);
}

void f3(int i, int j)
{
    struct timespec eighty = {.tv_sec = 0, .tv_nsec = 80000000};
    sleep_for(&eighty);
}
