#define _GNU_SOURCE

#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "func.h"
#include "utils.h"

const int CLASS = 1; // PL1
const int GROUP = 6; // Group 6, Paulo & Mateo

typedef void (*FUNC)(int, int);

void fail(void)
{
    exit(1);
}

struct timespec measure(FUNC f)
{
    clockid_t clock = CLOCK_MONOTONIC;

    struct timespec start, end, result;
    if (clock_gettime(clock, &start))
        fail();

    f(CLASS, GROUP);

    if (clock_gettime(clock, &end))
        fail();

    timespec_diff(&end, &start, &result);
    printf("Time: ");
    print(&result);
    printf("\n");
    return result;
}

int main(void)
{
    if (!set_realtime_priority())
        fail();

    if (!pin_this_thread())
        fail();

    if (!increase_clock_resolution())
        fail();

    measure(f1);
    measure(f2);
    measure(f3);

    reset_clock_resolution();

    return 0;
}
