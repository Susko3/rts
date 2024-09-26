#define _GNU_SOURCE

#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>

#include "utils.h"

bool increase_clock_resolution(void)
{
    struct timespec res;
    if(clock_getres(CLOCK_MONOTONIC, &res))
    {
        printf("Failed to get clock resolution.\n");
    }
    else
    {
        printf("Monotonic clock resolution: ");
        print(&res);
        printf("\n");
    }

    return true;
}

void reset_clock_resolution(void)
{
}

// TODO: pin this process, as we'll be creating threads.
bool pin_this_thread(void)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset))
    {
        printf("Failed to set pthread affinity\n");
        return false;
    }
    return true;
}

bool set_realtime_priority(void)
{
    if (setpriority(PRIO_PROCESS, 0, -20))
    {
        printf("Failed to set priority\n");
        return false;
    }
    return true;
}

void print(struct timespec *t)
{
    if (t->tv_sec >= 0)
        printf("%ld.%09ld s", t->tv_sec, t->tv_nsec);
    else if(t->tv_nsec != 0)
        printf("-%ld.%09ld s", -t->tv_sec - 1, 1000000000L - t->tv_nsec);
    else
        printf("-%ld.%09ld s", -t->tv_sec, 0L);
}
