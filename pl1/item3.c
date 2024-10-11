#define _GNU_SOURCE

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "func.h"
#include "jitter.h"
#include "utils.h"

#define NUM_TASKS 3

const bool run_inverse = false;
const bool run_item4a = false;
const bool run_item4b = true;
const bool run_item4b_and_invert = true;
const bool run_item6 = false;

void fail(void)
{
    exit(1);
}

const int CLASS = 1; // PL1
const int GROUP = 6; // Group 6, Paulo & Mateo

struct log_table_entry
{
    size_t task;
    struct timespec desired_start;
    struct timespec activated;
    struct timespec finished;
};

static size_t num_logs[NUM_TASKS];
static struct log_table_entry logs[NUM_TASKS][300];

void add_log(struct log_table_entry *e)
{
    if (e->task == 0 || e->task > NUM_TASKS)
    {
        printf("bad task id: %zu\n", e->task);
        return;
    }

    size_t i = e->task - 1;
    logs[i][num_logs[i]++] = *e;
}

void test_logs(void)
{
    struct log_table_entry e = {
        .task = 1,
        .activated = {
            .tv_sec = 0,
            .tv_nsec = 17,
        },
        .finished = {
            .tv_sec = 0,
            .tv_nsec = 360000,
        }};
    add_log(&e);
    e.task = 2;
    add_log(&e);
    e.activated.tv_sec = 1;
    e.finished.tv_sec = 3;
    add_log(&e);

    // print_log_table();
}

void function_wrapper(struct timespec *desired_start, size_t id)
{
    struct log_table_entry e = {.task = id, .desired_start = *desired_start};

    if (clock_gettime(CLOCK_MONOTONIC, &e.activated))
        fail();

    switch (id)
    {
    case 1:
        f1(CLASS, GROUP);
        break;
    case 2:
        f2(CLASS, GROUP);
        break;
    case 3:
        f3(CLASS, GROUP);
        break;
    default:
        fail();
    }

    if (clock_gettime(CLOCK_MONOTONIC, &e.finished))
        fail();

    add_log(&e);
}

void sleep_until(struct timespec *t)
{
    int ret;
    do
    {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);

        if (ret)
            printf("clock nanosleep failed with %d", ret);

    } while (ret == EINTR);
}

struct thread_info
{
    size_t task;
    struct timespec initial_time;
    struct timespec end_time;
    struct timespec interval;
    int priority;
};

void *thread_entrypoint(void *arg)
{
    // let's store info on our stack :)
    struct thread_info info = *(struct thread_info *)arg;
    struct timespec next_wake = info.initial_time;

    while (timespec_greater_than(&info.end_time, &next_wake))
    {
        sleep_until(&next_wake);

        function_wrapper(&next_wake, info.task);

        timespec_add(&next_wake, &info.interval, &next_wake);
    }

    return NULL;
}

void bootstrap_threads(struct thread_info info[], pthread_t new_threads[], size_t num_threads)
{
    pthread_attr_t attr;
    if (pthread_attr_init(&attr))
        fail();

    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    for (size_t i = 0; i < num_threads; i++)
    {
        struct sched_param sp = {.sched_priority = info[i].priority};
        pthread_attr_setschedparam(&attr, &sp);

        if (pthread_create(&new_threads[i], &attr, thread_entrypoint, (void *)&info[i]))
            fail();
    }

    if (pthread_attr_destroy(&attr))
        fail();
}

void print_log_table(struct thread_info info[], size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        printf("Logs for task %zu\n", info[i].task);
        struct timespec max = {};
        struct timespec min = {.tv_sec=30};
        struct jitter_t jitter;
        jitter_init(&jitter);

        for (size_t j = 0; j < num_logs[i]; j++)
        {
            struct log_table_entry *tr = &logs[i][j];

            struct timespec response_time;
            timespec_diff(&tr->finished, &tr->desired_start, &response_time);

            if (timespec_greater_than(&response_time, &max))
            {
                max = response_time;
                jitter_new_max(&jitter,&response_time);
            }
            if (timespec_less_than(&response_time, &min))
            {
                min = response_time;
                jitter_new_min(&jitter,&response_time);
            }

            jitter_add_datapoint(&jitter, &response_time);

            tr->desired_start.tv_sec -= info[i].initial_time.tv_sec;
            tr->activated.tv_sec -= info[i].initial_time.tv_sec;
            tr->finished.tv_sec -= info[i].initial_time.tv_sec;

            printf("Desired:   ");
            print(&tr->desired_start);
            printf("\n");
            printf("Activated: ");
            print(&tr->activated);
            printf("\n");
            printf("Finished:  ");
            print(&tr->finished);
            if (timespec_greater_than(&response_time, &info[i].interval))
                printf(" *** Over deadline");
            printf("\n");
        }
        printf("\n");

        printf("Max response time:\t");
        print(&max);
        printf("\n");
        printf("Min response time:\t");
        print(&min);
        printf("\n");

        printf("Jitter:\t\t\t%.9f s\n", jitter_get(&jitter));

        printf("\n");
    }
}

void item4a(struct timespec *inital_time, pthread_t threads[])
{
    struct timespec time, one95 = {.tv_sec = 1, .tv_nsec = 950000000L}, three95 = {.tv_sec = 3, .tv_nsec = 950000000L};
    timespec_add(inital_time, &one95, &time);
    sleep_until(&time);

    if (pthread_setschedprio(threads[0], 1))
        fail();
    if (pthread_setschedprio(threads[1], 2))
        fail();
    if (pthread_setschedprio(threads[2], 3))
        fail();

    timespec_add(inital_time, &three95, &time);
    sleep_until(&time);

    if (pthread_setschedprio(threads[0], 3))
        fail();
    if (pthread_setschedprio(threads[1], 2))
        fail();
    if (pthread_setschedprio(threads[2], 1))
        fail();
}

int main(void)
{
    if (run_item4a)
    {
        printf("Item 4, Alternative A\n");
    }
    else if (run_item4b)
    {
        printf("Item 4, Alternative B%s\n", run_item4b_and_invert ? " (invert)" : "");
    }
    else if (run_item6)
    {
        printf("Item 6\n");
    }
    else
    {
        printf("Item 3\n");
    }

    if (!set_realtime_priority())
        fail();

    if (!run_item4b)
    {
        if (!pin_this_thread())
            fail();
    }

    if (!increase_clock_resolution())
        fail();

    struct timespec initial_time, end_time;

    clock_gettime(CLOCK_MONOTONIC, &initial_time);
    ++initial_time.tv_sec;
    initial_time.tv_nsec = 0;
    end_time.tv_sec = initial_time.tv_sec + 6;

    struct thread_info info[NUM_TASKS] =
        {
            {.task = 1, .initial_time = initial_time, .end_time = end_time, .interval = {.tv_sec = 0, .tv_nsec = 100000000}, .priority = 3},
            {.task = 2, .initial_time = initial_time, .end_time = end_time, .interval = {.tv_sec = 0, .tv_nsec = 200000000}, .priority = 2},
            {.task = 3, .initial_time = initial_time, .end_time = end_time, .interval = {.tv_sec = 0, .tv_nsec = 300000000}, .priority = 1},
        };

    if (run_inverse)
    {
        info[0].priority = 1;
        info[1].priority = 2;
        info[2].priority = 3;
    }
    if (run_item4b_and_invert)
    {
        info[0].priority = 1;
        info[1].priority = 2;
        info[2].priority = 3;
    }

    if (run_item6)
    {
        info[0].priority = 1;
        info[1].priority = 1;
        info[2].priority = 1;
    }

    pthread_t threads[NUM_TASKS] = {};

    bootstrap_threads(info, threads, NUM_TASKS);

    if (run_item4a)
        item4a(&initial_time, threads);

    for (size_t i = 0; i < NUM_TASKS; i++)
    {
        pthread_join(threads[i], NULL);
    }

    print_log_table(info, NUM_TASKS);

    reset_clock_resolution();
    return 0;
}
