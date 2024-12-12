#ifndef PROCESSING_THREADS_H
#define PROCESSING_THREADS_H

#include <csignal>
#include <functional>

#include <pthread.h>
#include <unistd.h>

#include "preprocess.h"

struct thread_safe_lidar_data_storage
{
    lidar_data data;
    bool has_data;
    pthread_mutex_t *mutex;
    pthread_cond_t *data_is_null;
    pthread_cond_t *data_available;
};

struct state
{
    struct timespec initial_time;
    thread_safe_lidar_data_storage loaded;
    thread_safe_lidar_data_storage preprocessed;
    volatile sig_atomic_t running = 0;

    std::function<void(lidar_data *)> load_data_blocking;
    std::function<void(lidar_data *)> publish_data;
};

#endif
