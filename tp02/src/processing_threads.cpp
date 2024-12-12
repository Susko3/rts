#include "processing_threads.h"

#include "utils.h"

void *load_data_thread(void *arg)
{
    struct timespec interval
    {
        0, 100000000
    };

    auto state = static_cast<struct state *>(arg);

    struct timespec next_wake = state->initial_time;

    while (state->running)
    {
        // TODO: fault detection if our cycle is over 10 Hz
        sleep_until(&next_wake);

        lidar_data inflight{};
        state->load_data_blocking(&inflight);

        pthread_mutex_lock(state->loaded.mutex);

        while (state->loaded.has_data)
        {
            pthread_cond_wait(state->loaded.data_is_null, state->loaded.mutex);

            if (!state->running)
            {
                pthread_mutex_unlock(state->loaded.mutex);
                return nullptr;
            }
        }

        state->loaded.data = inflight;
        pthread_cond_signal(state->loaded.data_available);

        pthread_mutex_unlock(state->loaded.mutex);

        timespec_add(&next_wake, &interval, &next_wake);
    }

    return nullptr;
}

void *preprocess_discard_thread(void *arg)
{
    auto state = static_cast<struct state *>(arg);

    while (state->running)
    {
        pthread_mutex_lock(state->loaded.mutex);

        while (!state->loaded.has_data)
        {
            pthread_cond_wait(state->loaded.data_available, state->loaded.mutex);

            if (!state->running)
            {
                pthread_mutex_unlock(state->loaded.mutex);
                return nullptr;
            }
        }

        lidar_data inflight = state->loaded.data;
        state->loaded.has_data = false;
        pthread_cond_signal(state->loaded.data_is_null);

        pthread_mutex_unlock(state->loaded.mutex);

        lidar_data outbound{};
        preprocess_discard(&inflight, &outbound);

        pthread_mutex_lock(state->preprocessed.mutex);

        while (state->preprocessed.has_data)
        {
            pthread_cond_wait(state->preprocessed.data_is_null, state->preprocessed.mutex);

            if (!state->running)
            {
                pthread_mutex_unlock(state->preprocessed.mutex);
                return nullptr;
            }
        }

        state->preprocessed.data = outbound;
        state->preprocessed.has_data = true;
        pthread_cond_signal(state->preprocessed.data_available);

        pthread_mutex_unlock(state->preprocessed.mutex);
    }

    return nullptr;
}

void *identify_driveable_thread(void *arg)
{
    auto state = static_cast<struct state *>(arg);

    while (state->running)
    {
        pthread_mutex_lock(state->preprocessed.mutex);

        while (!state->preprocessed.has_data)
        {
            pthread_cond_wait(state->preprocessed.data_available, state->preprocessed.mutex);

            if (!state->running)
            {
                pthread_mutex_unlock(state->preprocessed.mutex);
                return nullptr;
            }
        }

        lidar_data inflight = state->preprocessed.data;
        state->preprocessed.has_data = false;
        pthread_cond_signal(state->preprocessed.data_is_null);

        pthread_mutex_unlock(state->preprocessed.mutex);

        lidar_data output;
        identify_driveable(&inflight, &output);

        state->publish_data(&output);
    }

    return nullptr;
}

void setup_mutex_cond(struct state *state)
{
    state->loaded.mutex = new pthread_mutex_t;
    state->loaded.data_is_null = new pthread_cond_t;
    state->loaded.data_available = new pthread_cond_t;
    state->preprocessed.mutex = new pthread_mutex_t;
    state->preprocessed.data_is_null = new pthread_cond_t;
    state->preprocessed.data_available = new pthread_cond_t;
    pthread_mutex_init(state->loaded.mutex, nullptr);
    pthread_cond_init(state->loaded.data_is_null, nullptr);
    pthread_cond_init(state->loaded.data_available, nullptr);
    pthread_mutex_init(state->preprocessed.mutex, nullptr);
    pthread_cond_init(state->preprocessed.data_is_null, nullptr);
    pthread_cond_init(state->preprocessed.data_available, nullptr);
}
