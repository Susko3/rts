#include <cassert>
#include <csignal>
#include <initializer_list>
#include <iostream>
#include <optional>

#include <pthread.h>
#include <unistd.h>

#include "preprocess.h"
#include "utils.h"

struct thread_safe_lidar_data
{
    lidar_data *data;
    pthread_mutex_t *mutex;
    pthread_cond_t *data_is_null;
    pthread_cond_t *data_available;
};

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
    thread_safe_lidar_data loaded;
    thread_safe_lidar_data_storage preprocessed;
    // thread_safe_lidar_data_storage identified;
};

static struct state state_unsafe;

static volatile sig_atomic_t running = 0;

void signal_handler(int sig, siginfo_t *info, void *ucontext)
{
    if (sig == SIGINT)
    {
        running = 0;

        pthread_mutex_lock(state_unsafe.loaded.mutex);
        pthread_cond_signal(state_unsafe.loaded.data_available);
        pthread_cond_signal(state_unsafe.loaded.data_is_null);
        pthread_mutex_unlock(state_unsafe.loaded.mutex);

        pthread_mutex_lock(state_unsafe.preprocessed.mutex);
        pthread_cond_signal(state_unsafe.preprocessed.data_available);
        pthread_cond_signal(state_unsafe.preprocessed.data_is_null);
        pthread_mutex_unlock(state_unsafe.preprocessed.mutex);
    }
}

void *load_data_thread(void *arg)
{
    struct timespec interval
    {
        0, 100000000
    };

    auto state = static_cast<struct state *>(arg);

    std::initializer_list<std::string> files = {
        "point_cloud1.txt",
        "point_cloud2.txt",
        "point_cloud3.txt",
    };

    int next_file = 0;
    struct timespec next_wake = state->initial_time;

    while (running)
    {
        // TODO: fault detection if our cycle is over 10 Hz
        sleep_until(&next_wake);

        lidar_data *inflight = load_data(files.begin()[next_file]);

        pthread_mutex_lock(state->loaded.mutex);

        while (state->loaded.data != nullptr)
        {
            pthread_cond_wait(state->loaded.data_is_null, state->loaded.mutex);

            if (!running)
            {
                pthread_mutex_unlock(state->loaded.mutex);
                return nullptr;
            }
        }

        state->loaded.data = inflight;
        pthread_cond_signal(state->loaded.data_available);

        pthread_mutex_unlock(state->loaded.mutex);

        timespec_add(&next_wake, &interval, &next_wake);
        next_file = (next_file + 1) % files.size();
    }

    return nullptr;
}

void *preprocess_discard_thread(void *arg)
{
    auto state = static_cast<struct state *>(arg);

    while (running)
    {
        pthread_mutex_lock(state->loaded.mutex);

        while (state->loaded.data == nullptr)
        {
            pthread_cond_wait(state->loaded.data_available, state->loaded.mutex);

            if (!running)
            {
                pthread_mutex_unlock(state->loaded.mutex);
                return nullptr;
            }
        }

        lidar_data *inflight = state->loaded.data;
        state->loaded.data = nullptr;
        pthread_cond_signal(state->loaded.data_is_null);

        pthread_mutex_unlock(state->loaded.mutex);

        lidar_data outbound{};
        float forward = 30;
        float side = 15;
        float top = 2;
        preprocess_discard(inflight, &outbound,forward,side,top);
        delete inflight;

        pthread_mutex_lock(state->preprocessed.mutex);

        while (state->preprocessed.has_data)
        {
            pthread_cond_wait(state->preprocessed.data_is_null, state->preprocessed.mutex);

            if (!running)
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

    float forward = 30;
    float side = 15;
    float maxDiff = 2;
    float maxIncline = 0.10;

    while (running)
    {
        pthread_mutex_lock(state->preprocessed.mutex);

        while (!state->preprocessed.has_data)
        {
            pthread_cond_wait(state->preprocessed.data_available, state->preprocessed.mutex);

            if (!running)
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
        identify_driveable(&inflight, &output, forward,side, maxDiff, maxIncline);

        std::cout << "Final data size: " << output.points.size() << std::endl;

        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);

        std::cout << "Clock time: " << std::flush;
        print(&t);
        std::printf("\n\n");
    }

    return nullptr;
}

void setup_signal_handler()
{
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = signal_handler;
    assert(sigaction(SIGINT, &sa, nullptr) == 0);
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

int main()
{
    assert(set_realtime_priority());
    assert(pin_this_thread());
    assert(increase_clock_resolution());

    running = 1;
    setup_mutex_cond(&state_unsafe);

    setup_signal_handler();

    clock_gettime(CLOCK_MONOTONIC, &state_unsafe.initial_time);
    state_unsafe.initial_time.tv_sec++;
    state_unsafe.initial_time.tv_nsec = 0;

    pthread_t load, prep, id;

    assert(!pthread_create(&load, nullptr, load_data_thread, &state_unsafe));
    assert(!pthread_create(&prep, nullptr, preprocess_discard_thread, &state_unsafe));
    assert(!pthread_create(&id, nullptr, identify_driveable_thread, &state_unsafe));

    pthread_join(load, nullptr);
    pthread_join(prep, nullptr);
    pthread_join(id, nullptr);

    std::cout << "Main thread is finished." << std::endl;

    reset_clock_resolution();

    // There is no need to destroy the mutexes, conditional variables or free the memory:
    // the operating system will do it for us :)
}
