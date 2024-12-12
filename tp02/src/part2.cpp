#include <cassert>
#include <csignal>
#include <initializer_list>
#include <iostream>
#include <optional>

#include <pthread.h>
#include <unistd.h>

#include "preprocess.h"
#include "utils.h"
#include "processing_threads.h"

static struct state state_unsafe;

void signal_handler(int sig, siginfo_t *info, void *ucontext)
{
    if (sig == SIGINT)
    {
        state_unsafe.running = 0;

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

void load_data_from_files(lidar_data &data)
{
    std::initializer_list<std::string> files = {
        "point_cloud1.txt",
        "point_cloud2.txt",
        "point_cloud3.txt",
    };

    static int next_file = 0;

    load_data(files.begin()[next_file], data);

    next_file = (next_file + 1) % files.size();
}

void setup_signal_handler()
{
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = signal_handler;
    assert(sigaction(SIGINT, &sa, nullptr) == 0);
}

void print_data(const lidar_data &data)
{
    std::cout << "Final data size: " << data.points.size() << std::endl;

    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);

    std::cout << "Clock time: " << std::flush;
    print(&t);
    std::printf("\n\n");
}

int main()
{
    assert(set_realtime_priority());
    assert(pin_this_thread());
    assert(increase_clock_resolution());

    state_unsafe.load_data_blocking = load_data_from_files;
    state_unsafe.publish_data = print_data;

    state_unsafe.running = 1;
    setup_mutex_cond(state_unsafe);

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
