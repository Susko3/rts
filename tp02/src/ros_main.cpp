#include <cassert>
#include <csignal>
#include <initializer_list>
#include <iostream>
#include <optional>

#include <pthread.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "ros_preprocess.h"
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

struct thread_safe_lidar_cloud
{
    bool has_cloud;
    bool processed;
    pthread_mutex_t *mutex;
    pthread_cond_t *cloud_available;
    pthread_cond_t *cloud_processed;
};

struct state
{
    struct timespec initial_time;
    thread_safe_lidar_data loaded;
    thread_safe_lidar_data_storage preprocessed;
    thread_safe_lidar_cloud cloud;
};

static struct state state_unsafe;

static volatile sig_atomic_t running = 0;

ros::Publisher newPointCloud;

bool runflag = false;

sensor_msgs::PointCloud2::ConstPtr pointcloud;

sensor_msgs::PointCloud::ConstPtr pointcloudolder;

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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void *load_data_thread(void *arg)
{
    struct timespec interval
    {
        0, 100000000
    };

    auto state = static_cast<struct state *>(arg);

    struct timespec next_wake = state->initial_time;

    while (running)
    {
        // TODO: fault detection if our cycle is over 10 Hz
        sleep_until(&next_wake);

        pthread_mutex_lock(state->cloud.mutex);
        while (!state->cloud.has_cloud)
        {
            pthread_cond_wait(state->cloud.cloud_available, state->cloud.mutex);

            if (!running)
            {
                pthread_mutex_unlock(state->cloud.mutex);
                return nullptr;
            }
        }
        sensor_msgs::PointCloud output;
        sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud, output);

        lidar_data *inflight = load_data(output);

        state->cloud.has_cloud = false;

        pthread_mutex_unlock(state->cloud.mutex);

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
    }

    return nullptr;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        preprocess_discard(inflight, &outbound, forward, side, top);
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void *identify_driveable_thread(void *arg)
{
    auto state = static_cast<struct state *>(arg);

    float forward = 30;
    float side = 15;
    float maxDiff = 1;
    float maxIncline = 0.01;

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
        identify_driveable(&inflight, &output, forward, side, maxDiff, maxIncline);

        sensor_msgs::PointCloud final_output;
        final_output.header = pointcloud->header; // Use the header from /velodyne_points

        for (const auto &point : output.points)
        {
            geometry_msgs::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            final_output.points.push_back(p);
        }

        newPointCloud.publish(final_output);
        state->cloud.processed = true;
        pthread_cond_signal(state->cloud.cloud_processed);
    }

    return nullptr;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup_signal_handler()
{
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = signal_handler;
    sigaction(SIGINT, &sa, nullptr) == 0;
}

void setup_mutex_cond(struct state *state)
{
    state->loaded.mutex = new pthread_mutex_t;
    state->loaded.data_is_null = new pthread_cond_t;
    state->loaded.data_available = new pthread_cond_t;
    state->preprocessed.mutex = new pthread_mutex_t;
    state->preprocessed.data_is_null = new pthread_cond_t;
    state->preprocessed.data_available = new pthread_cond_t;
    state->cloud.mutex = new pthread_mutex_t;
    state->cloud.cloud_available = new pthread_cond_t;
    state->cloud.cloud_processed = new pthread_cond_t;
    pthread_mutex_init(state->loaded.mutex, nullptr);
    pthread_cond_init(state->loaded.data_is_null, nullptr);
    pthread_cond_init(state->loaded.data_available, nullptr);
    pthread_mutex_init(state->preprocessed.mutex, nullptr);
    pthread_cond_init(state->preprocessed.data_is_null, nullptr);
    pthread_cond_init(state->preprocessed.data_available, nullptr);
    pthread_mutex_init(state->cloud.mutex, nullptr);
    pthread_cond_init(state->cloud.cloud_available, nullptr);
    pthread_cond_init(state->cloud.cloud_processed, nullptr);
}

//

void handlePointCloud(sensor_msgs::PointCloud2::ConstPtr scan_out)
{
    pointcloud = scan_out;
    runflag = true;
}

int main(int argc, char **argv)
{
    std::cout << "tvoja mama" << std::endl;
    
    ros::init(argc, argv, "strdemo");
    ros::NodeHandle nh("~");
    newPointCloud = nh.advertise<sensor_msgs::PointCloud>("/output_results", 100);
    ros::Subscriber PointCloudHandlervelodyne =
        nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, handlePointCloud);
    ros::Rate rate(100.0);

    set_realtime_priority();
    pin_this_thread();
    increase_clock_resolution();

    running = 1;
    setup_mutex_cond(&state_unsafe);

    setup_signal_handler();

    clock_gettime(CLOCK_MONOTONIC, &state_unsafe.initial_time);
    state_unsafe.initial_time.tv_sec++;
    state_unsafe.initial_time.tv_nsec = 0;

    pthread_t load, prep, id;

    pthread_create(&load, nullptr, load_data_thread, &state_unsafe);
    pthread_create(&prep, nullptr, preprocess_discard_thread, &state_unsafe);
    pthread_create(&id, nullptr, identify_driveable_thread, &state_unsafe);

    auto state = static_cast<struct state *>(&state_unsafe);

    while (nh.ok())
    {
        if (runflag)
        {
            pthread_mutex_lock(state->cloud.mutex);
            state->cloud.has_cloud = true;
            pthread_cond_signal(state->cloud.cloud_available);
            while (!state->cloud.processed)
            {
                pthread_cond_wait(state->cloud.cloud_processed, state->cloud.mutex);

                if (!running)
                {
                    pthread_mutex_unlock(state->cloud.mutex);
                    return 1;
                }
            }
            state->cloud.processed = false;
            pthread_mutex_unlock(state->cloud.mutex);
            runflag = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    pthread_join(load, nullptr);
    pthread_join(prep, nullptr);
    pthread_join(id, nullptr);

    std::cout << "Main thread is finished." << std::endl;

    return 1;
}
