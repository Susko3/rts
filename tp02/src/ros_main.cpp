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

#include "preprocess.h"
#include "processing_threads.h"
#include "utils.h"

static struct state state_unsafe;

sensor_msgs::PointCloud2::ConstPtr point_cloud;
ros::Publisher newPointCloud;

pthread_mutex_t point_cloud_mutex;
pthread_cond_t point_cloud_empty_cond;
pthread_cond_t point_cloud_available_cond;
bool point_cloud_available = false;

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

        pthread_mutex_lock(&point_cloud_mutex);
        pthread_cond_signal(&point_cloud_empty_cond);
        pthread_cond_signal(&point_cloud_available_cond);
        pthread_mutex_unlock(&point_cloud_mutex);
    }
}

void setup_point_cloud_mutex()
{
    pthread_mutex_init(&point_cloud_mutex, nullptr);
    pthread_cond_init(&point_cloud_empty_cond, nullptr);
    pthread_cond_init(&point_cloud_available_cond, nullptr);
}

void handlePointCloud(sensor_msgs::PointCloud2::ConstPtr scan_out)
{
    pthread_mutex_lock(&point_cloud_mutex);

    while (point_cloud_available)
    {
        pthread_cond_wait(&point_cloud_empty_cond, &point_cloud_mutex);

        if (!state_unsafe.running)
            return;
    }

    point_cloud = scan_out;
    point_cloud_available = true;
    pthread_cond_signal(&point_cloud_available_cond);

    pthread_mutex_unlock(&point_cloud_mutex);
}

void load_data_from_ros(lidar_data *data)
{
    sensor_msgs::PointCloud output;

    pthread_mutex_lock(&point_cloud_mutex);

    while (!point_cloud_available)
    {
        pthread_cond_wait(&point_cloud_available_cond, &point_cloud_mutex);
        
        if (!state_unsafe.running)
            return;
    }

    sensor_msgs::convertPointCloud2ToPointCloud(*point_cloud, output);
    point_cloud_available = false;
    pthread_cond_signal(&point_cloud_empty_cond);

    pthread_mutex_unlock(&point_cloud_mutex);

    for (const auto &point : output.points)
    {
        point3d p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        data->points.push_back(p);
    }

    data_stats(data);
}

void publish_data(lidar_data *data)
{
    sensor_msgs::PointCloud final_output;
    final_output.header = point_cloud->header; // Use the header from /velodyne_points

    for (const auto &point : data->points)
    {
        geometry_msgs::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        final_output.points.push_back(p);
    }

    newPointCloud.publish(final_output);
}

void setup_signal_handler()
{
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = signal_handler;
    sigaction(SIGINT, &sa, nullptr) == 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strdemo");

    setup_point_cloud_mutex();

    ros::NodeHandle nh("~");
    newPointCloud = nh.advertise<sensor_msgs::PointCloud>("/output_results", 100);
    ros::Subscriber PointCloudHandlervelodyne =
        nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, handlePointCloud);
    ros::Rate rate(100.0);

    set_realtime_priority();
    pin_this_thread();
    increase_clock_resolution();

    state_unsafe.load_data_blocking = load_data_from_ros;
    state_unsafe.publish_data = publish_data;

    state_unsafe.running = 1;
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

    while (nh.ok() && state_unsafe.running)
    {
        ros::spinOnce();
        rate.sleep();
    }

    pthread_join(load, nullptr);
    pthread_join(prep, nullptr);
    pthread_join(id, nullptr);

    std::cout << "Main thread is finished." << std::endl;

    return 1;
}
