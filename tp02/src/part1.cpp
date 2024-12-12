#include <cassert>
#include <cstdio>

#include "preprocess.h"
#include "utils.h"

timespec start;

void start_timing()
{
    start = {};
    assert(clock_gettime(CLOCK_MONOTONIC, &start) == 0);
}

timespec end_timing()
{
    timespec end{};
    assert(clock_gettime(CLOCK_MONOTONIC, &end) == 0);

    timespec result{};
    timespec_diff(&end, &start, &result);
    return result;
}

void assert_some_filtered(const lidar_data &pr, const lidar_data &dr)
{
    assert(pr.points.size() >= dr.points.size());
}

void process(std::string file_name)
{
    start_timing();
    lidar_data data{};
    load_data(file_name, data);
    auto delta_load = end_timing();

    std::printf("Loading took:\t\t");
    print(&delta_load);
    std::printf("\n");

    lidar_data preprocessed = {};

    start_timing();
    preprocess_discard(data, preprocessed);
    auto delta_pp = end_timing();

    std::printf("Preprocessing took:\t");
    print(&delta_pp);
    std::printf("\n");

    assert_some_filtered(data, preprocessed);

    lidar_data driveable = {};

    start_timing();
    identify_driveable(preprocessed, driveable);
    auto delta_id = end_timing();

    std::printf("Identifying took:\t");
    print(&delta_id);
    std::printf("\n");

    assert_some_filtered(preprocessed, driveable);

    write_data(file_name + "_out.txt", driveable);
}

int main()
{
    assert(set_realtime_priority());
    assert(pin_this_thread());
    assert(increase_clock_resolution());

    std::printf("\nPoint cloud 1:\n");
    process("point_cloud1.txt");
    std::printf("\nPoint cloud 2:\n");
    process("point_cloud2.txt");
    std::printf("\nPoint cloud 3:\n");
    process("point_cloud3.txt");

    reset_clock_resolution();
}
