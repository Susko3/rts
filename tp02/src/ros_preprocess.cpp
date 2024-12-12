#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include "ros_preprocess.h"

void data_stats(lidar_data *data)
{
    if (data->points.empty())
    {
        std::cout << "No points found in the file." << std::endl;
    }

    size_t num_points = data->points.size();
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();

    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (const auto &p : data->points)
    {
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        min_z = std::min(min_z, p.z);

        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
        max_z = std::max(max_z, p.z);

        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
    }

    float mean_x = sum_x / num_points;
    float mean_y = sum_y / num_points;
    float mean_z = sum_z / num_points;

    float sum_sq_diff_x = 0, sum_sq_diff_y = 0, sum_sq_diff_z = 0;
    for (const auto &p : data->points)
    {
        sum_sq_diff_x += (p.x - mean_x) * (p.x - mean_x);
        sum_sq_diff_y += (p.y - mean_y) * (p.y - mean_y);
        sum_sq_diff_z += (p.z - mean_z) * (p.z - mean_z);
    }

    float std_x = std::sqrt(sum_sq_diff_x / num_points);
    float std_y = std::sqrt(sum_sq_diff_y / num_points);
    float std_z = std::sqrt(sum_sq_diff_z / num_points);

    std::cout << "Number of points: " << num_points << std::endl;
    std::cout << "X: Min = " << min_x << ", Max = " << max_x << ", Mean = " << mean_x << ", Std = " << std_x << std::endl;
    std::cout << "Y: Min = " << min_y << ", Max = " << max_y << ", Mean = " << mean_y << ", Std = " << std_y << std::endl;
    std::cout << "Z: Min = " << min_z << ", Max = " << max_z << ", Mean = " << mean_z << ", Std = " << std_z << "\n"
              << std::endl;
}

lidar_data *load_data(sensor_msgs::PointCloud cloud)
{
    lidar_data *data = new lidar_data{};

    for (const auto point : cloud.points)
    {
        point3d p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        data->points.push_back(p);
    }

    data_stats(data);

    return data;
}

void write_data(std::string file_name, lidar_data *data)
{
    std::ofstream file{file_name};

    for (const auto &p : data->points)
    {
        file << p.x << ' ' << p.y << ' ' << p.z << '\n';
    }
}

void preprocess_discard(lidar_data *input, lidar_data *output, float forward = 30, float side = 15, float top = 2)
{
    assert(input);
    assert(output);
    assert(output->points.size() == 0);

    // 2.1.  Remove back of the car
    // 2.2.  Detect and remove two groups (clusters) of points that are located
    //       very close to the car. Covered by x<0 as well because
    //       cluster are located in x<0
    // 2.3.  Discard also points that clearly do not correspond to the
    //       ground/road (i.e., the outliers). Further than 30 m ahead,
    //       15 m to the sides and 4 m over the car
    for (const auto &p : input->points)
    {
        if (p.x > 0 && p.x < forward && abs(p.y) < side && p.z < top)
            output->points.push_back(p);
    }
}

void identify_driveable(lidar_data *input, lidar_data *output, float forward = 30, float side = 15, float maxDiff = 0.5, float maxIncline = 0.15)
{
    assert(input);
    assert(output);
    assert(output->points.size() == 0);

    forward = ceil(forward);
    side = ceil(side);

    std::vector<std::vector<point3d>> grid(forward * (side * 2));
    for (const auto &p : input->points)
    {
        int cellX = static_cast<int>(floor(p.x));
        int cellY = static_cast<int>(floor(p.y) + side);

        if (cellX >= 0 && cellX < forward && cellY >= 0 && cellY < side * 2)
        {
            int cellIndex = cellY * forward + cellX;
            grid[cellIndex].push_back(p);
        }
    }

    std::vector<point3d> drivable;

    for (const auto &cell : grid)
    {
        if (cell.empty())
            continue;

        // Calculate min and max z-values for the current cell
        float minZ = std::numeric_limits<float>::max();
        float maxZ = std::numeric_limits<float>::lowest();

        for (const auto &p : cell)
        {
            minZ = std::min(minZ, p.z);
            maxZ = std::max(maxZ, p.z);
        }

        // Check the z difference condition
        if ((maxZ - minZ) <= maxDiff)
        {
            // Add points from this cell to the result
            drivable.insert(drivable.end(), cell.begin(), cell.end());
        }
    }

    for (const auto &p : drivable)
    {
        float distance = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
        if (p.z <= distance * maxIncline)
        {
            output->points.push_back(p);
        }
    }
}
