#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <vector>
#include <string>

struct point3d
{
    float x;
    float y;
    float z;
};

struct lidar_data
{
    std::vector<point3d> points;
};

void data_stats(const lidar_data &data);

void load_data(std::string file_name, lidar_data &data);

void write_data(std::string file_name, const lidar_data &data);

void preprocess_discard(const lidar_data &input, lidar_data &output, float forward = 30, float side = 15, float top = 2);

// void discard_behind(lidar_data *data);
// void discard_car_points(lidar_data *data);
// // too high or too low on Z direction
// void discard_outliers(lidar_data *data);

// discard walls, sidewalks, other obstacles
void identify_driveable(const lidar_data &input, lidar_data &output, float forward = 30, float side = 15, float maxDiff = 1, float maxIncline = 0.01);

#endif
