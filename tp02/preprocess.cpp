#include "preprocess.h"
#include <cassert>
#include <fstream>

lidar_data *load_data(std::string file_name)
{
    lidar_data *data = new lidar_data{};

    std::ifstream file{file_name};

    while (!file.eof())
    {
        point3d p;
        file >> p.x >> p.y >> p.z;
        data->points.push_back(p);
    }

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

void preprocess_discard(lidar_data *input, lidar_data *output)
{
    assert(input);
    assert(output);
    assert(output->points.size() == 0);

    for (const auto &p : input->points)
    {
        if (p.x > 0)
            output->points.push_back(p);
    }
}

void identify_driveable(lidar_data *input, lidar_data *output)
{
    assert(input);
    assert(output);
    assert(output->points.size() == 0);

    for (const auto &p : input->points)
    {
        if (p.y > 0)
            output->points.push_back(p);
    }
}
