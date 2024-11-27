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

lidar_data *load_data(std::string file_name);
void write_data(std::string file_name, lidar_data *data);

void preprocess_discard(lidar_data* input, lidar_data* output);

// void discard_behind(lidar_data *data);
// void discard_car_points(lidar_data *data);
// // too high or too low on Z direction
// void discard_outliers(lidar_data *data);

// discard walls, sidewalks, other obstacles
void identify_driveable(lidar_data* input, lidar_data* output);
