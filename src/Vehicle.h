#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


#include <map>
#include <string>
#include <vector>
#include "Map.h"
#include "OtherVehicle.h"
#include "constants.h"

class Vehicle {

public:
    std::string state = "KL";
    int lane = 1;

    // localization variables
    EgoPose pose;
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;

    // next waypoins
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

//    Deque<EgoPose, 4> poses;  // pose histories, used to calculate accel, jerk

    Vehicle();
    ~Vehicle();
    void trajectory_update(const std::vector<TrajectoryPoint> next_trajectory, const Map& hwy_map);

private:
    void update_waypoints(const Map& hwy_map);
    std::vector<std::vector<double>> get_spline_points(const Map& hwy_map, double target_x=30);

};


#endif //PATH_PLANNING_VEHICLE_H
