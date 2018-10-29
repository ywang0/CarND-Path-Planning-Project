#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <map>
#include <vector>
#include "Cost.h"
#include "OtherVehicle.h"
#include "Vehicle.h"

class PathPlanner {

public:
    PathPlanner(Vehicle& ego, std::map<int, std::vector<OtherVehicle>>& predictions);
    ~PathPlanner();
    std::vector<TrajectoryPoint> choose_next_state();

private:
    Cost cost_func;
    Vehicle ego;
    std::map<int, std::vector<OtherVehicle>> predictions;
    std::map<std::string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

    std::vector<std::string> successor_states();
    std::vector<TrajectoryPoint> generate_trajectory(std::string state);
    std::vector<double> get_kinematics(int lane, double dt, bool lane_change=false);
    std::vector<TrajectoryPoint> constant_speed_trajectory();
    std::vector<TrajectoryPoint> keep_lane_trajectory();
    std::vector<TrajectoryPoint> lane_change_trajectory(std::string state);
    std::vector<TrajectoryPoint> prep_lane_change_trajectory(std::string state);
    bool get_vehicle_behind(int lane, OtherVehicle& veh_behind);
    bool get_vehicle_ahead(int lane, OtherVehicle& veh_ahead);

private:
    int max_preferred_buffer = 40; // impacts "keep lane" behavior.
    int min_preferred_buffer = 20; // impacts "keep lane" behavior.
    int change_lane_buffer = 30; // impacts "change lane" behavior.
    double base_max_acceleration = 3.36;  // m/s^2

};


#endif //PATH_PLANNING_PATHPLANNER_H
