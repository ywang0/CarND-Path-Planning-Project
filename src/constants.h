#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H


const double target_speed = 49.50;  // mph
const double mps_to_mph = 2.24;
const double mph_to_mps = 0.447;
const double t_tick = 0.02;  // 20 ms
const double horizon = 200.0;
const int n_waypoints = 25;
const double t_interval = t_tick * n_waypoints;
const int n_lanes = 3;
const double lane_width = 4.0;


#endif //PATH_PLANNING_CONSTANTS_H
