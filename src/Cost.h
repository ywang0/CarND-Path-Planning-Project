#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>
#include "OtherVehicle.h"
#include "types.h"
#include "constants.h"

class Cost {

public:
    double inefficient_cost(const std::vector<TrajectoryPoint>& trajectory);
    double buffer_cost(const std::vector<TrajectoryPoint>& trajectory,
                       const std::map<int, std::vector<OtherVehicle>>& predictions);
    double lane_change_cost(const std::vector<TrajectoryPoint> trajectory);
    double calculate_cost(const std::vector<TrajectoryPoint>& trajectory,
                          const std::map<int, std::vector<OtherVehicle>>& predictions);

private:
    std::map<std::string, double> weights {{"inefficient", 2e5},
                                           {"buffer", 1e5},
                                           {"lane_change", 5e3},
                                           {"collision", 1},  // not used
                                           {"max_accel", 1},  // not used
                                           {"max_jerk", 1},  // not used
                                           {"total_accel", 1},  // not used
                                           {"total_jerk", 1}};  // not used

    const double exponent_decay_factor = 0.42;
    const double KL_buffer_ahead_weight = 1.0;
    const double LC_buffer_ahead_weight = 0.5;

};


#endif //PATH_PLANNING_COST_H
