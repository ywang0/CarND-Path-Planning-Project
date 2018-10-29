#include <iostream>
#include <map>
#include <vector>
#include "Cost.h"

using namespace std;

double Cost::calculate_cost(const vector<TrajectoryPoint>& trajectory,
                            const map<int, vector<OtherVehicle>>& predictions) {

  double inefficient_c = inefficient_cost(trajectory);
  double buffer_c = buffer_cost(trajectory, predictions);
  double lc_cost = lane_change_cost(trajectory);

  return weights["inefficient"] * inefficient_c + weights["buffer"] * buffer_c + \
         weights["lane_change"] * lc_cost;

}

double Cost::inefficient_cost(const std::vector<TrajectoryPoint>& trajectory) {

  double v1 = trajectory[1].v;
  double v2 = trajectory[2].v;
  double delta_v = fabs(target_speed-v1) + fabs(target_speed-v2);
  double cost = 1 - exp(-delta_v / target_speed);

  return cost;

}

double Cost::buffer_cost(const vector<TrajectoryPoint>& trajectory,
                         const map<int, vector<OtherVehicle>>& predictions) {

  double ego_s = trajectory[1].s;
  int ego_lane = trajectory[1].lane;
  string ego_state = trajectory[1].state;

  double min_s = numeric_limits<double>::max();
  double max_s = numeric_limits<double>::min();
  double buffer_ahead = numeric_limits<double>::max();
  double buffer_behind = numeric_limits<double>::max();

  for (auto it=predictions.begin(); it!=predictions.end(); ++it) {
    OtherVehicle other = it->second[n_waypoints - 1];  // the last prediction for this vehicle
    int other_lane = other.get_lane();

    // get vehicle in front
    if (other_lane == ego_lane && other.pose.s > ego_s && other.pose.s < min_s) {
      min_s = other.pose.s;
      buffer_ahead = min_s - ego_s;
    }
    // get vehicle behind
    if (other_lane == ego_lane && other.pose.s < ego_s && other.pose.s > max_s) {
      max_s = other.pose.s;
      buffer_behind = ego_s - max_s;
    }
  }

  // buffer cost = (exponent_decay_factor * e)^(-x)
  double buffer_ahead_c = 1 / pow(exponent_decay_factor * M_E, buffer_ahead);
  double buffer_behind_c = 1 / pow(exponent_decay_factor * M_E, buffer_behind);

  double cost;
  if (ego_state == "LCL" || ego_state == "LCR")
    cost = LC_buffer_ahead_weight*buffer_ahead_c + (1-LC_buffer_ahead_weight)*buffer_behind_c;
  else
    cost = KL_buffer_ahead_weight*buffer_ahead_c + (1-KL_buffer_ahead_weight)*buffer_behind_c;

  return cost;

}

double Cost::lane_change_cost(const vector<TrajectoryPoint> trajectory) {

  if (trajectory[1].state == "LCL" || trajectory[1].state == "LCR")
    return 1;

}

//double max_accel_cost() {}
//
//double max_jerk_cost() {}
//
//double collision_cost() {}
//
//double total_accel_cost() {}
//
//double total_jerk_cost() {}
