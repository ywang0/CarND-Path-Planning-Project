#include <iostream>
#include "PathPlanner.h"

using namespace std;

PathPlanner::PathPlanner(Vehicle& ego, map<int, vector<OtherVehicle>>& predictions)
  : ego(ego), predictions(predictions) {}

PathPlanner::~PathPlanner() {}

vector<TrajectoryPoint> PathPlanner::choose_next_state() {

    vector<string> states = successor_states();

    vector<TrajectoryPoint> best_trajectory;
    double best_cost = numeric_limits<int>::max();

    for (auto s : states) {
        vector<TrajectoryPoint> trajectory = generate_trajectory(s);
        if (trajectory.size() > 1) {  // there is a valid trajectory
            double cost = cost_func.calculate_cost(trajectory, predictions);
            if (cost < best_cost) {
                best_cost = cost;
                best_trajectory = trajectory;
            }
        }
    }
    return best_trajectory;

}

vector<string> PathPlanner::successor_states() {
    vector<string> states;

    states.push_back("KL");
    if (ego.state == "KL") {
        if (ego.lane != 0) states.push_back("PLCL");
        if (ego.lane != 2) states.push_back("PLCR");
    }
    else if (ego.state == "PLCR") {
        states.push_back("PLCR");
        states.push_back("LCR");
    }
    else if (ego.state == "PLCL") {
        states.push_back("PLCL");
        states.push_back("LCL");
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;

}

vector<TrajectoryPoint> PathPlanner::generate_trajectory(string new_state) {

    vector<TrajectoryPoint> trajectory;
    if (new_state == "CS") {
        trajectory = constant_speed_trajectory();
    } else if (new_state == "KL") {
        trajectory = keep_lane_trajectory();
    } else if (new_state == "LCL" || new_state == "LCR") {
        trajectory = lane_change_trajectory(new_state);
    } else if (new_state == "PLCL" || new_state == "PLCR") {
        trajectory = prep_lane_change_trajectory(new_state);
    }
    return trajectory;
}

vector<double>
PathPlanner::get_kinematics(int lane, double dt, bool lane_change) {

    double max_acceleration = exp(-0.1*ego.pose.speed) + base_max_acceleration;
    double max_velocity_limit = (max_acceleration + ego.pose.speed) * mph_to_mps;  // mps
    double min_velocity_limit = (ego.pose.speed - max_acceleration) * mph_to_mps;  // mps
    double new_s; // m
    double new_speed;  // mps and converted to mph at the end
    double new_accel;  // mpss
    OtherVehicle veh_ahead;
//    OtherVehicle veh_behind;
    double target_speed_in_mps = target_speed * mph_to_mps;  // m/s

    if (get_vehicle_ahead(lane, veh_ahead)) {

//        if (get_vehicle_behind(lane, veh_behind)) {
//
//            new_speed = veh_ahead.pose.v; // in mps, must travel at the speed of traffic, regardless of preferred buffer
//            new_speed = min(min(new_speed, max_velocity_limit), target_speed);
//
//        } else {
            double delta_s = veh_ahead.pose.s - ego.pose.s;

            // for KL, calculate preferred buffer based on how far the vehicle in front
            double preferred_buffer;
            if (delta_s < min_preferred_buffer) preferred_buffer = min_preferred_buffer;
            else if (delta_s > max_preferred_buffer) preferred_buffer = max_preferred_buffer;
            else preferred_buffer = delta_s;

            double buffer = (lane_change)? change_lane_buffer : preferred_buffer;  // m
            double max_speed_in_front = delta_s - buffer + veh_ahead.pose.v - 0.5*ego.pose.a;
            new_speed = min(min(max_speed_in_front, max_velocity_limit), target_speed_in_mps);

//        }
    } else {
        new_speed = min(max_velocity_limit, target_speed_in_mps);

    }

    new_speed = (new_speed > 0)? new_speed : 0;
    new_accel = (new_speed - ego.pose.speed*mph_to_mps); // mpss

    if (new_accel < -max_acceleration) {
        new_accel = -max_acceleration;   // mpss
        new_speed = min_velocity_limit;  // mps
    }

    new_speed = ego.pose.speed*mph_to_mps + new_accel*dt;      // mps
    new_s = ego.pose.s + ego.pose.speed*mph_to_mps*dt + 0.5*new_accel*dt*dt;  // m
    new_speed *= mps_to_mph;

    return {new_s, new_speed, new_accel};

}

vector<TrajectoryPoint> PathPlanner::constant_speed_trajectory() {

    vector<TrajectoryPoint> trajectory = {TrajectoryPoint(ego.lane, ego.pose.s, ego.pose.speed, ego.pose.a, ego.state)};
    double next_s = ego.pose.s + ego.pose.speed;
    trajectory.emplace_back(TrajectoryPoint(ego.lane, next_s, ego.pose.speed, ego.pose.a, ego.state));
//  next_s = ego.pose.s + ego.pose.speed*t_interval;
    trajectory.emplace_back(TrajectoryPoint(ego.lane, next_s, ego.pose.speed, ego.pose.a, ego.state));

    return trajectory;

}

vector<TrajectoryPoint> PathPlanner::keep_lane_trajectory() {

    vector<TrajectoryPoint> trajectory = {TrajectoryPoint(ego.lane, ego.pose.s, ego.pose.speed, ego.pose.a, ego.state)};
    vector<double> new_kinematics = get_kinematics(ego.lane, t_interval);
    double new_s = new_kinematics[0];
    double new_v = new_kinematics[1];
    double new_a = new_kinematics[2];

    trajectory.emplace_back(TrajectoryPoint(ego.lane, new_s, new_v, new_a, "KL"));
    trajectory.emplace_back(TrajectoryPoint(ego.lane, new_s, new_v, new_a, "KL"));

    return trajectory;

}

vector<TrajectoryPoint> PathPlanner::prep_lane_change_trajectory(string new_state) {

    vector<TrajectoryPoint> trajectory = {TrajectoryPoint(ego.lane, ego.pose.s, ego.pose.speed, ego.pose.a, ego.state)};

    double new_s;
    double new_v;
    double new_a;
    double new_ss;
    double new_vv;
    double new_aa;
    OtherVehicle veh_behind;
    int new_lane = ego.lane + lane_direction[new_state];
    vector<double> curr_lane_new_kinematics = get_kinematics(ego.lane, t_interval);
    vector<double> next_lane_new_kinematics = get_kinematics(new_lane, t_interval, true);

    if (get_vehicle_behind(ego.lane, veh_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    }
    else {
        vector<double> best_kinematics;

        //Choose kinematics with lowest velocity
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
            best_kinematics = next_lane_new_kinematics;
        else
            best_kinematics = curr_lane_new_kinematics;

        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }
    trajectory.emplace_back(TrajectoryPoint(ego.lane, new_s, new_v, new_a, new_state));

    new_ss = next_lane_new_kinematics[0];
    new_vv = next_lane_new_kinematics[1];
    new_aa = next_lane_new_kinematics[2];

    trajectory.emplace_back(TrajectoryPoint(new_lane, new_ss, new_vv, new_aa, new_state));

    return trajectory;

}

vector<TrajectoryPoint> PathPlanner::lane_change_trajectory(string new_state) {

    vector<TrajectoryPoint> trajectory = {TrajectoryPoint(ego.lane, ego.pose.s, ego.pose.speed, ego.pose.a, ego.state)};
    OtherVehicle veh_ahead;
    OtherVehicle veh_behind;
    int new_lane = ego.lane + lane_direction[new_state];
    vector<double> new_lane_kinematics = get_kinematics(new_lane, t_interval, true);
    double new_s = new_lane_kinematics[0];
    double new_v = new_lane_kinematics[1];
    double new_a = new_lane_kinematics[2];

    bool is_enough_buffer_ahead = true;
    bool is_enough_buffer_behind = true;
    if (get_vehicle_ahead(new_lane, veh_ahead)) {
        is_enough_buffer_ahead = (veh_ahead.pose.s + veh_ahead.pose.v*t_interval - new_s - change_lane_buffer) > 0;
    }
    if (get_vehicle_behind(new_lane, veh_behind)) {
        is_enough_buffer_behind = (new_s - veh_behind.pose.s - veh_behind.pose.v*t_interval - change_lane_buffer) > 0;
    }

    if (is_enough_buffer_ahead && is_enough_buffer_behind) {
        trajectory.emplace_back(TrajectoryPoint{new_lane, new_s, new_v, new_a, new_state});
        trajectory.emplace_back(TrajectoryPoint{new_lane, new_s, new_v, new_a, new_state});
    }
    return trajectory;

}

bool PathPlanner::get_vehicle_ahead(int lane, OtherVehicle& veh_ahead) {

    double min_s = numeric_limits<double>::max();
    bool found = false;
    OtherVehicle veh;

    for (auto it=predictions.begin(); it!=predictions.end(); ++it) {
        veh = it->second[0];

        if (veh.get_lane() == lane) {
            if (veh.pose.s > ego.pose.s && veh.pose.s < min_s && ((veh.pose.s - ego.pose.s) < horizon)) {
                min_s = veh.pose.s;
                veh_ahead = veh;
                found = true;
            }
        }
    }
    return found;

}

bool PathPlanner::get_vehicle_behind(int lane, OtherVehicle& veh_behind) {

    double max_s = numeric_limits<double>::min();
    bool found = false;
    OtherVehicle veh;

    for (auto it=predictions.begin(); it!=predictions.end(); ++it) {
        veh = it->second[0];

        if (veh.get_lane() == lane) {
            if (ego.pose.s > veh.pose.s && veh.pose.s > max_s && ((ego.pose.s - veh.pose.s) < horizon)) {
                max_s = veh.pose.s;
                veh_behind = veh;
                found = true;
            }
        }
    }
    return found;

}