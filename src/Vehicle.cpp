#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include "spline.h"
#include "Vehicle.h"
#include "helper.h"

using namespace std;

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::trajectory_update(const vector<TrajectoryPoint> next_trajectory, const Map& hwy_map) {

  lane = next_trajectory[1].lane;
  state = next_trajectory[1].state;
  pose.s = next_trajectory[1].s;
  pose.speed = next_trajectory[1].v;
  pose.a = next_trajectory[1].a;

  update_waypoints(hwy_map);

}

void Vehicle::update_waypoints(const Map& hwy_map) {

  next_x_vals.clear();
  next_y_vals.clear();

  // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
  // reuse previous left over points
  for (int i=0; i<previous_path_x.size(); ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  vector<vector<double>> spline_pts;
  spline_pts = get_spline_points(hwy_map);

  for (int i=0; i<spline_pts[0].size(); ++i) {
    next_x_vals.push_back(spline_pts[0][i]);
    next_y_vals.push_back(spline_pts[1][i]);
  }

}

vector<vector<double>>
Vehicle::get_spline_points(const Map& hwy_map, double target_x) {

  vector<double> pts_x;
  vector<double> pts_y;
  double ref_x = pose.x;
  double ref_y = pose.y;
  double ref_yaw = deg2rad(pose.yaw);
  auto prev_size = previous_path_x.size();

  if (prev_size < 2) {
    double prev_car_x = pose.x - cos(pose.yaw);
    double prev_car_y = pose.y - sin(pose.yaw);

    pts_x.push_back(prev_car_x);
    pts_x.push_back(pose.x);
    pts_y.push_back(prev_car_y);
    pts_y.push_back(pose.y);
  }
  else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_prev_x = previous_path_x[prev_size - 2];
    double ref_prev_y = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);

    pts_x.push_back(ref_prev_x);
    pts_x.push_back(ref_x);
    pts_y.push_back(ref_prev_y);
    pts_y.push_back(ref_y);
  }

  double car_s = (prev_size > 0)? end_path_s : pose.s;
  double car_d = lane_width * (0.5 + lane);
  if (lane == n_lanes-1)  car_d -= 0.2;  // to prevent "out of lane" error when ego is at the rightmost lane

  // add 30 ,60 90m points ahead of starting point (in Frenet)
  for (auto delta_s : {40, 60, 90}) {
    vector<double> wp = getXY(car_s + delta_s, car_d, hwy_map);
    pts_x.push_back(wp[0]);
    pts_y.push_back(wp[1]);
  }

  // rotate to car coord-frame inplace
  for (int i = 0; i < pts_x.size(); ++i) {
    global_to_local(pts_x[i], pts_y[i], ref_x, ref_y, ref_yaw);
  }

  // fit spline; the desired path
  tk::spline sp;
  sp.set_points(pts_x, pts_y);

  // pick points every .02 sec from the spline, given the x-range (target_x)
  double target_y = sp(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double N = target_dist / (t_tick * pose.speed * mph_to_mps);
  double step = target_x / N;
  vector<double> spline_x_vals;
  vector<double> spline_y_vals;

  for (int i = 0; i <= n_waypoints - prev_size; ++i) {
    double pt_x = step * (i+1);
    double pt_y = sp(pt_x);

    // rotate to global coord-frame inplace
    local_to_global(pt_x, pt_y, ref_x, ref_y, ref_yaw);

    spline_x_vals.push_back(pt_x);
    spline_y_vals.push_back(pt_y);
  }

  vector<vector<double>> s_pts;
  s_pts.emplace_back(spline_x_vals);
  s_pts.emplace_back(spline_y_vals);

  return s_pts;

}
