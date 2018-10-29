#include "Road.h"

using namespace std;

extern vector<double> getFrenet(double x, double y, double theta, const Map& hwy_map);

Road::Road(const Map& hwy_map): hwy_map(hwy_map) {}

Road::~Road() {}

void Road::advance(const json& msg) {

  map<int, vector<OtherVehicle>> predictions;
  vector<TrajectoryPoint> trajectory;

  update_ego_pose(msg);
  update_other_vehicles_poses(msg);
  predictions = generate_predictions();
  PathPlanner path_planner = PathPlanner(ego, predictions);
  trajectory = path_planner.choose_next_state();
  ego.trajectory_update(trajectory, hwy_map);

}

void Road::update_ego_pose(const json& msg) {

  ego.pose.x = msg["x"];
  ego.pose.y = msg["y"];
  ego.pose.s = msg["s"];
  ego.pose.d = msg["d"];
  ego.pose.yaw = msg["yaw"];
  ego.pose.speed = msg["speed"];
  ego.previous_path_x = msg["previous_path_x"].get<vector<double>>();
  ego.previous_path_y = msg["previous_path_y"].get<vector<double>>();
  ego.end_path_s = msg["end_path_s"];
  ego.end_path_d = msg["end_path_d"];

}

void Road::update_other_vehicles_poses(const json& msg) {

  auto sensor_fusion = msg["sensor_fusion"].get<vector<vector<double>>>();
  other_vehs.clear();

  for (auto& sf : sensor_fusion) {
    double d = sf[6];
    if (d > 0 && d < lane_width * n_lanes) {
      int id = sf[0];
      double x = sf[1];
      double y = sf[2];
      double vx = sf[3];  // m/s
      double vy = sf[4];  // m/s
      double s = sf[5];
      double v = sqrt(vx * vx + vy * vy);
      OtherPose pose{x, y, vx, vy, s, d, v};
      other_vehs[id] = pose;
    }
  }
}

map<int, std::vector<OtherVehicle>>
Road::generate_predictions() {

  map<int, vector<OtherVehicle>> predictions;

  for (auto it=other_vehs.begin(); it!=other_vehs.end(); ++it) {
    int id = it->first;
    OtherPose pose = it->second;
    vector<OtherVehicle> veh_preds;

    for (int i=0; i<n_waypoints; ++i) {  // assuming other vehicles are at constant speed
      double next_x = pose.x + pose.vx * t_tick * i;  // m
      double next_y = pose.y + pose.vy * t_tick * i;  // m
      double v = sqrt(pose.vx*pose.vx + pose.vy*pose.vy);
      vector<double> fn_coord = getFrenet(next_x, next_y, atan2(pose.vy, pose.vx), hwy_map);
      OtherPose next_pose {next_x, next_y, pose.vx, pose.vy, fn_coord[0], fn_coord[1], v};
      veh_preds.emplace_back(OtherVehicle(id, next_pose));
    }
    predictions[id] = veh_preds;
  }
  return predictions;

}
