#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "Map.h"

using namespace std;

Map::Map(const string map_file) {

  max_s = 6945.554;
  ifstream in_map(map_file.c_str(), ifstream::in);
  string line;
  double x;
  double y;
  double s;
  double dx;
  double dy;

  while (getline(in_map, line)) {
    istringstream iss(line);
    iss >> x;
    iss >> y;
    iss >> s;
    iss >>dx;
    iss >>dy;
    waypoints_x.push_back(x);
    waypoints_y.push_back(y);
    waypoints_s.push_back(s);
    waypoints_dx.push_back(dx);
    waypoints_dy.push_back(dy);
  }

  fit_splines();

}

Map::~Map() {}

void Map::fit_splines(bool is_circular) {
  if (is_circular) {
    int n_waypoints = waypoints_x.size();
    double last_delta_s = waypoints_s[n_waypoints-1] - waypoints_s[n_waypoints-2];
    waypoints_x.push_back(waypoints_x[0]);
    waypoints_y.push_back(waypoints_y[0]);
    waypoints_s.push_back(waypoints_s[n_waypoints-1] + last_delta_s);
    waypoints_dx.push_back(waypoints_dx[0]);
    waypoints_dy.push_back(waypoints_dy[0]);
  }

  spline_x_s.set_points(waypoints_s, waypoints_x);
  spline_y_s.set_points(waypoints_s, waypoints_y);
  spline_dx_s.set_points(waypoints_s, waypoints_dx);
  spline_dy_s.set_points(waypoints_s, waypoints_dy);

}
