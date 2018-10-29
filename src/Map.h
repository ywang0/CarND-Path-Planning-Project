#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <string>
#include <vector>
#include "spline.h"


class Map {

public:
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

    tk::spline spline_x_s;
    tk::spline spline_y_s;
    tk::spline spline_dx_s;
    tk::spline spline_dy_s;

    Map() {}
    Map(const std::string map_file);
    ~Map();


private:
    double max_s;

    void fit_splines(bool is_circular=true);

};


#endif //PATH_PLANNING_MAP_H
