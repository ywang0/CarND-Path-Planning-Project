#ifndef PATH_PLANNING_ROAD_H
#define PATH_PLANNING_ROAD_H


#include <map>
#include "json.hpp"
#include "Map.h"
#include "OtherVehicle.h"
#include "PathPlanner.h"
#include "Vehicle.h"
#include "types.h"

using json = nlohmann::json;

class Road {

public:
    Vehicle ego;

    Road(const Map& hwy_map);
    ~Road();
    void advance(const json& msg);

private:
    Map hwy_map;
    std::map<int, OtherPose> other_vehs;

    void update_ego_pose(const json& msg);
    void update_other_vehicles_poses(const json& msg);
    std::map<int, std::vector<OtherVehicle>> generate_predictions();

};


#endif //PATH_PLANNING_ROAD_H
