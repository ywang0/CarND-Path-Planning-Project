#ifndef PATH_PLANNING_OTHERVEHICLE_H
#define PATH_PLANNING_OTHERVEHICLE_H


#include "types.h"
#include "constants.h"

class OtherVehicle {

public:
    int id;
    OtherPose pose;
//    Deque<OtherPose, 4> poses;  // pose histories, used to calculate speed, accel

    OtherVehicle();
    OtherVehicle(int id, OtherPose pose);
    int get_lane();
//    void add_pose(OtherPose pose);

};


#endif //PATH_PLANNING_OTHERVEHICLE_H
