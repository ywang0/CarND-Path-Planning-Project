#include "OtherVehicle.h"

OtherVehicle::OtherVehicle() {}

OtherVehicle::OtherVehicle(int id, OtherPose pose) : id(id), pose(pose) {}

int OtherVehicle::get_lane() {
  int lane = -1;
  double d = pose.d;
  while (d > 0) {
    lane += 1;
    d -= lane_width;
  }
  return lane;
}

//void OtherVehicle::add_pose(OtherPose pose) {
//  poses.add(pose);
//}

