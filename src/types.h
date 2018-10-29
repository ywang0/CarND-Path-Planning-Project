#ifndef PATH_PLANNING_TYPES_H
#define PATH_PLANNING_TYPES_H

#include <string>
//#include "constants.h"

struct EgoPose {
    double x;
    double y;
    double s;
    double d;
    double speed;  // mph
    double yaw;
    double a = 0;  // calculated, not sensor data
};

struct OtherPose {
    double x;
    double y;
    double vx;  // m/s
    double vy;  // m/s
    double s;
    double d;
    double v;  // calculated, not sensor data

    OtherPose() {}
    OtherPose(double x, double y, double vx, double vy, double s, double d, double v)
      : x(x), y(y), vx(vx), vy(vy), s(s), d(d), v(v)
      {}
};

struct TrajectoryPoint {
    int lane;
    double s;
    double v;
    double a;
    std::string state;

    TrajectoryPoint() {}
    TrajectoryPoint(int lane, double s, double v, double a, std::string state)
      : lane(lane), s(s), v(v), a(a), state(state)
      {}
};

template<typename T, size_t N>
struct Deque {
    T deque[N];
    unsigned int count_ = 0;

    void add(const T& elem) { deque[count_++ % N] = elem; }
    T operator[](int idx) const { int i = (count_ >= N)? (count_ + idx) % N : idx; return deque[i]; }
    T* data() noexcept { return deque; }
    int size() { return count_ > N ? N : count_; }
    int count() { return count_; }
};

#endif //PATH_PLANNING_TYPES_H

