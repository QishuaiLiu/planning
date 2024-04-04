#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
constexpr float M_PIf = M_PI;

namespace obstacle {
struct Point2d {
    float x;
    float y;
    float theta;
};
struct BoundingBox {
    Point2d center;
    float length;
    float width;
};
};  // namespace obstacle
