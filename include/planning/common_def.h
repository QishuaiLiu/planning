#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
constexpr float M_PIf = M_PI;

namespace obstacle {
struct Point2d {
    float x;
    float y;
    float theta;
    Point2d(const float x_value, const float y_value, const float theta_value)
        : x(x_value), y(y_value), theta(theta_value){};
    Point2d() : x(0), y(0), theta(0){};
};
struct BoundingBox {
    Point2d center;
    float length;
    float width;
};
};  // namespace obstacle
