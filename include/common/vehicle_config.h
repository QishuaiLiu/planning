
#pragma once
#include <string>

namespace common {
// struct Transform {
//     int source_frame = 1;  // Also known as "frame_id."

//     int target_frame = 2;  // Also known as "child_frame_id."

//     // Position of target in the source frame.
//     Point3D translation = 3;

//     // Activate rotation from the source frame to the target frame.
//     Quaternion rotation = 4;
// };
struct LatencyParam {
    double dead_time = 0;
    double rise_time = 0;
    double peak_time = 0;
    double settling_time = 0;
};

struct VehicleParam {
    std::string vehicle_id = "mkz";
    double front_edge_to_center = 3.89;
    double back_edge_to_center = 1.043;
    double left_edge_to_center = 1.055;
    double right_edge_to_center = 1.055;

    double length = 4.933;
    double width = 2.11;
    double height = 1.48;

    double min_turn_radius = 5.05386147161;
    double max_acceleration = 2.0;
    double max_deceleration = -6.0;

    // The following items are used to compute trajectory constraints in
    // planning/control/canbus,
    // vehicle max steer angle
    double max_steer_angle = 8.20304748437;
    // vehicle max steer rate; how fast can the steering wheel turn.
    double max_steer_angle_rate = 6.98131700798;
    // vehicle min steer rate;
    double min_steer_angle_rate = 0;
    // ratio between the turn of steering wheel and the turn of wheels
    double steer_ratio = 16;
    // the distance between the front and back wheels
    double wheel_base = 2.8448;
    // Tire effective rolling radius (vertical distance between the wheel center
    // and the ground).
    double wheel_rolling_radius = 0.335;

    // minimum differentiable vehicle speed, in m/s
    float max_abs_speed_when_stopped = 0.2;

    // minimum value get from chassis.brake, in percentage
    double brake_deadzone = 14.5;
    // minimum value get from chassis.throttle, in percentage
    double throttle_deadzone = 15.7;

    // vehicle latency parameters
    LatencyParam steering_latency_param;
    LatencyParam throttle_latency_param;
    LatencyParam brake_latency_param;
};
}  // namespace common
