#pragma once

namespace planning {

struct RoiConfig {
    double roi_longitudinal_reang_start = 15.0;
    double roi_longitudinal_rang_end = 15.0;
    double parking_start_range = 12.0;
    bool parking_inwards = false;
};

struct SCurveConfig {
    double acc_weight = 1.0;
    double jerk_weight = 0.0;
    double kappa_penalty_weight = 100.0;
    double ref_s_weight = 0.1;
    double ref_v_weight = 0.0;
};

struct WarmStartConfig {
    double xy_grid_resolution = 0.3;
    double phi_grid_resolution = 0.1;
    int next_node_num = 16;
    double step_size = 0.1;
    double traj_forward_penalty = 1.0;
    double traj_back_penalty = 1.0;
    double traj_gear_switch_penalty = 3.0;
    int max_explored_num = 10000;
    double traj_steer_penalty = 3.0;
    double traj_steer_change_penalty = 1.0;
    double grid_a_star_xy_resolution = 0.25;
    double node_radius = 0.2;
    double traj_kappa_contraint_ratio = 0.5;
    double traj_short_length_penalty = 5.0;
    double traj_expected_shortest_length = 0.5;
    SCurveConfig s_curve_config;
};

struct PlannerOpenSpaceConfig {
    RoiConfig roi_config;
    WarmStartConfig warm_start_config;
};
}  // namespace planning
