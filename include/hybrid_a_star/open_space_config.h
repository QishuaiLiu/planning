#pragma once
#include <string>

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

// ipopt
struct IpoptConfig {
    int ipopt_print_level = 0;
    int mumps_mem_percent = 6000;
    double mumps_pivtol = 1e-06;
    int ipopt_max_iter = 100;
    double ipopt_tol = 1e-05;
    double ipopt_acceptable_constr_viol_tol = 0.1;
    double ipopt_min_hessian_perturbation = 1e-12;
    double ipopt_jacobian_regularization_value = 1e-07;
    std::string ipopt_print_timing_statistics = "yes";
    std::string ipopt_alpha_for_y = "min";
    std::string ipopt_recalc_y = "yes";
};

struct DualVariableWarmStartConfig {
    IpoptConfig ipopt_config;
    double weight_d = 1.0;
    // qp_format= OSQP;
    double min_safety_distance = 0.01;
    bool debug_osqp = false;
    double beta = 1.0;
};

struct FemPosDeviationSmootherConfig {
    double weight_fem_pos_deviation = 1e8;
    double weight_path_length = 1.0;
    double weight_ref_deviation = 1e3;
    bool apply_curvature_constraint = false;
    double weight_curvature_constraint_slack_var = 1e8;
    double curvature_constraint = 0.2;
    int max_iter = 500;
    double ime_limit = 0.0;
    bool erbose = false;
    bool caled_termination = true;
    bool arm_start = true;
};

struct IterativeAnchoringSmootherConfig {
    FemPosDeviationSmootherConfig fem_pos_deviation_smoother_config;
    SCurveConfig s_curve_config;
    double interpolated_delta_s = 0.1;
    double reanchoring_trails_num = 50;
    double reanchoring_pos_stddev = 0.25;
    double reanchoring_length_stddev = 1.0;
    double estimate_bound = false;
    double default_bound = 2.0;
    double vehicle_shortest_dimension = 1.04;
    double collision_decrease_ratio = 0.9;
    double max_forward_v = 2.0;
    double max_reverse_v = 2.0;
    double max_forward_acc = 3.0;
    double max_reverse_acc = 2.0;
    double max_acc_jerk = 4.0;
    double delta_t = 0.2;
};

struct PlannerOpenSpaceConfig {
    RoiConfig roi_config;
    WarmStartConfig warm_start_config;
    double delta_t = 0.5;
    double near_destination_threshold = 0.05;
    bool enable_linear_interpolation = false;
};
}  // namespace planning
