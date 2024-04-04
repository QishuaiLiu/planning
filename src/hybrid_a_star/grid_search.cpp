#include "hybrid_a_star/grid_search.h"
#include <cmath>

namespace planning {
GridSearch::GridSearch(const PlannerOpenSpaceConfig &open_space_conf) {
    xy_grid_resolution_ = open_space_conf.warm_start_config.grid_a_star_xy_resolution;
    node_radius_ = open_space_conf.warm_start_config.node_radius;
}

double GridSearch::EuclidDistance(const double x1, const double y1, const double x2,
                                  const double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool GridSearch::CheckConstraints(std::shared_ptr<Node2d> node) {
    const double node_grid_x = node->GetGridX();
    const double node_grid_y = node->GetGridY();
    if (node_grid_x > max_grid_x_ || node_grid_x < 0 || node_grid_y > max_grid_y_ ||
        node_grid_y < 0) {
        return false;
    }

    if (obstacles_linesegments_vec_.empty()) {
        return true;
    }

    for (const auto &obstacle_linesegments : obstacles_linesegments_vec_) {
        for (const math::LineSegment2d &linesegment : obstacle_linesegments) {
            if (linesegment.DistanceTo(
                    {static_cast<double>(node->GetGridX() * xy_grid_resolution_),
                     static_cast<double>(node->GetGridY() * xy_grid_resolution_)}) < node_radius_) {
                return false;
            }
        }
    }
    return true;
}

std::vector<std::shared_ptr<Node2d>> GridSearch::GenerateNextNodes(
    std::shared_ptr<Node2d> current_node) {
    int current_node_x = current_node->GetGridX();
    int current_node_y = current_node->GetGridY();
    double current_node_path_cost = current_node->GetPathCost();
    double diagonal_distance = std::sqrt(2.0);
    std::vector<std::shared_ptr<Node2d>> next_nodes;
    next_nodes.resize(8);
    std::vector<int> dir_x = {0, 1, 1, 1, 0, -1, -1, -1};
    std::vector<int> dir_y = {1, 1, 0, -1, -1, -1, 0, 1};
    std::vector<double> cost(8);
    for (int i = 0; i < 8; ++i) {
        cost[i] = std::abs(dir_x[i] + dir_y[i]) == 1 ? 1.0 : diagonal_distance;
    }
    for (int i = 0; i < 8; ++i) {
        next_nodes[i] = std::make_shared<Node2d>(current_node_x + dir_x[i],
                                                 current_node_y + dir_y[i], XYbounds_);
        next_nodes[i]->SetPathCost(cost[i]);
    }
    return next_nodes;
}

bool GridSearch::GenerateAStarPath(
    const double sx, const double sy, const double ex, const double ey,
    const std::vector<double> &XYbounds,
    const std::vector<std::vector<math::LineSegment2d>> &obstacles_linesegments_vec,
    GridAStarResult *result) {
    std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>,
                        cmp>
        open_pq;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;

    XYbounds_ = XYbounds;
    std::shared_ptr<Node2d> start_node =
        std::make_shared<Node2d>(sx, sy, xy_grid_resolution_, XYbounds_);
    std::shared_ptr<Node2d> end_node =
        std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
    std::shared_ptr<Node2d> final_node_ = nullptr;
    obstacles_linesegments_vec_ = obstacles_linesegments_vec;
    open_set.emplace(start_node->GetIndex(), start_node);
    open_pq.emplace(start_node->GetIndex(), start_node->GetCost());

    size_t explored_node_num = 0;
    while (!open_pq.empty()) {
    }

    if (final_node_ == nullptr) {
        return false;
    }
    LoadGridAStarResult(result);

    return true;
}
}  // namespace planning
