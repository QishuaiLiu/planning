#include "hybrid_a_star/grid_search.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

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
        next_nodes[i]->SetPathCost(current_node_path_cost + cost[i]);
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

    GenerateDpMap(ex, ey, XYbounds_, obstacles_linesegments_vec_);
    open_set.emplace(start_node->GetIndex(), start_node);
    open_pq.emplace(start_node->GetIndex(), start_node->GetCost());

    size_t explored_node_num = 0;
    while (!open_pq.empty()) {
        std::string current_id = open_pq.top().first;
        open_pq.pop();
        std::shared_ptr<Node2d> current_node = open_set[current_id];

        if (*(current_node) == *(end_node)) {
            final_node_ = current_node;
            break;
        }
        close_set.emplace(current_node->GetIndex(), current_node);
        std::vector<std::shared_ptr<Node2d>> next_nodes =
            std::move(GenerateNextNodes(current_node));

        for (auto &next_node : next_nodes) {
            if (!CheckConstraints(next_node)) {
                continue;
            }
            if (close_set.find(next_node->GetIndex()) != close_set.end()) {
                continue;
            }

            if (open_set.find(next_node->GetIndex()) == open_set.end()) {
                ++explored_node_num;
                next_node->SetHeuristic(EuclidDistance(next_node->GetGridX(), next_node->GetGridY(),
                                                       end_node->GetGridX(), end_node->GetGridY()));
                next_node->SetPreNode(current_node);
                open_set.emplace(next_node->GetIndex(), next_node);
                open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
            }  // else if
        }
    }

    if (final_node_ == nullptr) {
        return false;
    }
    double cost = final_node_->GetPathCost();

#if 0
    LoadGridAStarResult(result);
#else
    (*result).path_cost = final_node_->GetPathCost() * xy_grid_resolution_;
    std::shared_ptr<Node2d> current_node = final_node_;
    std::vector<double> grid_a_x;
    std::vector<double> grid_a_y;
    while (current_node->GetPreNode() != nullptr) {
        grid_a_x.push_back(current_node->GetGridX() * xy_grid_resolution_ + XYbounds_[0]);
        grid_a_y.push_back(current_node->GetGridY() * xy_grid_resolution_ + XYbounds_[2]);
        current_node = current_node->GetPreNode();
    }
    std::reverse(grid_a_x.begin(), grid_a_x.end());
    std::reverse(grid_a_y.begin(), grid_a_y.end());
    (*result).x = std::move(grid_a_x);
    (*result).y = std::move(grid_a_y);
#endif

    return true;
}

bool GridSearch::GenerateDpMap(
    const double ex, const double ey, const std::vector<double> &XYbounds,
    const std::vector<std::vector<math::LineSegment2d>> &obstacles_linesegments_vec) {
    std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>,
                        cmp>
        open_pq;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
    dp_map_ = decltype(dp_map_)();
    XYbounds_ = XYbounds;
    max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
    max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);
    std::shared_ptr<Node2d> end_node =
        std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
    obstacles_linesegments_vec_ = obstacles_linesegments_vec;
    open_set.emplace(end_node->GetIndex(), end_node);
    open_pq.emplace(end_node->GetIndex(), end_node->GetCost());

    size_t explored_node_num = 0;
    while (!open_pq.empty()) {
        const std::string current_id = open_pq.top().first;
        open_pq.pop();
        std::shared_ptr<Node2d> current_node = open_set[current_id];
        dp_map_.emplace(current_node->GetIndex(), current_node);
        std::vector<std::shared_ptr<Node2d>> next_nodes =
            std::move(GenerateNextNodes(current_node));
        for (auto &next_node : next_nodes) {
            if (!CheckConstraints(next_node)) {
                continue;
            }
            if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
                continue;
            }
            if (open_set.find(next_node->GetIndex()) == open_set.end()) {
                ++explored_node_num;
                next_node->SetPreNode(current_node);
                open_set.emplace(next_node->GetIndex(), next_node);
                open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
            } else {
                if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
                    open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
                    open_set[next_node->GetIndex()]->SetPreNode(current_node);
                }
            }
        }
    }
    return true;
}

double GridSearch::CheckDpMap(const double sx, const double sy) {
    std::string index = Node2d::CalcIndex(sx, sy, xy_grid_resolution_, XYbounds_);
    if (dp_map_.find(index) != dp_map_.end()) {
        return dp_map_[index]->GetCost() * xy_grid_resolution_;
    } else {
        return std::numeric_limits<double>::infinity();
    }
}

void GridSearch::LoadGridAStarResult(GridAStarResult *result) {
    (*result).path_cost = final_node_->GetPathCost() * xy_grid_resolution_;
    std::shared_ptr<Node2d> current_node = final_node_;
    std::vector<double> grid_a_x;
    std::vector<double> grid_a_y;
    while (current_node->GetPreNode() != nullptr) {
        grid_a_x.push_back(current_node->GetGridX() * xy_grid_resolution_ + XYbounds_[0]);
        grid_a_y.push_back(current_node->GetGridY() * xy_grid_resolution_ + XYbounds_[2]);
        current_node = current_node->GetPreNode();
    }
    std::reverse(grid_a_x.begin(), grid_a_x.end());
    std::reverse(grid_a_y.begin(), grid_a_y.end());
    (*result).x = std::move(grid_a_x);
    (*result).y = std::move(grid_a_y);
}
}  // namespace planning
