#include "common/print_debug_info.h"
#include <iostream>
#include <sstream>

bool FLAGS_enable_print_curve = true;

namespace planning {

void PrintPoints::set_id(std::string id) {
    id_ = id;
}
void PrintPoints::AddPoint(double x, double y) {
    points.emplace_back(x, y);
}

void PrintPoints::PrintToLog() {
    if (!FLAGS_enable_print_curve) {
        return;
    }
    std::stringstream ssm;
    ssm << "print_" << id_ << ":";
    for (size_t i = 0; i < points.size(); i++) {
        ssm << std::fixed << "(" << points[i].first << ", " << points[i].second << ");";
    }
    std::cout << ssm.str();
}

void PrintCurves::AddPoint(std::string key, double x, double y) {
    if (!FLAGS_enable_print_curve) {
        return;
    }
    if (curve_map_.count(key) == 0) {
        curve_map_[key] = PrintPoints(key);
    }
    curve_map_[key].AddPoint(x, y);
}

void PrintCurves::AddPoint(std::string key, const math::Vec2d &point) {
    if (!FLAGS_enable_print_curve) {
        return;
    }
    if (curve_map_.count(key) == 0) {
        curve_map_[key] = PrintPoints(key);
    }
    curve_map_[key].AddPoint(point.x(), point.y());
}

void PrintCurves::AddPoint(std::string key, const std::vector<math::Vec2d> &points) {
    if (!FLAGS_enable_print_curve) {
        return;
    }
    for (const auto &point : points) {
        AddPoint(key, point);
    }
}
void PrintCurves::PrintToLog() {
    if (!FLAGS_enable_print_curve) {
        return;
    }
    for (auto iter = curve_map_.begin(); iter != curve_map_.end(); iter++) {
        iter->second.PrintToLog();
    }
}

}  // namespace planning
