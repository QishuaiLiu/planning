#pragma once
#include <map>
#include <string>
#include <vector>
#include "math/Vec2d.h"

namespace planning {

class PrintPoints {
   public:
    PrintPoints() {}
    explicit PrintPoints(std::string id) : id_(id) {}
    /**
     * @brief set curve id
     */
    void set_id(std::string id);
    /**
     * @brief add points to curve
     */
    void AddPoint(double x, double y);
    /**
     * @brief print curve to log
     */
    void PrintToLog();

   private:
    std::string id_;
    std::vector<std::pair<double, double>> points;
};

class PrintCurves {
   public:
    /**
     * @brief add point to curve key
     */
    void AddPoint(std::string key, double x, double y);
    void AddPoint(std::string key, const math::Vec2d &point);
    void AddPoint(std::string key, const std::vector<math::Vec2d> &points);
    void PrintToLog();

   private:
    std::map<std::string, PrintPoints> curve_map_;
};

}  // namespace planning
