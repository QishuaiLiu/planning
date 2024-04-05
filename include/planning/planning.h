
#pragma once
#include <vector>
#include "geometry_msgs/PolygonStamped.h"
#include "hybrid_a_star/grid_search.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "planning/common_def.h"
#include "ros/ros.h"

class polygonShow {
   public:
    polygonShow(ros::NodeHandle &nh, const std::vector<obstacle::BoundingBox> &points);
    void setMapBoundary(const obstacle::BoundingBox &map_boundary);
    void createObstaclePolygon(jsk_recognition_msgs::BoundingBoxArray &bound_boxes_msgs);

    void createMapBoundary(jsk_recognition_msgs::BoundingBox &map_msgs);

    void setStartAndEndPose(const std::vector<obstacle::BoundingBox> &start_end_pose);

    void showStartEndPose(jsk_recognition_msgs::BoundingBoxArray &start_end_pose);

    void checkBoundBox(geometry_msgs::PolygonStamped &check_points,
                       const jsk_recognition_msgs::BoundingBoxArray &bound_boxes_msgs);

    bool generateAstarPath(planning::GridAStarResult &astar_path);

    void run();

   private:
    // visualization related
    std::vector<obstacle::BoundingBox> polygon_points_;
    std::vector<obstacle::BoundingBox> start_and_end_;
    obstacle::BoundingBox map_boundary_;
    ros::NodeHandle nh_;
    ros::Publisher poly_points_pub_;
    ros::Publisher map_boundary_pub_;
    ros::Publisher start_and_end_pub_;
    ros::Publisher check_pub_;

    // grid search
    std::unique_ptr<planning::GridSearch> grid_search_ptr_;
};
