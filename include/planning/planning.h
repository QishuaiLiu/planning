
#pragma once
#include <vector>
#include "geometry_msgs/PolygonStamped.h"
#include "ros/ros.h"

class polygonShow {
   public:
    polygonShow(ros::NodeHandle &nh, const std::vector<std::vector<float>> &points);

    void createPolygon(geometry_msgs::PolygonStamped &poly_msg);

    void run();

   private:
    std::vector<std::vector<float>> polygon_points_;
    ros::NodeHandle nh_;
    ros::Publisher poly_points_pub_;
};
