
#pragma once
#include <vector>
#include "geometry_msgs/PolygonStamped.h"
#include "ros/ros.h"

struct Point2d {
    float x;
    float y;
    float theta;
};

class polygonShow {
   public:
    polygonShow(ros::NodeHandle &nh, const std::vector<Point2d> &points);

    void createObstaclePolygon(geometry_msgs::PolygonStamped &poly_msg);

    void run();

   private:
    std::vector<Point2d> polygon_points_;
    ros::NodeHandle nh_;
    ros::Publisher poly_points_pub_;
};
