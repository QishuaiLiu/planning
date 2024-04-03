
#include "planning/planning.h"
#include "ros/ros.h"

#include <iostream>

std::vector<Point2d> createObstacle() {
    std::vector<Point2d> points;
    std::vector<float> center;
    center.push_back(10);
    center.push_back(10);
    std::vector<float> dx = {2, 10, 10, 2};
    std::vector<float> dy = {2, 2, -2, -2};
    for (int i = 0; i < 4; ++i) {
        Point2d point;
        point.x = center.front() + dx[i];
        point.y = center.back() + dy[i];
        point.theta = 0;
        points.push_back(point);
    }
    return points;
}

int main(int argc, char **argv) {
    std::cout << "hello" << std::endl;
    ros::init(argc, argv, "planning_test");
    ros::NodeHandle nh;

    std::vector<Point2d> points = createObstacle();

    polygonShow poly_show(nh, points);
    poly_show.run();

    return 0;
}
