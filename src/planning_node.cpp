
#include "planning/planning.h"
#include "ros/ros.h"

#include <iostream>

int main(int argc, char **argv) {
    std::cout << "hello" << std::endl;
    ros::init(argc, argv, "planning_test");
    ros::NodeHandle nh;

    std::vector<std::vector<float>> points;
    std::vector<float> center;
    center.push_back(10);
    center.push_back(10);
    std::vector<float> dx = {2, 10, 2, 10};
    std::vector<float> dy = {2, 2, -2, -2};
    for (int i = 0; i < 4; ++i) {
        std::vector<float> point;
        point.push_back(center.front() + dx[i]);
        point.push_back(center.front() + dy[i]);
        points.push_back(point);
    }

    polygonShow poly_show(nh, points);
    poly_show.run();

    return 0;
}
