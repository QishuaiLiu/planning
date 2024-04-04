
#include "planning/planning.h"
#include "ros/ros.h"

#include <iostream>

std::vector<obstacle::BoundingBox> createObstacle() {
    std::vector<obstacle::BoundingBox> bound_boxs;
    std::vector<obstacle::Point2d> centers;
    std::vector<float> length = {7, 10};
    std::vector<float> width = {4, 5};
    const int size = length.size();
    bound_boxs.resize(size);
    centers.resize(size);

    for (int i = 0; i < size; ++i) {
        centers[i].x = 10 + i * 15;
        centers[i].y = 8 + i * 15;
        centers[i].theta = 0 + i * M_PIf / 2;
        bound_boxs[i].center = centers[i];
        bound_boxs[i].length = length[i];
        bound_boxs[i].width = width[i];
    }
    return bound_boxs;
}

obstacle::BoundingBox createMapBound() {
    obstacle::BoundingBox map;
    obstacle::Point2d center;
    center.x = 15, center.y = 15, center.theta = 0;
    map.center = center;
    map.length = 30, map.width = 30;
    return map;
}

int main(int argc, char **argv) {
    std::cout << "hello" << std::endl;
    ros::init(argc, argv, "planning_test");
    ros::NodeHandle nh;

    std::vector<obstacle::BoundingBox> bound_boxs = createObstacle();
    const obstacle::BoundingBox map_boundary = createMapBound();

    polygonShow poly_show(nh, bound_boxs);
    poly_show.setMapBoundary(map_boundary);

    poly_show.run();

    return 0;
}
