#include "planning/planning.h"

polygonShow::polygonShow(ros::NodeHandle &nh, const std::vector<std::vector<float>> &points)
    : polygon_points_(points), nh_(nh) {
    poly_points_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("poly_points", 1, false);
}

void polygonShow::createPolygon(geometry_msgs::PolygonStamped &poly_msg) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    poly_msg.header = header;
    geometry_msgs::Polygon poly;
    int size_num = polygon_points_.size();
    poly.points.resize(size_num);

    for (int i = 0; i < size_num; ++i) {
        auto &point = polygon_points_[i];
        poly.points[i].x = point[0];
        poly.points[i].y = point[1];
        poly.points[i].z = 0;
    }
    poly_msg.polygon = poly;
}

void polygonShow::run() {
    geometry_msgs::PolygonStamped poly_msg;
    ros::Rate r(10);

    createPolygon(poly_msg);

    int count = 0;
    while (ros::ok()) {
        r.sleep();
        poly_points_pub_.publish(poly_msg);
    }
}
