#include "planning/planning.h"

polygonShow::polygonShow(ros::NodeHandle &nh, const std::vector<obstacle::BoundingBox> &points)
    : polygon_points_(points), nh_(nh) {
    poly_points_pub_ =
        nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("poly_points", 1, false);
    map_boundary_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBox>("map_boundary", 1, false);
}

void polygonShow::setMapBoundary(const obstacle::BoundingBox &map_boundary) {
    map_boundary_ = map_boundary;
}

void polygonShow::createObstaclePolygon(jsk_recognition_msgs::BoundingBoxArray &bound_box_msgs) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    bound_box_msgs.header = header;
    int size = polygon_points_.size();
    bound_box_msgs.boxes.resize(size);

    for (int i = 0; i < size; ++i) {
        auto &box = bound_box_msgs.boxes[i];
        box.header.frame_id = "map";
        box.header.stamp = ros::Time::now();
        box.pose.position.x = polygon_points_[i].center.x;
        box.pose.position.y = polygon_points_[i].center.y;
        box.pose.position.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, polygon_points_[i].center.theta);
        quat = quat.normalize();

        box.pose.orientation.x = quat.x();
        box.pose.orientation.y = quat.y();
        box.pose.orientation.z = quat.z();
        box.pose.orientation.w = quat.w();

        box.dimensions.x = polygon_points_[i].length;
        box.dimensions.y = polygon_points_[i].width;
        box.dimensions.z = 0.3;
    }
    return;
}
void polygonShow::createMapBoundary(jsk_recognition_msgs::BoundingBox &map_msgs) {
    map_msgs.header.frame_id = "map";
    map_msgs.header.stamp = ros::Time::now();
    map_msgs.pose.position.x = map_boundary_.center.x;
    map_msgs.pose.position.y = map_boundary_.center.y;
    map_msgs.pose.position.z = 0;

    map_msgs.pose.orientation.x = 0;
    map_msgs.pose.orientation.x = 0;
    map_msgs.pose.orientation.x = 0;
    map_msgs.pose.orientation.w = 1;

    map_msgs.dimensions.x = map_boundary_.length;
    map_msgs.dimensions.y = map_boundary_.width;
    map_msgs.dimensions.z = 0.3;
    return;
}

void polygonShow::run() {
    jsk_recognition_msgs::BoundingBoxArray poly_msg;
    jsk_recognition_msgs::BoundingBox map_boundary;
    ros::Rate r(10);

    createMapBoundary(map_boundary);
    createObstaclePolygon(poly_msg);

    int count = 0;
    while (ros::ok()) {
        r.sleep();
        poly_points_pub_.publish(poly_msg);
        map_boundary_pub_.publish(map_boundary);
    }
}
