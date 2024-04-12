#include "planning/planning.h"
#include "tf/tf.h"

polygonShow::polygonShow(ros::NodeHandle &nh, const std::vector<obstacle::BoundingBox> &points)
    : polygon_points_(points), nh_(nh) {
    // setting publisher
    poly_points_pub_ =
        nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("poly_points", 1, false);
    map_boundary_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBox>("map_boundary", 1, false);
    start_and_end_pub_ =
        nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("start_and_end", 1, false);
    check_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("check_point", 1, false);
    astar_path_pub_ = nh_.advertise<nav_msgs::Path>("astar_path", 1, false);

    // config planner
    planning::PlannerOpenSpaceConfig open_space_config;
    grid_search_ptr_ = std::make_unique<planning::GridSearch>(open_space_config);
    hybrid_astart_ptr_ = std::make_unique<planning::HybridAStar>(open_space_config);
}

void polygonShow::setMapBoundary(const obstacle::BoundingBox &map_boundary) {
    map_boundary_ = map_boundary;
}

void polygonShow::setStartAndEndPose(const std::vector<obstacle::BoundingBox> &start_end_pose) {
    start_and_end_ = start_end_pose;
    return;
}

void polygonShow::showStartEndPose(jsk_recognition_msgs::BoundingBoxArray &start_end_pose) {
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();

    start_end_pose.header = header;
    start_end_pose.boxes.resize(2);
    auto &boxes = start_end_pose.boxes;

    for (int i = 0; i < 2; ++i) {
        const auto &cur_pose = start_and_end_[i];
        boxes[i].header = header;
        boxes[i].pose.position.x = cur_pose.center.x;
        boxes[i].pose.position.y = cur_pose.center.y;
        boxes[i].pose.position.z = 0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, cur_pose.center.theta);
        quat = quat.normalize();

        boxes[i].pose.orientation.x = quat.x();
        boxes[i].pose.orientation.y = quat.y();
        boxes[i].pose.orientation.z = quat.z();
        boxes[i].pose.orientation.w = quat.w();

        boxes[i].dimensions.x = cur_pose.length;
        boxes[i].dimensions.y = cur_pose.width;
        boxes[i].dimensions.z = 0.3;
    }
    return;
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

// only for checking whether the points are in correct position
void polygonShow::checkBoundBox(geometry_msgs::PolygonStamped &check_points,
                                const jsk_recognition_msgs::BoundingBoxArray &bound_boxes_msgs) {
    auto &last_obstacle = bound_boxes_msgs.boxes.back();
    std::vector<float> center_point;
    center_point.resize(2);
    center_point.front() = last_obstacle.pose.position.x;
    center_point.back() = last_obstacle.pose.position.y;
    tf::Quaternion q(last_obstacle.pose.orientation.x, last_obstacle.pose.orientation.y,
                     last_obstacle.pose.orientation.z, last_obstacle.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    const float length = last_obstacle.dimensions.x;
    const float width = last_obstacle.dimensions.y;

    std::vector<std::vector<float>> corner_points;
    std::vector<float> dir_x = {-1, 1, 1, -1};
    std::vector<float> dir_y = {1, 1, -1, -1};
    corner_points.resize(4);
    for (int i = 0; i < 4; ++i) {
        const float x = center_point[0];
        const float y = center_point[1];
        const float dx = length / 2 * dir_x[i];
        const float dy = width / 2 * dir_y[i];
        const float temp_x = dx * std::cos(yaw) - dy * std::sin(yaw);
        const float temp_y = dx * std::sin(yaw) + dy * std::cos(yaw);
        const std::vector<float> corner_point = {x + temp_x, y + temp_y};
        corner_points[i] = corner_point;
    }

    check_points.header = bound_boxes_msgs.header;
    auto &points = check_points.polygon.points;
    points.resize(4);
    for (int i = 0; i < 4; ++i) {
        points[i].x = corner_points[i][0];
        points[i].y = corner_points[i][1];
        points[i].z = 0.3;
    }

    return;
}

bool polygonShow::generateAstarPath(planning::GridAStarResult &astar_path) {
    // create map boundary for grid search
    std::vector<double> xy_bounds;
    xy_bounds.resize(4);
    auto &center = map_boundary_.center;
    const double length = map_boundary_.length;
    const double width = map_boundary_.width;
    const double theta = center.theta;
    std::vector<double> dx = {-length / 2, length / 2};
    std::vector<double> dy = {-width / 2, width / 2};
    for (int i = 0; i < 2; ++i) {
        const double temp_x = dx[i] * std::cos(theta) - dy[i] * std::sin(theta);
        const double temp_y = dx[i] * std::sin(theta) + dy[i] * std::cos(theta);
        xy_bounds[i] = temp_x + center.x;
        xy_bounds[i + 2] = temp_y + center.y;
    }

    // create obstacle line segment2d
    std::vector<std::vector<math::LineSegment2d>> obstacles_linesegments_vec;
    int obstacle_size = polygon_points_.size();
    obstacles_linesegments_vec.resize(obstacle_size);
    for (int i = 0; i < obstacle_size; ++i) {
        std::vector<math::LineSegment2d> obstacle_linesegments;
        const double length = polygon_points_[i].length;
        const double width = polygon_points_[i].width;
        auto &center = polygon_points_[i].center;
        const double theta = center.theta;
        std::vector<double> dx = {-length / 2, -length / 2, length / 2, length / 2};
        std::vector<double> dy = {width / 2, -width / 2, -width / 2, width / 2};
        for (int j = 0; j < 4; ++j) {
            const double temp_pre_x = dx[j % 4] * std::cos(theta) - dy[j % 4] * std::sin(theta);
            const double temp_pre_y = dx[j % 4] * std::sin(theta) + dy[j % 4] * std::cos(theta);
            math::Vec2d start(center.x + temp_pre_x, center.y + temp_pre_y);

            const double temp_next_x =
                dx[(j + 1) % 4] * std::cos(theta) - dy[(j + 1) % 4] * std::sin(theta);
            const double temp_next_y =
                dx[(j + 1) % 4] * std::sin(theta) + dy[(j + 1) % 4] * std::cos(theta);
            math::Vec2d end(center.x + temp_next_x, center.y + temp_next_y);

            math::LineSegment2d obstacle_line = math::LineSegment2d(start, end);
            obstacle_linesegments.push_back(obstacle_line);
        }
        obstacles_linesegments_vec[i] = obstacle_linesegments;
    }

    // prepare the start and end pose
    auto &start = start_and_end_.front();
    auto &end = start_and_end_.back();
    const double sx = start.center.x, sy = start.center.y;
    const double ex = end.center.x, ey = end.center.y;

    bool plan_result = grid_search_ptr_->GenerateAStarPath(sx, sy, ex, ey, xy_bounds,
                                                           obstacles_linesegments_vec, &astar_path);
    std::cout << "plan result is: " << plan_result << std::endl;
    if (plan_result) {
        std::cout << "path size: " << astar_path.x.size() << std::endl;
    }

    return true;
}
void polygonShow::showAstarPath(const planning::GridAStarResult &astar_path,
                                nav_msgs::Path &pose_msgs) {
    int size = astar_path.x.size();
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();
    pose_msgs.header = header;

    pose_msgs.poses.resize(size);
    for (int i = 0; i < size; ++i) {
        auto &pose = pose_msgs.poses[i];
        pose.header = header;
        pose.pose.position.x = astar_path.x[i];
        pose.pose.position.y = astar_path.y[i];
        pose.pose.position.z = 0.3;

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
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

    // verify the map boundary is correct
    // jsk_recognition_msgs::BoundingBoxArray map_boundary_array;
    // map_boundary_array.boxes.push_back(map_boundary);
    // map_boundary_array.header = map_boundary.header;

    jsk_recognition_msgs::BoundingBoxArray start_end_msg;
    showStartEndPose(start_end_msg);

    geometry_msgs::PolygonStamped check_points;
    checkBoundBox(check_points, poly_msg);

    planning::GridAStarResult astar_path;

    generateAstarPath(astar_path);

    nav_msgs::Path astar_pose;
    showAstarPath(astar_path, astar_pose);

    int count = 0;
    while (ros::ok()) {
        r.sleep();
        poly_points_pub_.publish(poly_msg);
        map_boundary_pub_.publish(map_boundary);
        start_and_end_pub_.publish(start_end_msg);
        check_pub_.publish(check_points);
        astar_path_pub_.publish(astar_pose);
    }
}
