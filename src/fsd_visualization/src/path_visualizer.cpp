#include "fsd_visualization/path_visualizer.hpp"

namespace fsd_viz {

PathVisualizer::PathVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // 参数
    pnh.param<std::string>("frame_id", frame_id_, FRAME_GLOBAL);
    pnh.param<double>("path_width", path_width_, PATH_WIDTH);
    pnh.param<double>("point_size", point_size_, PATH_POINT_SIZE);
    pnh.param<bool>("compat/enable_legacy_partial_full", enable_legacy_partial_full_, false);
    pnh.param<std::string>("topics/pathlimits", unified_path_topic_, "planning/pathlimits");
    pnh.param<std::string>("topics/path_partial", partial_path_topic_, "planning/high_speed_tracking/pathlimits/partial");
    pnh.param<std::string>("topics/path_full", full_path_topic_, "planning/high_speed_tracking/pathlimits/full");
    pnh.param<std::string>("topics/nav_path", nav_path_topic_, "planning/line_detection/path");
    pnh.param<std::string>("topics/markers", markers_topic_, "fsd/viz/path");
    pnh.param<std::string>("topics/boundaries", boundaries_topic_, "fsd/viz/boundaries");
    
    // 订阅
    sub_path_unified_ = nh.subscribe(unified_path_topic_, 1,
        &PathVisualizer::pathLimitsUnifiedCallback, this);
    if (enable_legacy_partial_full_) {
        sub_path_partial_ = nh.subscribe(partial_path_topic_, 1,
            &PathVisualizer::pathLimitsPartialCallback, this);
        sub_path_full_ = nh.subscribe(full_path_topic_, 1,
            &PathVisualizer::pathLimitsFullCallback, this);
        ROS_WARN("[PathVisualizer] Legacy partial/full compatibility mode enabled.");
    }
    sub_nav_path_ = nh.subscribe(nav_path_topic_, 1,
        &PathVisualizer::navPathCallback, this);
    
    // 发布
    // Latch latest path/boundary markers so RViz can render immediately.
    pub_path_markers_ = nh.advertise<visualization_msgs::MarkerArray>(markers_topic_, 1, true);
    pub_boundary_markers_ = nh.advertise<visualization_msgs::MarkerArray>(boundaries_topic_, 1, true);
    
    ROS_INFO("[PathVisualizer] Initialized");
}

void PathVisualizer::pathLimitsUnifiedCallback(
    const autodrive_msgs::HUAT_PathLimits::ConstPtr& msg) {

    auto markers = createPathMarkers(*msg, "path_unified", PATH_CENTER);
    auto boundary_markers = createBoundaryMarkers(*msg);

    pub_path_markers_.publish(markers);
    pub_boundary_markers_.publish(boundary_markers);
}

void PathVisualizer::pathLimitsPartialCallback(
    const autodrive_msgs::HUAT_PathLimits::ConstPtr& msg) {
    
    auto markers = createPathMarkers(*msg, "path_partial", PATH_PARTIAL);
    auto boundary_markers = createBoundaryMarkers(*msg);
    
    pub_path_markers_.publish(markers);
    pub_boundary_markers_.publish(boundary_markers);
}

void PathVisualizer::pathLimitsFullCallback(
    const autodrive_msgs::HUAT_PathLimits::ConstPtr& msg) {
    
    auto markers = createPathMarkers(*msg, "path_full", PATH_FULL);
    pub_path_markers_.publish(markers);
}

void PathVisualizer::navPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    visualization_msgs::MarkerArray markers;
    
    // 路径线条
    visualization_msgs::Marker line_marker;
    line_marker.header = msg->header;
    line_marker.ns = "nav_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = path_width_;
    line_marker.color.r = PATH_CENTER[0];
    line_marker.color.g = PATH_CENTER[1];
    line_marker.color.b = PATH_CENTER[2];
    line_marker.color.a = PATH_CENTER[3];
    line_marker.pose.orientation.w = 1.0;
    
    for (const auto& pose : msg->poses) {
        line_marker.points.push_back(pose.pose.position);
    }
    
    markers.markers.push_back(line_marker);
    pub_path_markers_.publish(markers);
}

visualization_msgs::MarkerArray PathVisualizer::createPathMarkers(
    const autodrive_msgs::HUAT_PathLimits& msg,
    const std::string& ns,
    const std::array<float, 4>& color) {
    
    visualization_msgs::MarkerArray markers;
    
    // 中心路径线条 - 使用 path 数组
    visualization_msgs::Marker center_line;
    center_line.header = msg.header;
    center_line.header.frame_id = frame_id_;
    center_line.ns = ns + "_center";
    center_line.id = 0;
    center_line.type = visualization_msgs::Marker::LINE_STRIP;
    center_line.action = visualization_msgs::Marker::ADD;
    center_line.scale.x = path_width_;
    center_line.color.r = color[0];
    center_line.color.g = color[1];
    center_line.color.b = color[2];
    center_line.color.a = color[3];
    center_line.pose.orientation.w = 1.0;
    center_line.lifetime = ros::Duration(0.2);
    
    for (const auto& pt : msg.path) {
        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.05;
        center_line.points.push_back(p);
    }
    
    if (!center_line.points.empty()) {
        markers.markers.push_back(center_line);
    }
    
    // 路径点（球体）
    visualization_msgs::Marker points;
    points.header = msg.header;
    points.header.frame_id = frame_id_;
    points.ns = ns + "_points";
    points.id = 1;
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    points.scale.x = point_size_;
    points.scale.y = point_size_;
    points.scale.z = point_size_;
    points.color.r = color[0];
    points.color.g = color[1];
    points.color.b = color[2];
    points.color.a = color[3];
    points.pose.orientation.w = 1.0;
    points.lifetime = ros::Duration(0.2);
    
    for (const auto& pt : msg.path) {
        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.05;
        points.points.push_back(p);
    }
    
    if (!points.points.empty()) {
        markers.markers.push_back(points);
    }
    
    return markers;
}

visualization_msgs::MarkerArray PathVisualizer::createBoundaryMarkers(
    const autodrive_msgs::HUAT_PathLimits& msg) {
    
    visualization_msgs::MarkerArray markers;
    
    // 左边界 - 使用 tracklimits.left 数组
    visualization_msgs::Marker left_line;
    left_line.header = msg.header;
    left_line.header.frame_id = frame_id_;
    left_line.ns = "boundary_left";
    left_line.id = 0;
    left_line.type = visualization_msgs::Marker::LINE_STRIP;
    left_line.action = visualization_msgs::Marker::ADD;
    left_line.scale.x = path_width_ * 0.5;
    left_line.color.r = BOUNDARY_LEFT[0];
    left_line.color.g = BOUNDARY_LEFT[1];
    left_line.color.b = BOUNDARY_LEFT[2];
    left_line.color.a = BOUNDARY_LEFT[3];
    left_line.pose.orientation.w = 1.0;
    left_line.lifetime = ros::Duration(0.2);
    
    for (const auto& cone : msg.tracklimits.left) {
        geometry_msgs::Point p;
        p.x = cone.position_global.x;
        p.y = cone.position_global.y;
        p.z = 0.02;
        left_line.points.push_back(p);
    }
    
    if (!left_line.points.empty()) {
        markers.markers.push_back(left_line);
    }
    
    // 右边界 - 使用 tracklimits.right 数组
    visualization_msgs::Marker right_line;
    right_line.header = msg.header;
    right_line.header.frame_id = frame_id_;
    right_line.ns = "boundary_right";
    right_line.id = 1;
    right_line.type = visualization_msgs::Marker::LINE_STRIP;
    right_line.action = visualization_msgs::Marker::ADD;
    right_line.scale.x = path_width_ * 0.5;
    right_line.color.r = BOUNDARY_RIGHT[0];
    right_line.color.g = BOUNDARY_RIGHT[1];
    right_line.color.b = BOUNDARY_RIGHT[2];
    right_line.color.a = BOUNDARY_RIGHT[3];
    right_line.pose.orientation.w = 1.0;
    right_line.lifetime = ros::Duration(0.2);
    
    for (const auto& cone : msg.tracklimits.right) {
        geometry_msgs::Point p;
        p.x = cone.position_global.x;
        p.y = cone.position_global.y;
        p.z = 0.02;
        right_line.points.push_back(p);
    }
    
    if (!right_line.points.empty()) {
        markers.markers.push_back(right_line);
    }
    
    return markers;
}

}  // namespace fsd_viz
