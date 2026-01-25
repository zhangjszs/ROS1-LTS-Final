#include "fsd_visualization/cone_visualizer.hpp"

namespace fsd_viz {

ConeVisualizer::ConeVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // 参数
    pnh.param<std::string>("frame_id", frame_id_, FRAME_GLOBAL);
    pnh.param<double>("cone_radius", cone_radius_, CONE_RADIUS);
    pnh.param<double>("cone_height", cone_height_, CONE_HEIGHT);
    pnh.param<std::string>("topics/cone_detections", cone_detections_topic_, "perception/lidar_cluster/detections");
    pnh.param<std::string>("topics/cone_map", cone_map_topic_, "localization/cone_map");
    pnh.param<std::string>("topics/markers", markers_topic_, "fsd/viz/cones");
    
    // 订阅
    sub_cone_detections_ = nh.subscribe(cone_detections_topic_, 1, 
        &ConeVisualizer::coneDetectionsCallback, this);
    sub_cone_map_ = nh.subscribe(cone_map_topic_, 1,
        &ConeVisualizer::coneMapCallback, this);
    
    // 发布
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(markers_topic_, 1);
    
    ROS_INFO("[ConeVisualizer] Initialized");
}

void ConeVisualizer::coneDetectionsCallback(
    const autodrive_msgs::HUAT_ConeDetections::ConstPtr& msg) {
    
    visualization_msgs::MarkerArray markers;
    
    // 先清除旧的 markers
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.header.frame_id = frame_id_;
    delete_marker.header.stamp = msg->header.stamp;
    markers.markers.push_back(delete_marker);
    
    // HUAT_ConeDetections 使用 points 数组和 color 字符串
    // 解析 color 字符串确定颜色类型
    int color_type = static_cast<int>(ConeType::UNKNOWN);
    if (!msg->color.data.empty()) {
        char c = msg->color.data[0];
        if (c == 'b' || c == 'B') color_type = static_cast<int>(ConeType::BLUE);
        else if (c == 'y' || c == 'Y') color_type = static_cast<int>(ConeType::YELLOW);
        else if (c == 'r' || c == 'R' || c == 'o' || c == 'O') 
            color_type = static_cast<int>(ConeType::ORANGE);
    }
    
    // 创建新的锥桶 markers
    for (size_t i = 0; i < msg->points.size(); ++i) {
        const auto& point = msg->points[i];
        auto marker = createConeMarker(
            point.x, point.y, point.z,
            static_cast<int>(i), color_type, "cone_detections");
        marker.header.stamp = msg->header.stamp;
        markers.markers.push_back(marker);
    }
    
    pub_markers_.publish(markers);
}

void ConeVisualizer::coneMapCallback(
    const autodrive_msgs::HUAT_ConeMap::ConstPtr& msg) {
    
    visualization_msgs::MarkerArray markers;
    
    // 先清除旧的 markers
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.header.frame_id = frame_id_;
    delete_marker.header.stamp = msg->header.stamp;
    delete_marker.ns = "cone_map";
    markers.markers.push_back(delete_marker);
    
    // 创建锥桶 markers - 使用 cone 数组和 position_global
    for (size_t i = 0; i < msg->cone.size(); ++i) {
        const auto& cone = msg->cone[i];
        auto marker = createConeMarker(
            cone.position_global.x, cone.position_global.y, cone.position_global.z,
            static_cast<int>(i), static_cast<int>(cone.type), "cone_map");
        marker.header.stamp = msg->header.stamp;
        markers.markers.push_back(marker);
    }
    
    pub_markers_.publish(markers);
}

visualization_msgs::Marker ConeVisualizer::createConeMarker(
    double x, double y, double z,
    int id, int type, const std::string& ns) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 位置（锥桶底部在地面）
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z + cone_height_ / 2.0;
    marker.pose.orientation.w = 1.0;
    
    // 尺寸（锥形近似为圆柱）
    marker.scale.x = cone_radius_ * 2.0;
    marker.scale.y = cone_radius_ * 2.0;
    marker.scale.z = cone_height_;
    
    // 颜色
    auto color = getConeColor(type);
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    
    marker.lifetime = ros::Duration(0.2);  // 200ms 超时
    
    return marker;
}

}  // namespace fsd_viz
