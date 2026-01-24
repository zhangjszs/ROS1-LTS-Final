#include "fsd_visualization/vehicle_visualizer.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace fsd_viz {

VehicleVisualizer::VehicleVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // 参数
    pnh.param<std::string>("frame_id", frame_id_, FRAME_GLOBAL);
    pnh.param<int>("trail_length", trail_max_size_, 200);
    pnh.param<bool>("show_trail", show_trail_, true);
    
    // 订阅
    sub_car_state_ = nh.subscribe("/Carstate", 1,
        &VehicleVisualizer::carStateCallback, this);
    
    // 发布
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/fsd/viz/vehicle", 1);
    
    ROS_INFO("[VehicleVisualizer] Initialized, trail_length=%d", trail_max_size_);
}

void VehicleVisualizer::carStateCallback(
    const autodrive_msgs::HUAT_CarState::ConstPtr& msg) {
    
    visualization_msgs::MarkerArray markers;
    
    // 获取车辆状态 (car_state 是 geometry_msgs::Pose2D)
    const auto& state = msg->car_state;
    
    // 更新轨迹
    if (show_trail_) {
        geometry_msgs::Point trail_point;
        trail_point.x = state.x;
        trail_point.y = state.y;
        trail_point.z = 0.01;  // 略高于地面
        
        // 只在移动时添加点
        if (trail_points_.empty() || 
            std::hypot(trail_point.x - trail_points_.back().x,
                       trail_point.y - trail_points_.back().y) > 0.1) {
            trail_points_.push_back(trail_point);
            
            // 限制轨迹长度
            while (static_cast<int>(trail_points_.size()) > trail_max_size_) {
                trail_points_.pop_front();
            }
        }
    }
    
    // 车身
    auto body = createBodyMarker(state);
    body.header.stamp = msg->header.stamp;
    markers.markers.push_back(body);
    
    // 航向箭头
    auto arrow = createArrowMarker(state);
    arrow.header.stamp = msg->header.stamp;
    markers.markers.push_back(arrow);
    
    // 四个车轮
    auto wheels = createWheelMarkers(state);
    for (auto& wheel : wheels) {
        wheel.header.stamp = msg->header.stamp;
        markers.markers.push_back(wheel);
    }
    
    // 轨迹
    if (show_trail_ && !trail_points_.empty()) {
        auto trail = createTrailMarker(msg->header.stamp);
        markers.markers.push_back(trail);
    }
    
    pub_markers_.publish(markers);
}

visualization_msgs::Marker VehicleVisualizer::createBodyMarker(
    const geometry_msgs::Pose2D& state) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 位置
    marker.pose.position.x = state.x;
    marker.pose.position.y = state.y;
    marker.pose.position.z = VEHICLE_HEIGHT / 2.0;
    
    // 朝向 - 使用 theta 字段
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    // 尺寸
    marker.scale.x = VEHICLE_LENGTH;
    marker.scale.y = VEHICLE_WIDTH;
    marker.scale.z = VEHICLE_HEIGHT;
    
    // 颜色
    marker.color.r = VEHICLE_BODY[0];
    marker.color.g = VEHICLE_BODY[1];
    marker.color.b = VEHICLE_BODY[2];
    marker.color.a = VEHICLE_BODY[3];
    
    marker.lifetime = ros::Duration(0.1);
    
    return marker;
}

visualization_msgs::Marker VehicleVisualizer::createArrowMarker(
    const geometry_msgs::Pose2D& state) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "vehicle_arrow";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 位置
    marker.pose.position.x = state.x;
    marker.pose.position.y = state.y;
    marker.pose.position.z = VEHICLE_HEIGHT + 0.1;
    
    // 朝向 - 使用 theta 字段
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    // 尺寸：长度、宽度、高度
    marker.scale.x = VEHICLE_LENGTH * 0.8;  // 箭头长度
    marker.scale.y = 0.15;  // 箭身宽度
    marker.scale.z = 0.15;  // 箭身高度
    
    // 颜色
    marker.color.r = VEHICLE_ARROW[0];
    marker.color.g = VEHICLE_ARROW[1];
    marker.color.b = VEHICLE_ARROW[2];
    marker.color.a = VEHICLE_ARROW[3];
    
    marker.lifetime = ros::Duration(0.1);
    
    return marker;
}

std::vector<visualization_msgs::Marker> VehicleVisualizer::createWheelMarkers(
    const geometry_msgs::Pose2D& state) {
    
    std::vector<visualization_msgs::Marker> wheels;
    
    // 车轮位置（相对于车辆中心）
    const double half_length = VEHICLE_LENGTH / 2.0 - 0.3;
    const double half_width = VEHICLE_WIDTH / 2.0;
    
    std::vector<std::pair<double, double>> wheel_offsets = {
        { half_length,  half_width},   // 左前
        { half_length, -half_width},   // 右前
        {-half_length,  half_width},   // 左后
        {-half_length, -half_width}    // 右后
    };
    
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    
    for (size_t i = 0; i < wheel_offsets.size(); ++i) {
        visualization_msgs::Marker wheel;
        wheel.header.frame_id = frame_id_;
        wheel.ns = "vehicle_wheels";
        wheel.id = static_cast<int>(i);
        wheel.type = visualization_msgs::Marker::CYLINDER;
        wheel.action = visualization_msgs::Marker::ADD;
        
        // 转换到世界坐标
        double local_x = wheel_offsets[i].first;
        double local_y = wheel_offsets[i].second;
        
        wheel.pose.position.x = state.x + local_x * cos_theta - local_y * sin_theta;
        wheel.pose.position.y = state.y + local_x * sin_theta + local_y * cos_theta;
        wheel.pose.position.z = WHEEL_RADIUS;
        
        // 朝向（躺倒的圆柱）
        tf2::Quaternion q;
        q.setRPY(M_PI / 2.0, 0, state.theta);
        wheel.pose.orientation.x = q.x();
        wheel.pose.orientation.y = q.y();
        wheel.pose.orientation.z = q.z();
        wheel.pose.orientation.w = q.w();
        
        // 尺寸
        wheel.scale.x = WHEEL_RADIUS * 2.0;
        wheel.scale.y = WHEEL_RADIUS * 2.0;
        wheel.scale.z = WHEEL_WIDTH;
        
        // 颜色
        wheel.color.r = VEHICLE_WHEEL[0];
        wheel.color.g = VEHICLE_WHEEL[1];
        wheel.color.b = VEHICLE_WHEEL[2];
        wheel.color.a = VEHICLE_WHEEL[3];
        
        wheel.lifetime = ros::Duration(0.1);
        
        wheels.push_back(wheel);
    }
    
    return wheels;
}

visualization_msgs::Marker VehicleVisualizer::createTrailMarker(const ros::Time& stamp) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "vehicle_trail";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = TRAIL_WIDTH;
    
    marker.color.r = VEHICLE_TRAIL[0];
    marker.color.g = VEHICLE_TRAIL[1];
    marker.color.b = VEHICLE_TRAIL[2];
    marker.color.a = VEHICLE_TRAIL[3];
    
    marker.pose.orientation.w = 1.0;
    
    // 渐变透明度
    for (size_t i = 0; i < trail_points_.size(); ++i) {
        marker.points.push_back(trail_points_[i]);
        
        // 颜色渐变
        std_msgs::ColorRGBA color;
        float alpha = static_cast<float>(i) / static_cast<float>(trail_points_.size());
        color.r = VEHICLE_TRAIL[0];
        color.g = VEHICLE_TRAIL[1];
        color.b = VEHICLE_TRAIL[2];
        color.a = VEHICLE_TRAIL[3] * alpha;
        marker.colors.push_back(color);
    }
    
    marker.lifetime = ros::Duration(0.1);
    
    return marker;
}

void VehicleVisualizer::clearTrail() {
    trail_points_.clear();
    ROS_INFO("[VehicleVisualizer] Trail cleared");
}

}  // namespace fsd_viz
