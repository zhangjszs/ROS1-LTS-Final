#include "fsd_visualization/vehicle_visualizer.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace fsd_viz {

VehicleVisualizer::VehicleVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // 参数
    pnh.param<std::string>("frame_id", frame_id_, FRAME_GLOBAL);
    pnh.param<int>("trail_length", trail_max_size_, 200);
    pnh.param<bool>("show_trail", show_trail_, true);
    pnh.param<bool>("show_velocity", show_velocity_, true);
    pnh.param<bool>("show_steering", show_steering_, true);
    pnh.param<bool>("use_mesh", use_mesh_, true);
    pnh.param<std::string>("topics/car_state", car_state_topic_, "localization/car_state");
    pnh.param<std::string>("topics/sim_state", sim_state_topic_, "/simulation/state");
    pnh.param<std::string>("topics/markers", markers_topic_, "fsd/viz/vehicle");

    // 订阅
    sub_car_state_ = nh.subscribe(car_state_topic_, 1,
        &VehicleVisualizer::carStateCallback, this);
    sub_sim_state_ = nh.subscribe(sim_state_topic_, 1,
        &VehicleVisualizer::simStateCallback, this);

    // 发布
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(markers_topic_, 1);

    ROS_INFO("[VehicleVisualizer] Initialized, trail_length=%d, show_velocity=%d, show_steering=%d",
             trail_max_size_, show_velocity_, show_steering_);
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
    if (show_trail_ && trail_points_.size() >= 2) {
        auto trail = createTrailMarker(msg->header.stamp);
        markers.markers.push_back(trail);
    }

    // 速度矢量（如果有仿真状态）
    if (show_velocity_ && has_sim_state_) {
        auto velocity = createVelocityMarker(state, cached_vx_, cached_vy_);
        velocity.header.stamp = msg->header.stamp;
        markers.markers.push_back(velocity);
    }

    // 转向指示（如果有仿真状态）
    if (show_steering_ && has_sim_state_) {
        auto steering = createSteeringMarker(state, cached_steering_);
        steering.header.stamp = msg->header.stamp;
        markers.markers.push_back(steering);
    }

    pub_markers_.publish(markers);
}

visualization_msgs::Marker VehicleVisualizer::createBodyMarker(
    const geometry_msgs::Pose2D& state) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    // 朝向 - 使用 theta 字段
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    if (use_mesh_) {
        // 3D STL 赛车模型
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = MESH_VEHICLE_BODY;

        marker.pose.position.x = state.x;
        marker.pose.position.y = state.y;
        marker.pose.position.z = 0.0;

        // STL 单位为 mm，缩放到 m
        marker.scale.x = VEHICLE_MESH_SCALE;
        marker.scale.y = VEHICLE_MESH_SCALE;
        marker.scale.z = VEHICLE_MESH_SCALE;

        // 浅灰色
        marker.color.r = 0.75f;
        marker.color.g = 0.75f;
        marker.color.b = 0.75f;
        marker.color.a = 1.0f;
    } else {
        // 原始 CUBE 渲染
        marker.type = visualization_msgs::Marker::CUBE;

        marker.pose.position.x = state.x;
        marker.pose.position.y = state.y;
        marker.pose.position.z = VEHICLE_HEIGHT / 2.0;

        marker.scale.x = VEHICLE_LENGTH;
        marker.scale.y = VEHICLE_WIDTH;
        marker.scale.z = VEHICLE_HEIGHT;

        marker.color.r = VEHICLE_BODY[0];
        marker.color.g = VEHICLE_BODY[1];
        marker.color.b = VEHICLE_BODY[2];
        marker.color.a = VEHICLE_BODY[3];
    }

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

    // STL 模型已包含车轮，跳过
    if (use_mesh_) {
        return wheels;
    }
    
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

void VehicleVisualizer::simStateCallback(
    const autodrive_msgs::HUAT_SimState::ConstPtr& msg) {
    // 缓存仿真状态用于可视化
    cached_vx_ = msg->velocity.x;
    cached_vy_ = msg->velocity.y;
    cached_steering_ = msg->steering_angle;
    has_sim_state_ = true;
}

visualization_msgs::Marker VehicleVisualizer::createVelocityMarker(
    const geometry_msgs::Pose2D& state, double vx, double vy) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "vehicle_velocity";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // 起点在车辆位置
    geometry_msgs::Point start, end;
    start.x = state.x;
    start.y = state.y;
    start.z = VEHICLE_HEIGHT + 0.3;

    // 将车体坐标系的速度转换到全局坐标系
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double vx_global = vx * cos_theta - vy * sin_theta;
    double vy_global = vx * sin_theta + vy * cos_theta;

    // 速度矢量缩放（1m/s = 0.5m箭头长度）
    double scale = 0.5;
    end.x = start.x + vx_global * scale;
    end.y = start.y + vy_global * scale;
    end.z = start.z;

    marker.points.push_back(start);
    marker.points.push_back(end);

    // 箭头尺寸
    marker.scale.x = 0.1;   // 箭身直径
    marker.scale.y = 0.2;   // 箭头直径
    marker.scale.z = 0.2;   // 箭头长度

    // 颜色
    marker.color.r = VEHICLE_VELOCITY[0];
    marker.color.g = VEHICLE_VELOCITY[1];
    marker.color.b = VEHICLE_VELOCITY[2];
    marker.color.a = VEHICLE_VELOCITY[3];

    marker.lifetime = ros::Duration(0.1);

    return marker;
}

visualization_msgs::Marker VehicleVisualizer::createSteeringMarker(
    const geometry_msgs::Pose2D& state, double steering_angle) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.ns = "vehicle_steering";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // 前轴中心位置
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double front_offset = VEHICLE_LENGTH / 2.0 - 0.3;

    double front_x = state.x + front_offset * cos_theta;
    double front_y = state.y + front_offset * sin_theta;

    // 起点在前轴
    geometry_msgs::Point start, end;
    start.x = front_x;
    start.y = front_y;
    start.z = WHEEL_RADIUS + 0.1;

    // 转向方向
    double steer_global = state.theta + steering_angle;
    double arrow_length = 0.8;
    end.x = start.x + arrow_length * std::cos(steer_global);
    end.y = start.y + arrow_length * std::sin(steer_global);
    end.z = start.z;

    marker.points.push_back(start);
    marker.points.push_back(end);

    // 箭头尺寸
    marker.scale.x = 0.08;  // 箭身直径
    marker.scale.y = 0.15;  // 箭头直径
    marker.scale.z = 0.15;  // 箭头长度

    // 颜色
    marker.color.r = VEHICLE_STEERING[0];
    marker.color.g = VEHICLE_STEERING[1];
    marker.color.b = VEHICLE_STEERING[2];
    marker.color.a = VEHICLE_STEERING[3];

    marker.lifetime = ros::Duration(0.1);

    return marker;
}

}  // namespace fsd_viz
