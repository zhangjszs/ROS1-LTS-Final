#pragma once

#include <ros/ros.h>
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_SimState.h>
#include <geometry_msgs/Pose2D.h>
#include "fsd_visualization/viz_config.hpp"

namespace fsd_viz {

class VehicleVisualizer {
public:
    VehicleVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void clearTrail();

private:
    void carStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr& msg);
    void simStateCallback(const autodrive_msgs::HUAT_SimState::ConstPtr& msg);

    // 使用 geometry_msgs::Pose2D 作为参数（x, y, theta）
    visualization_msgs::Marker createBodyMarker(const geometry_msgs::Pose2D& state);
    visualization_msgs::Marker createArrowMarker(const geometry_msgs::Pose2D& state);
    std::vector<visualization_msgs::Marker> createWheelMarkers(const geometry_msgs::Pose2D& state);
    visualization_msgs::Marker createTrailMarker(const ros::Time& stamp);

    // 新增：速度矢量和转向可视化
    visualization_msgs::Marker createVelocityMarker(const geometry_msgs::Pose2D& state,
                                                     double vx, double vy);
    visualization_msgs::Marker createSteeringMarker(const geometry_msgs::Pose2D& state,
                                                     double steering_angle);

    ros::Subscriber sub_car_state_;
    ros::Subscriber sub_sim_state_;
    ros::Publisher pub_markers_;

    std::string car_state_topic_;
    std::string sim_state_topic_;
    std::string markers_topic_;

    std::string frame_id_;
    int trail_max_size_;
    bool show_trail_;
    bool show_velocity_;
    bool show_steering_;

    // 轨迹历史
    std::deque<geometry_msgs::Point> trail_points_;

    // 仿真状态缓存
    double cached_vx_ = 0.0;
    double cached_vy_ = 0.0;
    double cached_steering_ = 0.0;
    bool has_sim_state_ = false;
};

}  // namespace fsd_viz
