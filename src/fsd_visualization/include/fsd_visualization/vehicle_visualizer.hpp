#pragma once

#include <ros/ros.h>
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <geometry_msgs/Pose2D.h>
#include "fsd_visualization/viz_config.hpp"

namespace fsd_viz {

class VehicleVisualizer {
public:
    VehicleVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    void clearTrail();
    
private:
    void carStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr& msg);
    
    // 使用 geometry_msgs::Pose2D 作为参数（x, y, theta）
    visualization_msgs::Marker createBodyMarker(const geometry_msgs::Pose2D& state);
    visualization_msgs::Marker createArrowMarker(const geometry_msgs::Pose2D& state);
    std::vector<visualization_msgs::Marker> createWheelMarkers(const geometry_msgs::Pose2D& state);
    visualization_msgs::Marker createTrailMarker(const ros::Time& stamp);
    
    ros::Subscriber sub_car_state_;
    ros::Publisher pub_markers_;
    
    std::string frame_id_;
    int trail_max_size_;
    bool show_trail_;
    
    // 轨迹历史
    std::deque<geometry_msgs::Point> trail_points_;
};

}  // namespace fsd_viz
