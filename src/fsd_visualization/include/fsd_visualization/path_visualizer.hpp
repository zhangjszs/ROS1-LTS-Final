#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <autodrive_msgs/HUAT_PathLimits.h>
#include "fsd_visualization/viz_config.hpp"

namespace fsd_viz {

class PathVisualizer {
public:
    PathVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
private:
    void pathLimitsPartialCallback(const autodrive_msgs::HUAT_PathLimits::ConstPtr& msg);
    void pathLimitsFullCallback(const autodrive_msgs::HUAT_PathLimits::ConstPtr& msg);
    void navPathCallback(const nav_msgs::Path::ConstPtr& msg);
    
    visualization_msgs::MarkerArray createPathMarkers(
        const autodrive_msgs::HUAT_PathLimits& msg,
        const std::string& ns,
        const std::array<float, 4>& color);
    
    visualization_msgs::MarkerArray createBoundaryMarkers(
        const autodrive_msgs::HUAT_PathLimits& msg);
    
    ros::Subscriber sub_path_partial_;
    ros::Subscriber sub_path_full_;
    ros::Subscriber sub_nav_path_;
    
    ros::Publisher pub_path_markers_;
    ros::Publisher pub_boundary_markers_;
    
    std::string frame_id_;
    double path_width_;
    double point_size_;
};

}  // namespace fsd_viz
