#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include "fsd_visualization/viz_config.hpp"

namespace fsd_viz {

class ConeVisualizer {
public:
    ConeVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
private:
    void coneDetectionsCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr& msg);
    void coneMapCallback(const autodrive_msgs::HUAT_ConeMap::ConstPtr& msg);
    
    visualization_msgs::Marker createConeMarker(
        double x, double y, double z,
        int id, int type, const std::string& ns);
    
    ros::Subscriber sub_cone_detections_;
    ros::Subscriber sub_cone_map_;
    ros::Publisher pub_markers_;
    
    std::string frame_id_;
    double cone_radius_;
    double cone_height_;
};

}  // namespace fsd_viz
