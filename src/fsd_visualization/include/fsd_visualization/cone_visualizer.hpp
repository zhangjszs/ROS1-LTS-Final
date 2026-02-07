#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
        int id, int type, const std::string& ns, const std::string& frame_id,
        float confidence = 1.0f);

    visualization_msgs::Marker createBoundingBoxMarker(
        const geometry_msgs::Point32& min_pt,
        const geometry_msgs::Point32& max_pt,
        int id, const std::string& ns, const std::string& frame_id);

    visualization_msgs::Marker createDistanceLabel(
        double x, double y, double z,
        float distance, int id, const std::string& ns, const std::string& frame_id);

    ros::Subscriber sub_cone_detections_;
    ros::Subscriber sub_cone_map_;
    ros::Publisher pub_markers_;
    
    std::string cone_detections_topic_;
    std::string cone_map_topic_;
    std::string markers_topic_;

    std::string frame_id_;
    double cone_radius_;
    double cone_height_;
    bool show_bounding_box_;
    bool show_distance_label_;
    bool use_mesh_;
    bool use_confidence_alpha_;
    bool publish_detection_markers_;
    float min_alpha_;
    std::string cone_mesh_type_;  // "fssim" or "gazebo"
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace fsd_viz
