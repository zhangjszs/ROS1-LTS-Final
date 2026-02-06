#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <autodrive_msgs/HUAT_HighSpeedViz.h>
#include "fsd_visualization/viz_config.hpp"

namespace fsd_viz {

class TriangulationVisualizer {
public:
    TriangulationVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void vizCallback(const autodrive_msgs::HUAT_HighSpeedViz::ConstPtr& msg);

    ros::Subscriber sub_viz_;
    ros::Publisher pub_markers_;

    std::string viz_topic_;
    std::string markers_topic_;
    std::string frame_id_;

    bool show_triangulation_;
    bool show_circumcenters_;
    bool show_midpoints_;
    bool show_computed_path_;
};

}  // namespace fsd_viz
