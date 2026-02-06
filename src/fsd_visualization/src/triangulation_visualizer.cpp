#include "fsd_visualization/triangulation_visualizer.hpp"

namespace fsd_viz {

TriangulationVisualizer::TriangulationVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("frame_id", frame_id_, FRAME_GLOBAL);
    pnh.param<std::string>("topics/high_speed_viz", viz_topic_, "planning/high_speed_tracking/viz");
    pnh.param<std::string>("topics/triangulation_markers", markers_topic_, "fsd/viz/triangulation");
    pnh.param<bool>("show_triangulation", show_triangulation_, true);
    pnh.param<bool>("show_circumcenters", show_circumcenters_, false);
    pnh.param<bool>("show_midpoints", show_midpoints_, true);
    pnh.param<bool>("show_computed_path", show_computed_path_, true);

    sub_viz_ = nh.subscribe(viz_topic_, 1, &TriangulationVisualizer::vizCallback, this);
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(markers_topic_, 1);

    ROS_INFO("[TriangulationVisualizer] Initialized");
}

void TriangulationVisualizer::vizCallback(
    const autodrive_msgs::HUAT_HighSpeedViz::ConstPtr& msg) {

    visualization_msgs::MarkerArray markers;
    int id = 0;

    // Delaunay 三角剖分边
    if (show_triangulation_ && !msg->triangulation_lines.empty()) {
        visualization_msgs::Marker tri_marker;
        tri_marker.header = msg->header;
        tri_marker.header.frame_id = frame_id_;
        tri_marker.ns = "triangulation_edges";
        tri_marker.id = id++;
        tri_marker.type = visualization_msgs::Marker::LINE_LIST;
        tri_marker.action = visualization_msgs::Marker::ADD;
        tri_marker.pose.orientation.w = 1.0;
        tri_marker.scale.x = TRI_LINE_WIDTH;
        tri_marker.color.r = TRI_EDGE[0];
        tri_marker.color.g = TRI_EDGE[1];
        tri_marker.color.b = TRI_EDGE[2];
        tri_marker.color.a = TRI_EDGE[3];
        tri_marker.lifetime = ros::Duration(0.5);
        tri_marker.points = msg->triangulation_lines;
        markers.markers.push_back(tri_marker);
    }

    // 外接圆圆心
    if (show_circumcenters_ && !msg->circumcenters.empty()) {
        visualization_msgs::Marker cc_marker;
        cc_marker.header = msg->header;
        cc_marker.header.frame_id = frame_id_;
        cc_marker.ns = "circumcenters";
        cc_marker.id = id++;
        cc_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        cc_marker.action = visualization_msgs::Marker::ADD;
        cc_marker.pose.orientation.w = 1.0;
        cc_marker.scale.x = MIDPOINT_SIZE;
        cc_marker.scale.y = MIDPOINT_SIZE;
        cc_marker.scale.z = MIDPOINT_SIZE;
        cc_marker.color.r = 1.0f;
        cc_marker.color.g = 1.0f;
        cc_marker.color.b = 0.0f;
        cc_marker.color.a = 0.8f;
        cc_marker.lifetime = ros::Duration(0.5);
        cc_marker.points = msg->circumcenters;
        markers.markers.push_back(cc_marker);
    }

    // 中点（edge_midpoints + triangle_edge_midpoints）
    if (show_midpoints_) {
        std::vector<geometry_msgs::Point> all_midpoints;
        all_midpoints.insert(all_midpoints.end(),
            msg->edge_midpoints.begin(), msg->edge_midpoints.end());
        all_midpoints.insert(all_midpoints.end(),
            msg->triangle_edge_midpoints.begin(), msg->triangle_edge_midpoints.end());

        if (!all_midpoints.empty()) {
            visualization_msgs::Marker mp_marker;
            mp_marker.header = msg->header;
            mp_marker.header.frame_id = frame_id_;
            mp_marker.ns = "midpoints";
            mp_marker.id = id++;
            mp_marker.type = visualization_msgs::Marker::SPHERE_LIST;
            mp_marker.action = visualization_msgs::Marker::ADD;
            mp_marker.pose.orientation.w = 1.0;
            mp_marker.scale.x = MIDPOINT_SIZE;
            mp_marker.scale.y = MIDPOINT_SIZE;
            mp_marker.scale.z = MIDPOINT_SIZE;
            mp_marker.color.r = TRI_MIDPOINT[0];
            mp_marker.color.g = TRI_MIDPOINT[1];
            mp_marker.color.b = TRI_MIDPOINT[2];
            mp_marker.color.a = TRI_MIDPOINT[3];
            mp_marker.lifetime = ros::Duration(0.5);
            mp_marker.points = all_midpoints;
            markers.markers.push_back(mp_marker);
        }
    }

    // 计算路径
    if (show_computed_path_ && msg->path.size() >= 2) {
        visualization_msgs::Marker path_marker;
        path_marker.header = msg->header;
        path_marker.header.frame_id = frame_id_;
        path_marker.ns = "computed_path";
        path_marker.id = id++;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.pose.orientation.w = 1.0;
        path_marker.scale.x = PATH_WIDTH;
        path_marker.color.r = 0.8f;
        path_marker.color.g = 0.0f;
        path_marker.color.b = 0.8f;
        path_marker.color.a = 1.0f;
        path_marker.lifetime = ros::Duration(0.5);
        path_marker.points = msg->path;
        markers.markers.push_back(path_marker);
    }

    // 左边界
    if (show_computed_path_ && msg->left.size() >= 2) {
        visualization_msgs::Marker left_marker;
        left_marker.header = msg->header;
        left_marker.header.frame_id = frame_id_;
        left_marker.ns = "boundary_left";
        left_marker.id = id++;
        left_marker.type = visualization_msgs::Marker::LINE_STRIP;
        left_marker.action = visualization_msgs::Marker::ADD;
        left_marker.pose.orientation.w = 1.0;
        left_marker.scale.x = PATH_WIDTH;
        left_marker.color.r = BOUNDARY_LEFT[0];
        left_marker.color.g = BOUNDARY_LEFT[1];
        left_marker.color.b = BOUNDARY_LEFT[2];
        left_marker.color.a = BOUNDARY_LEFT[3];
        left_marker.lifetime = ros::Duration(0.5);
        left_marker.points = msg->left;
        markers.markers.push_back(left_marker);
    }

    // 右边界
    if (show_computed_path_ && msg->right.size() >= 2) {
        visualization_msgs::Marker right_marker;
        right_marker.header = msg->header;
        right_marker.header.frame_id = frame_id_;
        right_marker.ns = "boundary_right";
        right_marker.id = id++;
        right_marker.type = visualization_msgs::Marker::LINE_STRIP;
        right_marker.action = visualization_msgs::Marker::ADD;
        right_marker.pose.orientation.w = 1.0;
        right_marker.scale.x = PATH_WIDTH;
        right_marker.color.r = BOUNDARY_RIGHT[0];
        right_marker.color.g = BOUNDARY_RIGHT[1];
        right_marker.color.b = BOUNDARY_RIGHT[2];
        right_marker.color.a = BOUNDARY_RIGHT[3];
        right_marker.lifetime = ros::Duration(0.5);
        right_marker.points = msg->right;
        markers.markers.push_back(right_marker);
    }

    pub_markers_.publish(markers);
}

}  // namespace fsd_viz
