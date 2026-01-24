/**
 * @file viz_node.cpp
 * @brief RViz visualization node for cones, vehicle body, and wheels.
 *
 * Subscriptions:
 *   - /coneMap     (autodrive_msgs/HUAT_ConeMap)      : Cone map in global frame
 *   - /Carstate    (autodrive_msgs/HUAT_CarState) : Vehicle pose in global frame
 *   - /path_global (nav_msgs/Path)             : Optional path in global frame
 *
 * Publications:
 *   - /coneMarker  (visualization_msgs/Marker) : Cone markers
 *   - /carBody     (visualization_msgs/Marker) : Vehicle body outline
 *   - /whole       (visualization_msgs/Marker) : Wheels
 *   - /viz/path    (nav_msgs/Path)             : Forwarded path
 *   - TF: global_frame -> vehicle_frame
 */

#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/HUAT_HighSpeedViz.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

class RaceRvizViz
{
public:
  RaceRvizViz(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), tf_broadcaster_()
  {
    pnh_.param<std::string>("global_frame", global_frame_, "velodyne");
    pnh_.param<std::string>("vehicle_frame", vehicle_frame_, "base_link");

    pnh_.param<bool>("publish_cones", publish_cones_, false);
    pnh_.param<std::string>("cones_topic", cones_topic_, "/coneMap");
    pnh_.param<std::string>("carstate_topic", carstate_topic_, "/Carstate");
    pnh_.param<std::string>("path_topic", path_topic_, "/path_global");
    pnh_.param<bool>("publish_high_speed_tracking_viz", publish_high_speed_tracking_viz_, false);
    pnh_.param<std::string>("high_speed_tracking_viz_topic", high_speed_tracking_viz_topic_, "/AS/P/high_speed_tracking/viz");

    pnh_.param<std::string>("cone_marker_topic", cone_marker_topic_, "/coneMarker");
    pnh_.param<std::string>("car_body_topic", car_body_topic_, "/carBody");
    pnh_.param<std::string>("wheels_topic", wheels_topic_, "/whole");
    pnh_.param<std::string>("path_out_topic", path_out_topic_, "/viz/path");
    pnh_.param<std::string>("high_speed_tracking_triangulation_out_topic",
                            high_speed_tracking_triangulation_out_topic_,
                            "/viz/high_speed_tracking/markers/triangulation");
    pnh_.param<std::string>("high_speed_tracking_midpoints_out_topic",
                            high_speed_tracking_midpoints_out_topic_,
                            "/viz/high_speed_tracking/markers/midpoints");
    pnh_.param<std::string>("high_speed_tracking_way_out_topic",
                            high_speed_tracking_way_out_topic_,
                            "/viz/high_speed_tracking/markers/way");

    pnh_.param<bool>("publish_lidar_cluster_bboxes", publish_lidar_cluster_bboxes_, true);
    pnh_.param<std::string>("lidar_cluster_cone_topic", lidar_cluster_cone_topic_, "/cone_position");
    pnh_.param<std::string>("lidar_cluster_bbox_out_topic", lidar_cluster_bbox_out_topic_, "/viz/lidar_cluster/bounding_box");

    pnh_.param<bool>("publish_pathlimits", publish_pathlimits_, false);
    pnh_.param<std::string>("pathlimits_topic", pathlimits_topic_, "/AS/P/pathlimits/partial");
    pnh_.param<std::string>("pathlimits_path_topic", pathlimits_path_topic_, "/viz/pathlimits/path");
    pnh_.param<std::string>("pathlimits_left_topic", pathlimits_left_topic_, "/viz/pathlimits/left");
    pnh_.param<std::string>("pathlimits_right_topic", pathlimits_right_topic_, "/viz/pathlimits/right");

    pnh_.param<bool>("publish_tf", publish_tf_, true);

    pnh_.param<double>("cone_radius", cone_radius_, 0.15);
    pnh_.param<double>("cone_height", cone_height_, 0.35);
    pnh_.param<double>("cone_color_r", cone_color_r_, 1.0);
    pnh_.param<double>("cone_color_g", cone_color_g_, 0.5);
    pnh_.param<double>("cone_color_b", cone_color_b_, 0.0);
    pnh_.param<double>("cone_color_a", cone_color_a_, 1.0);

    pnh_.param<double>("lidar_to_imu", lidar_to_imu_, 1.87);
    pnh_.param<double>("car_body_front", car_body_front_, 0.0);
    pnh_.param<double>("car_body_rear", car_body_rear_, -1.5);
    pnh_.param<double>("car_body_half_width", car_body_half_width_, 0.25);
    pnh_.param<double>("car_body_line_width", car_body_line_width_, 0.3);
    pnh_.param<double>("car_body_height", car_body_height_, 0.5);
    pnh_.param<double>("car_body_color_r", car_body_color_r_, 1.0);
    pnh_.param<double>("car_body_color_g", car_body_color_g_, 0.0);
    pnh_.param<double>("car_body_color_b", car_body_color_b_, 0.0);
    pnh_.param<double>("car_body_color_a", car_body_color_a_, 1.0);

    pnh_.param<double>("wheel_front_x", wheel_front_x_, -0.2);
    pnh_.param<double>("wheel_rear_x", wheel_rear_x_, -1.2);
    pnh_.param<double>("wheel_half_track", wheel_half_track_, 0.35);
    pnh_.param<double>("wheel_diameter", wheel_diameter_, 0.5);
    pnh_.param<double>("wheel_width", wheel_width_, 0.10);
    pnh_.param<double>("wheel_color_r", wheel_color_r_, 0.0);
    pnh_.param<double>("wheel_color_g", wheel_color_g_, 1.0);
    pnh_.param<double>("wheel_color_b", wheel_color_b_, 0.0);
    pnh_.param<double>("wheel_color_a", wheel_color_a_, 1.0);

    pnh_.param<double>("pathlimits_line_width", pathlimits_line_width_, 0.03);
    pnh_.param<double>("pathlimits_path_color_r", pathlimits_path_color_r_, 1.0);
    pnh_.param<double>("pathlimits_path_color_g", pathlimits_path_color_g_, 0.5);
    pnh_.param<double>("pathlimits_path_color_b", pathlimits_path_color_b_, 0.0);
    pnh_.param<double>("pathlimits_path_color_a", pathlimits_path_color_a_, 0.6);
    pnh_.param<double>("pathlimits_left_color_r", pathlimits_left_color_r_, 0.0);
    pnh_.param<double>("pathlimits_left_color_g", pathlimits_left_color_g_, 0.4);
    pnh_.param<double>("pathlimits_left_color_b", pathlimits_left_color_b_, 1.0);
    pnh_.param<double>("pathlimits_left_color_a", pathlimits_left_color_a_, 0.45);
    pnh_.param<double>("pathlimits_right_color_r", pathlimits_right_color_r_, 1.0);
    pnh_.param<double>("pathlimits_right_color_g", pathlimits_right_color_g_, 1.0);
    pnh_.param<double>("pathlimits_right_color_b", pathlimits_right_color_b_, 0.0);
    pnh_.param<double>("pathlimits_right_color_a", pathlimits_right_color_a_, 0.45);
    pnh_.param<double>("pathlimits_z_offset_path", pathlimits_z_offset_path_, 0.07);
    pnh_.param<double>("pathlimits_z_offset_lr", pathlimits_z_offset_lr_, 0.06);

    pnh_.param<double>("high_speed_triangulation_line_width", high_speed_triangulation_line_width_, 0.05);
    pnh_.param<double>("high_speed_triangulation_color_r", high_speed_triangulation_color_r_, 1.0);
    pnh_.param<double>("high_speed_triangulation_color_g", high_speed_triangulation_color_g_, 0.0);
    pnh_.param<double>("high_speed_triangulation_color_b", high_speed_triangulation_color_b_, 0.0);
    pnh_.param<double>("high_speed_triangulation_color_a", high_speed_triangulation_color_a_, 1.0);

    pnh_.param<double>("high_speed_circumcenter_scale", high_speed_circumcenter_scale_, 0.1);
    pnh_.param<double>("high_speed_circumcenter_color_r", high_speed_circumcenter_color_r_, 0.0);
    pnh_.param<double>("high_speed_circumcenter_color_g", high_speed_circumcenter_color_g_, 1.0);
    pnh_.param<double>("high_speed_circumcenter_color_b", high_speed_circumcenter_color_b_, 1.0);
    pnh_.param<double>("high_speed_circumcenter_color_a", high_speed_circumcenter_color_a_, 1.0);

    pnh_.param<double>("high_speed_tri_midpoint_scale", high_speed_tri_midpoint_scale_, 0.1);
    pnh_.param<double>("high_speed_tri_midpoint_color_r", high_speed_tri_midpoint_color_r_, 1.0);
    pnh_.param<double>("high_speed_tri_midpoint_color_g", high_speed_tri_midpoint_color_g_, 0.0);
    pnh_.param<double>("high_speed_tri_midpoint_color_b", high_speed_tri_midpoint_color_b_, 1.0);
    pnh_.param<double>("high_speed_tri_midpoint_color_a", high_speed_tri_midpoint_color_a_, 0.3);

    pnh_.param<double>("high_speed_midpoint_scale", high_speed_midpoint_scale_, 0.04);
    pnh_.param<double>("high_speed_midpoint_color_r", high_speed_midpoint_color_r_, 0.2);
    pnh_.param<double>("high_speed_midpoint_color_g", high_speed_midpoint_color_g_, 0.8);
    pnh_.param<double>("high_speed_midpoint_color_b", high_speed_midpoint_color_b_, 1.0);
    pnh_.param<double>("high_speed_midpoint_color_a", high_speed_midpoint_color_a_, 0.4);

    pnh_.param<double>("high_speed_path_width", high_speed_path_width_, 0.2);
    pnh_.param<double>("high_speed_path_color_r", high_speed_path_color_r_, 0.0);
    pnh_.param<double>("high_speed_path_color_g", high_speed_path_color_g_, 1.0);
    pnh_.param<double>("high_speed_path_color_b", high_speed_path_color_b_, 0.0);
    pnh_.param<double>("high_speed_path_color_a", high_speed_path_color_a_, 1.0);

    pnh_.param<double>("high_speed_left_width", high_speed_left_width_, 0.15);
    pnh_.param<double>("high_speed_left_color_r", high_speed_left_color_r_, 0.0);
    pnh_.param<double>("high_speed_left_color_g", high_speed_left_color_g_, 0.0);
    pnh_.param<double>("high_speed_left_color_b", high_speed_left_color_b_, 0.7);
    pnh_.param<double>("high_speed_left_color_a", high_speed_left_color_a_, 1.0);

    pnh_.param<double>("high_speed_right_width", high_speed_right_width_, 0.15);
    pnh_.param<double>("high_speed_right_color_r", high_speed_right_color_r_, 0.7);
    pnh_.param<double>("high_speed_right_color_g", high_speed_right_color_g_, 0.7);
    pnh_.param<double>("high_speed_right_color_b", high_speed_right_color_b_, 0.0);
    pnh_.param<double>("high_speed_right_color_a", high_speed_right_color_a_, 1.0);

    pnh_.param<double>("lidar_cluster_bbox_line_width", lidar_cluster_bbox_line_width_, 0.01);
    pnh_.param<double>("lidar_cluster_bbox_color_r", lidar_cluster_bbox_color_r_, 1.0);
    pnh_.param<double>("lidar_cluster_bbox_color_g", lidar_cluster_bbox_color_g_, 1.0);
    pnh_.param<double>("lidar_cluster_bbox_color_b", lidar_cluster_bbox_color_b_, 1.0);
    pnh_.param<double>("lidar_cluster_bbox_color_a", lidar_cluster_bbox_color_a_, 0.25);
    pnh_.param<double>("lidar_cluster_bbox_z_offset", lidar_cluster_bbox_z_offset_, 0.03);

    if (publish_cones_)
    {
      cones_sub_ = nh_.subscribe(cones_topic_, 10, &RaceRvizViz::conesCallback, this);
      cone_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(cone_marker_topic_, 10);
    }
    carstate_sub_ = nh_.subscribe(carstate_topic_, 10, &RaceRvizViz::carstateCallback, this);
    path_sub_ = nh_.subscribe(path_topic_, 10, &RaceRvizViz::pathCallback, this);

    car_body_pub_ = nh_.advertise<visualization_msgs::Marker>(car_body_topic_, 10);
    wheels_pub_ = nh_.advertise<visualization_msgs::Marker>(wheels_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_out_topic_, 10);

    if (publish_pathlimits_)
    {
      pathlimits_sub_ = nh_.subscribe(pathlimits_topic_, 10, &RaceRvizViz::pathlimitsCallback, this);
      pathlimits_path_pub_ = nh_.advertise<visualization_msgs::Marker>(pathlimits_path_topic_, 10);
      pathlimits_left_pub_ = nh_.advertise<visualization_msgs::Marker>(pathlimits_left_topic_, 10);
      pathlimits_right_pub_ = nh_.advertise<visualization_msgs::Marker>(pathlimits_right_topic_, 10);
    }

    if (publish_high_speed_tracking_viz_)
    {
      high_speed_tracking_viz_sub_ = nh_.subscribe(high_speed_tracking_viz_topic_, 10, &RaceRvizViz::highSpeedTrackingVizCallback, this);
      high_speed_tracking_triangulation_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(high_speed_tracking_triangulation_out_topic_, 10);
      high_speed_tracking_midpoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(high_speed_tracking_midpoints_out_topic_, 10);
      high_speed_tracking_way_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(high_speed_tracking_way_out_topic_, 10);
    }

    if (publish_lidar_cluster_bboxes_)
    {
      lidar_cluster_cone_sub_ = nh_.subscribe(lidar_cluster_cone_topic_, 10, &RaceRvizViz::lidarClusterConeCallback, this);
      lidar_cluster_bbox_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(lidar_cluster_bbox_out_topic_, 10);
    }

    ROS_INFO("[RaceRvizViz] Initialized.");
  }

  void conesCallback(const autodrive_msgs::HUAT_ConeMap::ConstPtr& msg)
  {
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.header.frame_id = global_frame_;
    delete_marker.header.stamp = ros::Time::now();
    cone_marker_pub_.publish(delete_marker);

    if (msg->cone.empty())
    {
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    for (const auto& cone : msg->cone)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = stamp;
      marker.ns = "cones";
      marker.id = static_cast<int>(cone.id);
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = cone.position_global.x;
      marker.pose.position.y = cone.position_global.y;
      marker.pose.position.z = cone.position_global.z + cone_height_ / 2.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = cone_radius_ * 2.0;
      marker.scale.y = cone_radius_ * 2.0;
      marker.scale.z = cone_height_;

      marker.color.r = cone_color_r_;
      marker.color.g = cone_color_g_;
      marker.color.b = cone_color_b_;
      marker.color.a = cone_color_a_;

      cone_marker_pub_.publish(marker);
    }
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    nav_msgs::Path path = *msg;
    path.header.frame_id = global_frame_;
    path.header.stamp = ros::Time::now();
    for (auto& pose : path.poses)
    {
      pose.header.frame_id = global_frame_;
    }
    path_pub_.publish(path);
  }

  void highSpeedTrackingVizCallback(const autodrive_msgs::HUAT_HighSpeedViz::ConstPtr& msg)
  {
    const std::string frame_id = msg->header.frame_id.empty() ? global_frame_ : msg->header.frame_id;
    const ros::Time stamp = ros::Time(0);

    visualization_msgs::MarkerArray triangulation_array;

    visualization_msgs::Marker tri_lines;
    tri_lines.header.frame_id = frame_id;
    tri_lines.header.stamp = stamp;
    tri_lines.ns = "tri_edges";
    tri_lines.id = 0;
    tri_lines.type = visualization_msgs::Marker::LINE_LIST;
    tri_lines.action = visualization_msgs::Marker::ADD;
    tri_lines.scale.x = high_speed_triangulation_line_width_;
    tri_lines.color.r = high_speed_triangulation_color_r_;
    tri_lines.color.g = high_speed_triangulation_color_g_;
    tri_lines.color.b = high_speed_triangulation_color_b_;
    tri_lines.color.a = high_speed_triangulation_color_a_;
    tri_lines.points = offsetPoints(msg->triangulation_lines, 0.01);
    triangulation_array.markers.push_back(tri_lines);

    visualization_msgs::Marker circumcenters;
    circumcenters.header.frame_id = frame_id;
    circumcenters.header.stamp = stamp;
    circumcenters.ns = "tri_circumcenters";
    circumcenters.id = 1;
    circumcenters.type = visualization_msgs::Marker::SPHERE_LIST;
    circumcenters.action = visualization_msgs::Marker::ADD;
    circumcenters.scale.x = high_speed_circumcenter_scale_;
    circumcenters.scale.y = high_speed_circumcenter_scale_;
    circumcenters.scale.z = high_speed_circumcenter_scale_;
    circumcenters.color.r = high_speed_circumcenter_color_r_;
    circumcenters.color.g = high_speed_circumcenter_color_g_;
    circumcenters.color.b = high_speed_circumcenter_color_b_;
    circumcenters.color.a = high_speed_circumcenter_color_a_;
    circumcenters.points = offsetPoints(msg->circumcenters, 0.01);
    triangulation_array.markers.push_back(circumcenters);

    visualization_msgs::Marker tri_midpoints;
    tri_midpoints.header.frame_id = frame_id;
    tri_midpoints.header.stamp = stamp;
    tri_midpoints.ns = "tri_midpoints";
    tri_midpoints.id = 2;
    tri_midpoints.type = visualization_msgs::Marker::CUBE_LIST;
    tri_midpoints.action = visualization_msgs::Marker::ADD;
    tri_midpoints.scale.x = high_speed_tri_midpoint_scale_;
    tri_midpoints.scale.y = high_speed_tri_midpoint_scale_;
    tri_midpoints.scale.z = high_speed_tri_midpoint_scale_;
    tri_midpoints.color.r = high_speed_tri_midpoint_color_r_;
    tri_midpoints.color.g = high_speed_tri_midpoint_color_g_;
    tri_midpoints.color.b = high_speed_tri_midpoint_color_b_;
    tri_midpoints.color.a = high_speed_tri_midpoint_color_a_;
    tri_midpoints.points = offsetPoints(msg->triangle_edge_midpoints, 0.02);
    triangulation_array.markers.push_back(tri_midpoints);

    high_speed_tracking_triangulation_pub_.publish(triangulation_array);

    visualization_msgs::MarkerArray midpoints_array;

    visualization_msgs::Marker midpoints;
    midpoints.header.frame_id = frame_id;
    midpoints.header.stamp = stamp;
    midpoints.ns = "mid_filtered";
    midpoints.id = 0;
    midpoints.type = visualization_msgs::Marker::CUBE_LIST;
    midpoints.action = visualization_msgs::Marker::ADD;
    midpoints.scale.x = high_speed_midpoint_scale_;
    midpoints.scale.y = high_speed_midpoint_scale_;
    midpoints.scale.z = high_speed_midpoint_scale_;
    midpoints.color.r = high_speed_midpoint_color_r_;
    midpoints.color.g = high_speed_midpoint_color_g_;
    midpoints.color.b = high_speed_midpoint_color_b_;
    midpoints.color.a = high_speed_midpoint_color_a_;
    midpoints.points = offsetPoints(msg->edge_midpoints, 0.03);
    midpoints_array.markers.push_back(midpoints);

    high_speed_tracking_midpoints_pub_.publish(midpoints_array);

    visualization_msgs::MarkerArray way_array;

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = frame_id;
    path_marker.header.stamp = stamp;
    path_marker.ns = "centerline";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = high_speed_path_width_;
    path_marker.color.r = high_speed_path_color_r_;
    path_marker.color.g = high_speed_path_color_g_;
    path_marker.color.b = high_speed_path_color_b_;
    path_marker.color.a = high_speed_path_color_a_;
    path_marker.points = offsetPoints(msg->path, 0.04);
    way_array.markers.push_back(path_marker);

    visualization_msgs::Marker left_marker = path_marker;
    left_marker.id = 1;
    left_marker.ns = "bound_left";
    left_marker.scale.x = high_speed_left_width_;
    left_marker.color.r = high_speed_left_color_r_;
    left_marker.color.g = high_speed_left_color_g_;
    left_marker.color.b = high_speed_left_color_b_;
    left_marker.color.a = high_speed_left_color_a_;
    left_marker.points = offsetPoints(msg->left, 0.035);
    way_array.markers.push_back(left_marker);

    visualization_msgs::Marker right_marker = path_marker;
    right_marker.id = 2;
    right_marker.ns = "bound_right";
    right_marker.scale.x = high_speed_right_width_;
    right_marker.color.r = high_speed_right_color_r_;
    right_marker.color.g = high_speed_right_color_g_;
    right_marker.color.b = high_speed_right_color_b_;
    right_marker.color.a = high_speed_right_color_a_;
    right_marker.points = offsetPoints(msg->right, 0.035);
    way_array.markers.push_back(right_marker);

    high_speed_tracking_way_pub_.publish(way_array);
  }

  void pathlimitsCallback(const autodrive_msgs::HUAT_PathLimits::ConstPtr& msg)
  {
    const std::string frame_id = msg->header.frame_id.empty() ? global_frame_ : msg->header.frame_id;
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = frame_id;
    path_marker.header.stamp = stamp;
    path_marker.ns = "pathlimits_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = pathlimits_line_width_;
    path_marker.color.r = pathlimits_path_color_r_;
    path_marker.color.g = pathlimits_path_color_g_;
    path_marker.color.b = pathlimits_path_color_b_;
    path_marker.color.a = pathlimits_path_color_a_;
    path_marker.points = offsetPoints(msg->path, pathlimits_z_offset_path_);
    pathlimits_path_pub_.publish(path_marker);

    visualization_msgs::Marker left_marker;
    left_marker.header.frame_id = frame_id;
    left_marker.header.stamp = stamp;
    left_marker.ns = "pathlimits_left";
    left_marker.id = 0;
    left_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    left_marker.action = visualization_msgs::Marker::ADD;
    left_marker.scale.x = cone_radius_ * 2.0;
    left_marker.scale.y = cone_radius_ * 2.0;
    left_marker.scale.z = cone_height_;
    left_marker.color.r = pathlimits_left_color_r_;
    left_marker.color.g = pathlimits_left_color_g_;
    left_marker.color.b = pathlimits_left_color_b_;
    left_marker.color.a = pathlimits_left_color_a_;
    left_marker.points.reserve(msg->tracklimits.left.size());
    for (const auto& cone : msg->tracklimits.left)
    {
      geometry_msgs::Point p;
      p.x = cone.position_global.x;
      p.y = cone.position_global.y;
      p.z = cone.position_global.z + pathlimits_z_offset_lr_;
      left_marker.points.push_back(p);
    }
    pathlimits_left_pub_.publish(left_marker);

    visualization_msgs::Marker right_marker;
    right_marker.header.frame_id = frame_id;
    right_marker.header.stamp = stamp;
    right_marker.ns = "pathlimits_right";
    right_marker.id = 0;
    right_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    right_marker.action = visualization_msgs::Marker::ADD;
    right_marker.scale.x = cone_radius_ * 2.0;
    right_marker.scale.y = cone_radius_ * 2.0;
    right_marker.scale.z = cone_height_;
    right_marker.color.r = pathlimits_right_color_r_;
    right_marker.color.g = pathlimits_right_color_g_;
    right_marker.color.b = pathlimits_right_color_b_;
    right_marker.color.a = pathlimits_right_color_a_;
    right_marker.points.reserve(msg->tracklimits.right.size());
    for (const auto& cone : msg->tracklimits.right)
    {
      geometry_msgs::Point p;
      p.x = cone.position_global.x;
      p.y = cone.position_global.y;
      p.z = cone.position_global.z + pathlimits_z_offset_lr_;
      right_marker.points.push_back(p);
    }
    pathlimits_right_pub_.publish(right_marker);
  }

  void lidarClusterConeCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr& msg)
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    clear.header.frame_id = msg->header.frame_id.empty() ? global_frame_ : msg->header.frame_id;
    clear.header.stamp = ros::Time::now();
    clear.ns = "lidar_cluster_bbox";
    markers.markers.push_back(clear);

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const size_t count = std::min(msg->minPoints.size(), msg->maxPoints.size());
    for (size_t i = 0; i < count; ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = clear.header.frame_id;
      marker.header.stamp = stamp;
      marker.ns = "lidar_cluster_bbox";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = lidar_cluster_bbox_line_width_;
      marker.color.r = lidar_cluster_bbox_color_r_;
      marker.color.g = lidar_cluster_bbox_color_g_;
      marker.color.b = lidar_cluster_bbox_color_b_;
      marker.color.a = lidar_cluster_bbox_color_a_;
      appendBoundingBoxPoints(msg->minPoints[i], msg->maxPoints[i], &marker, lidar_cluster_bbox_z_offset_);
      markers.markers.push_back(marker);
    }

    lidar_cluster_bbox_pub_.publish(markers);
  }

  void carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr& msg)
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const double yaw = msg->car_state.theta;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    if (publish_tf_)
    {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = global_frame_;
      tf_msg.child_frame_id = vehicle_frame_;
      tf_msg.transform.translation.x = msg->car_state.x;
      tf_msg.transform.translation.y = msg->car_state.y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = q_msg;
      tf_broadcaster_.sendTransform(tf_msg);
    }

    publishCarBody(msg->car_state.x, msg->car_state.y, yaw, stamp, q_msg);
    publishWheels(msg->car_state.x, msg->car_state.y, yaw, stamp, q_msg);
  }

private:
  void publishCarBody(double x, double y, double yaw, const ros::Time& stamp,
                      const geometry_msgs::Quaternion& orientation)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = stamp;
    marker.ns = "car_body";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(lidar_to_imu_ * std::cos(yaw), lidar_to_imu_ * std::sin(yaw), 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.setRotation(q);
    tf2::Vector3 pos(x, y, 0.0);

    const double car_length = std::max(0.01, car_body_front_ - car_body_rear_);
    const double car_width = std::max(0.01, 2.0 * car_body_half_width_);
    const double car_height = std::max(0.01, car_body_height_);
    const double center_x = 0.5 * (car_body_front_ + car_body_rear_);

    tf2::Vector3 center_local(center_x, 0.0, 0.5 * car_height);
    tf2::Vector3 center = transform * center_local;

    marker.pose.position.x = pos.x() + center.x();
    marker.pose.position.y = pos.y() + center.y();
    marker.pose.position.z = center.z();
    marker.pose.orientation = orientation;
    marker.scale.x = car_length;
    marker.scale.y = car_width;
    marker.scale.z = car_height;
    marker.color.r = car_body_color_r_;
    marker.color.g = car_body_color_g_;
    marker.color.b = car_body_color_b_;
    marker.color.a = car_body_color_a_;

    car_body_pub_.publish(marker);
  }

  visualization_msgs::Marker makeWheelMarker(int id, const tf2::Vector3& position,
                                              const geometry_msgs::Quaternion& orientation,
                                              const ros::Time& stamp)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = stamp;
    marker.ns = "wheels";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = 0.0;
    marker.pose.orientation = orientation;

    marker.scale.x = wheel_diameter_;
    marker.scale.y = wheel_diameter_;
    marker.scale.z = wheel_width_;

    marker.color.r = wheel_color_r_;
    marker.color.g = wheel_color_g_;
    marker.color.b = wheel_color_b_;
    marker.color.a = wheel_color_a_;
    return marker;
  }

  void publishWheels(double x, double y, double yaw, const ros::Time& stamp,
                     const geometry_msgs::Quaternion& orientation)
  {
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(lidar_to_imu_ * std::cos(yaw), lidar_to_imu_ * std::sin(yaw), 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.setRotation(q);
    tf2::Vector3 pos(x, y, 0.0);

    tf2::Vector3 v1(wheel_front_x_, wheel_half_track_, 0.0);
    tf2::Vector3 v2(wheel_front_x_, -wheel_half_track_, 0.0);
    tf2::Vector3 v3(wheel_rear_x_, wheel_half_track_, 0.0);
    tf2::Vector3 v4(wheel_rear_x_, -wheel_half_track_, 0.0);

    v1 = transform * v1;
    v2 = transform * v2;
    v3 = transform * v3;
    v4 = transform * v4;

    wheels_pub_.publish(makeWheelMarker(1, tf2::Vector3(pos.x() + v1.x(), pos.y() + v1.y(), 0.0), orientation, stamp));
    wheels_pub_.publish(makeWheelMarker(2, tf2::Vector3(pos.x() + v2.x(), pos.y() + v2.y(), 0.0), orientation, stamp));
    wheels_pub_.publish(makeWheelMarker(3, tf2::Vector3(pos.x() + v3.x(), pos.y() + v3.y(), 0.0), orientation, stamp));
    wheels_pub_.publish(makeWheelMarker(4, tf2::Vector3(pos.x() + v4.x(), pos.y() + v4.y(), 0.0), orientation, stamp));
  }

  static std::vector<geometry_msgs::Point> offsetPoints(const std::vector<geometry_msgs::Point>& points, double dz)
  {
    std::vector<geometry_msgs::Point> adjusted;
    adjusted.reserve(points.size());
    for (const auto& p : points)
    {
      geometry_msgs::Point q = p;
      q.z += dz;
      adjusted.push_back(q);
    }
    return adjusted;
  }

  void appendBoundingBoxPoints(const geometry_msgs::Point32& min_pt,
                               const geometry_msgs::Point32& max_pt,
                               visualization_msgs::Marker* marker,
                               double z_offset) const
  {
    geometry_msgs::Point p;
    auto push = [&](double x, double y, double z)
    {
      p.x = x;
      p.y = y;
      p.z = z;
      marker->points.push_back(p);
    };

    const double min_z = min_pt.z + z_offset;
    const double max_z = max_pt.z + z_offset;

    push(min_pt.x, min_pt.y, min_z);
    push(max_pt.x, min_pt.y, min_z);
    push(max_pt.x, min_pt.y, min_z);
    push(max_pt.x, max_pt.y, min_z);
    push(max_pt.x, max_pt.y, min_z);
    push(min_pt.x, max_pt.y, min_z);
    push(min_pt.x, max_pt.y, min_z);
    push(min_pt.x, min_pt.y, min_z);

    push(min_pt.x, min_pt.y, max_z);
    push(max_pt.x, min_pt.y, max_z);
    push(max_pt.x, min_pt.y, max_z);
    push(max_pt.x, max_pt.y, max_z);
    push(max_pt.x, max_pt.y, max_z);
    push(min_pt.x, max_pt.y, max_z);
    push(min_pt.x, max_pt.y, max_z);
    push(min_pt.x, min_pt.y, max_z);

    push(min_pt.x, min_pt.y, min_z);
    push(min_pt.x, min_pt.y, max_z);
    push(max_pt.x, min_pt.y, min_z);
    push(max_pt.x, min_pt.y, max_z);
    push(max_pt.x, max_pt.y, min_z);
    push(max_pt.x, max_pt.y, max_z);
    push(min_pt.x, max_pt.y, min_z);
    push(min_pt.x, max_pt.y, max_z);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber cones_sub_;
  ros::Subscriber carstate_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber pathlimits_sub_;
  ros::Subscriber high_speed_tracking_viz_sub_;
  ros::Subscriber lidar_cluster_cone_sub_;

  ros::Publisher cone_marker_pub_;
  ros::Publisher car_body_pub_;
  ros::Publisher wheels_pub_;
  ros::Publisher path_pub_;
  ros::Publisher pathlimits_path_pub_;
  ros::Publisher pathlimits_left_pub_;
  ros::Publisher pathlimits_right_pub_;
  ros::Publisher high_speed_tracking_triangulation_pub_;
  ros::Publisher high_speed_tracking_midpoints_pub_;
  ros::Publisher high_speed_tracking_way_pub_;
  ros::Publisher lidar_cluster_bbox_pub_;

  std::string global_frame_;
  std::string vehicle_frame_;
  std::string cones_topic_;
  std::string carstate_topic_;
  std::string path_topic_;
  std::string high_speed_tracking_viz_topic_;
  std::string cone_marker_topic_;
  std::string car_body_topic_;
  std::string wheels_topic_;
  std::string path_out_topic_;
  std::string pathlimits_topic_;
  std::string pathlimits_path_topic_;
  std::string pathlimits_left_topic_;
  std::string pathlimits_right_topic_;
  std::string high_speed_tracking_triangulation_out_topic_;
  std::string high_speed_tracking_midpoints_out_topic_;
  std::string high_speed_tracking_way_out_topic_;
  std::string lidar_cluster_cone_topic_;
  std::string lidar_cluster_bbox_out_topic_;
  bool publish_tf_;
  bool publish_cones_;
  bool publish_high_speed_tracking_viz_;
  bool publish_lidar_cluster_bboxes_;
  bool publish_pathlimits_;

  double cone_radius_;
  double cone_height_;
  double cone_color_r_, cone_color_g_, cone_color_b_, cone_color_a_;

  double lidar_to_imu_;
  double car_body_front_;
  double car_body_rear_;
  double car_body_half_width_;
  double car_body_line_width_;
  double car_body_height_;
  double car_body_color_r_, car_body_color_g_, car_body_color_b_, car_body_color_a_;

  double wheel_front_x_;
  double wheel_rear_x_;
  double wheel_half_track_;
  double wheel_diameter_;
  double wheel_width_;
  double wheel_color_r_, wheel_color_g_, wheel_color_b_, wheel_color_a_;

  double pathlimits_line_width_;
  double pathlimits_path_color_r_, pathlimits_path_color_g_, pathlimits_path_color_b_, pathlimits_path_color_a_;
  double pathlimits_left_color_r_, pathlimits_left_color_g_, pathlimits_left_color_b_, pathlimits_left_color_a_;
  double pathlimits_right_color_r_, pathlimits_right_color_g_, pathlimits_right_color_b_, pathlimits_right_color_a_;
  double pathlimits_z_offset_path_;
  double pathlimits_z_offset_lr_;

  double high_speed_triangulation_line_width_;
  double high_speed_triangulation_color_r_, high_speed_triangulation_color_g_, high_speed_triangulation_color_b_, high_speed_triangulation_color_a_;
  double high_speed_circumcenter_scale_;
  double high_speed_circumcenter_color_r_, high_speed_circumcenter_color_g_, high_speed_circumcenter_color_b_, high_speed_circumcenter_color_a_;
  double high_speed_tri_midpoint_scale_;
  double high_speed_tri_midpoint_color_r_, high_speed_tri_midpoint_color_g_, high_speed_tri_midpoint_color_b_, high_speed_tri_midpoint_color_a_;
  double high_speed_midpoint_scale_;
  double high_speed_midpoint_color_r_, high_speed_midpoint_color_g_, high_speed_midpoint_color_b_, high_speed_midpoint_color_a_;
  double high_speed_path_width_;
  double high_speed_path_color_r_, high_speed_path_color_g_, high_speed_path_color_b_, high_speed_path_color_a_;
  double high_speed_left_width_;
  double high_speed_left_color_r_, high_speed_left_color_g_, high_speed_left_color_b_, high_speed_left_color_a_;
  double high_speed_right_width_;
  double high_speed_right_color_r_, high_speed_right_color_g_, high_speed_right_color_b_, high_speed_right_color_a_;

  double lidar_cluster_bbox_line_width_;
  double lidar_cluster_bbox_color_r_, lidar_cluster_bbox_color_g_, lidar_cluster_bbox_color_b_, lidar_cluster_bbox_color_a_;
  double lidar_cluster_bbox_z_offset_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "race_rviz_viz_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  RaceRvizViz viz(nh, pnh);
  ros::spin();
  return 0;
}
