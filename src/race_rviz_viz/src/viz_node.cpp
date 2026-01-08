/**
 * @file viz_node.cpp
 * @brief RViz visualization node for cones, vehicle body, and wheels.
 *
 * Subscriptions:
 *   - /coneMap     (common_msgs/HUAT_map)      : Cone map in global frame
 *   - /Carstate    (common_msgs/HUAT_Carstate) : Vehicle pose in global frame
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
#include <common_msgs/HUAT_Carstate.h>
#include <common_msgs/HUAT_map.h>
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
    publish_high_speed_tracking_markers_ = false;
    if (!pnh_.getParam("publish_high_speed_tracking_markers", publish_high_speed_tracking_markers_))
    {
      pnh_.getParam("publish_urinay_markers", publish_high_speed_tracking_markers_);
    }
    high_speed_tracking_triangulation_topic_ = "/AS/P/high_speed_tracking/markers/triangulation";
    if (!pnh_.getParam("high_speed_tracking_triangulation_topic", high_speed_tracking_triangulation_topic_))
    {
      pnh_.getParam("urinay_triangulation_topic", high_speed_tracking_triangulation_topic_);
    }
    high_speed_tracking_midpoints_topic_ = "/AS/P/high_speed_tracking/markers/midpoints";
    if (!pnh_.getParam("high_speed_tracking_midpoints_topic", high_speed_tracking_midpoints_topic_))
    {
      pnh_.getParam("urinay_midpoints_topic", high_speed_tracking_midpoints_topic_);
    }
    high_speed_tracking_way_topic_ = "/AS/P/high_speed_tracking/markers/way";
    if (!pnh_.getParam("high_speed_tracking_way_topic", high_speed_tracking_way_topic_))
    {
      pnh_.getParam("urinay_way_topic", high_speed_tracking_way_topic_);
    }

    pnh_.param<std::string>("cone_marker_topic", cone_marker_topic_, "/coneMarker");
    pnh_.param<std::string>("car_body_topic", car_body_topic_, "/carBody");
    pnh_.param<std::string>("wheels_topic", wheels_topic_, "/whole");
    pnh_.param<std::string>("path_out_topic", path_out_topic_, "/viz/path");
    high_speed_tracking_triangulation_out_topic_ = "/viz/high_speed_tracking/markers/triangulation";
    if (!pnh_.getParam("high_speed_tracking_triangulation_out_topic", high_speed_tracking_triangulation_out_topic_))
    {
      pnh_.getParam("urinay_triangulation_out_topic", high_speed_tracking_triangulation_out_topic_);
    }
    high_speed_tracking_midpoints_out_topic_ = "/viz/high_speed_tracking/markers/midpoints";
    if (!pnh_.getParam("high_speed_tracking_midpoints_out_topic", high_speed_tracking_midpoints_out_topic_))
    {
      pnh_.getParam("urinay_midpoints_out_topic", high_speed_tracking_midpoints_out_topic_);
    }
    high_speed_tracking_way_out_topic_ = "/viz/high_speed_tracking/markers/way";
    if (!pnh_.getParam("high_speed_tracking_way_out_topic", high_speed_tracking_way_out_topic_))
    {
      pnh_.getParam("urinay_way_out_topic", high_speed_tracking_way_out_topic_);
    }

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

    if (publish_high_speed_tracking_markers_)
    {
      high_speed_tracking_triangulation_sub_ = nh_.subscribe(high_speed_tracking_triangulation_topic_, 10, &RaceRvizViz::high_speed_trackingTriangulationCallback, this);
      high_speed_tracking_midpoints_sub_ = nh_.subscribe(high_speed_tracking_midpoints_topic_, 10, &RaceRvizViz::high_speed_trackingMidpointsCallback, this);
      high_speed_tracking_way_sub_ = nh_.subscribe(high_speed_tracking_way_topic_, 10, &RaceRvizViz::high_speed_trackingWayCallback, this);
      high_speed_tracking_triangulation_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(high_speed_tracking_triangulation_out_topic_, 10);
      high_speed_tracking_midpoints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(high_speed_tracking_midpoints_out_topic_, 10);
      high_speed_tracking_way_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(high_speed_tracking_way_out_topic_, 10);
    }

    ROS_INFO("[RaceRvizViz] Initialized.");
  }

  void conesCallback(const common_msgs::HUAT_map::ConstPtr& msg)
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

  void high_speed_trackingTriangulationCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    high_speed_tracking_triangulation_pub_.publish(*msg);
  }

  void high_speed_trackingMidpointsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    high_speed_tracking_midpoints_pub_.publish(*msg);
  }

  void high_speed_trackingWayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    high_speed_tracking_way_pub_.publish(*msg);
  }

  void carstateCallback(const common_msgs::HUAT_Carstate::ConstPtr& msg)
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber cones_sub_;
  ros::Subscriber carstate_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber high_speed_tracking_triangulation_sub_;
  ros::Subscriber high_speed_tracking_midpoints_sub_;
  ros::Subscriber high_speed_tracking_way_sub_;

  ros::Publisher cone_marker_pub_;
  ros::Publisher car_body_pub_;
  ros::Publisher wheels_pub_;
  ros::Publisher path_pub_;
  ros::Publisher high_speed_tracking_triangulation_pub_;
  ros::Publisher high_speed_tracking_midpoints_pub_;
  ros::Publisher high_speed_tracking_way_pub_;

  std::string global_frame_;
  std::string vehicle_frame_;
  std::string cones_topic_;
  std::string carstate_topic_;
  std::string path_topic_;
  std::string high_speed_tracking_triangulation_topic_;
  std::string high_speed_tracking_midpoints_topic_;
  std::string high_speed_tracking_way_topic_;
  std::string cone_marker_topic_;
  std::string car_body_topic_;
  std::string wheels_topic_;
  std::string path_out_topic_;
  std::string high_speed_tracking_triangulation_out_topic_;
  std::string high_speed_tracking_midpoints_out_topic_;
  std::string high_speed_tracking_way_out_topic_;
  bool publish_tf_;
  bool publish_cones_;
  bool publish_high_speed_tracking_markers_;

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
