#include "planning_ros/line_detection_node.hpp"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

namespace planning_ros
{

LineDetectionNode::LineDetectionNode(ros::NodeHandle &nh)
  : nh_(nh), core_(params_)
{
  LoadParameters();
  core_.SetParams(params_);

  cone_sub_ = nh_.subscribe(cone_topic_, 1, &LineDetectionNode::ConeCallback, this);
  car_state_sub_ = nh_.subscribe(car_state_topic_, 1, &LineDetectionNode::CarStateCallback, this);

  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
  finish_pub_ = nh_.advertise<std_msgs::Bool>("line_finish_signal", 1);

  ROS_INFO("[LineDetection] Node initialized");
}

void LineDetectionNode::LoadParameters()
{
  nh_.param("hough/rho_resolution", params_.hough_rho_resolution, 0.1);
  nh_.param("hough/theta_resolution", params_.hough_theta_resolution, 0.01);
  nh_.param("hough/min_votes", params_.hough_min_votes, 3);
  nh_.param("hough/theta_tolerance", params_.theta_tolerance, 0.2);

  nh_.param("cones/max_distance", params_.max_cone_distance, 50.0);
  nh_.param("cones/min_distance", params_.min_cone_distance, 2.0);

  nh_.param("path/interval", params_.path_interval, 0.1);
  nh_.param("path/max_distance", params_.max_path_distance, 75.0);

  nh_.param("vehicle/imu_offset_x", params_.imu_offset_x, 1.88);

  nh_.param("finish/threshold", params_.finish_line_threshold, 2.0);

  nh_.param<std::string>("topics/cone", cone_topic_, "/cone_position");
  nh_.param<std::string>("topics/car_state", car_state_topic_, "/Carstate");
  nh_.param<std::string>("topics/path", path_topic_, "/line_planned_path");
}

void LineDetectionNode::ConeCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &cone_msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (cone_msg->points.empty())
  {
    ROS_WARN("[LineDetection] Received empty cone message");
    core_.UpdateCones({});
    return;
  }

  std::vector<planning_core::ConePoint> cones;
  cones.reserve(cone_msg->points.size());

  for (const geometry_msgs::Point32 &point : cone_msg->points)
  {
    planning_core::ConePoint cone;
    cone.x = point.x;
    cone.y = point.y;
    cone.z = point.z;
    cones.push_back(cone);
  }

  core_.UpdateCones(cones);
}

void LineDetectionNode::CarStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &car_state)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  planning_core::VehicleState state;
  state.x = car_state->car_state.x;
  state.y = car_state->car_state.y;
  state.theta = car_state->car_state.theta;
  state.v = car_state->V;

  core_.UpdateVehicleState(state);
}

void LineDetectionNode::PublishPath(const std::vector<planning_core::Pose> &path_points)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = ros::Time::now();
  path_msg.poses.reserve(path_points.size());

  for (const auto &pose : path_points)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = path_msg.header;
    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    pose_msg.pose.position.z = pose.z;
    pose_msg.pose.orientation.x = pose.qx;
    pose_msg.pose.orientation.y = pose.qy;
    pose_msg.pose.orientation.z = pose.qz;
    pose_msg.pose.orientation.w = pose.qw;
    path_msg.poses.push_back(pose_msg);
  }

  path_pub_.publish(path_msg);
}

void LineDetectionNode::PublishFinishOnce()
{
  if (finish_published_)
  {
    return;
  }

  std_msgs::Bool finish_msg;
  finish_msg.data = true;
  finish_pub_.publish(finish_msg);
  finish_published_ = true;

  ROS_INFO("[LineDetection] Finish line reached!");
}

void LineDetectionNode::RunOnce()
{
  std::vector<planning_core::Pose> planned_path;
  bool finished = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    core_.RunAlgorithm();
    finished = core_.IsFinished();
    if (core_.HasPlannedPath())
    {
      planned_path = core_.GetPlannedPath();
    }
  }

  if (!finished && !planned_path.empty())
  {
    PublishPath(planned_path);
  }

  if (finished)
  {
    PublishFinishOnce();
  }
}

} // namespace planning_ros
