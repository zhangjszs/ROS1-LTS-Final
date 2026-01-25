#include "planning_ros/skidpad_detection_node.hpp"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

namespace planning_ros
{

SkidpadDetectionNode::SkidpadDetectionNode(ros::NodeHandle &nh)
  : nh_(nh), pnh_("~"), core_(params_)
{
  LoadParameters();
  core_.SetParams(params_);

  cone_sub_ = nh_.subscribe(cone_topic_, 100000, &SkidpadDetectionNode::ConeCallback, this);
  car_state_sub_ = nh_.subscribe(car_state_topic_, 1000, &SkidpadDetectionNode::CarStateCallback, this);
  log_path_pub_ = nh_.advertise<nav_msgs::Path>(log_path_topic_, 10, true);
  approaching_goal_pub_ = nh_.advertise<std_msgs::Bool>(approaching_goal_topic_, 10);
}

void SkidpadDetectionNode::LoadParameters()
{
  ROS_INFO("skidpad_detection loading Parameters");
  if (!pnh_.param("length/circle2lidar", params_.circle2lidar, 15.0))
  {
    ROS_WARN_STREAM("Did not load circle2lidar. Standard value is: " << params_.circle2lidar);
  }

  ROS_INFO("load parameters");
  if (!pnh_.param("length/targetX", params_.targetX, 16.0))
  {
    ROS_WARN_STREAM("Did not load targetX. Standard value is: " << params_.targetX);
  }

  if (!pnh_.param("length/targetY", params_.targetY, -1.0))
  {
    ROS_WARN_STREAM("Did not load targetY. Standard value is " << params_.targetY);
  }

  if (!pnh_.param("length/FinTargetX", params_.FinTargetX, 40.0))
  {
    ROS_WARN_STREAM("Did not load FinTargetX. Standard value is: " << params_.FinTargetX);
  }

  if (!pnh_.param("length/FinTargetY", params_.FinTargetY, 0.0))
  {
    ROS_WARN_STREAM("Did not load FinTargetY. Standard value is " << params_.FinTargetY);
  }

  if (!pnh_.param("length/distanceThreshold", params_.distanceThreshold, 0.5))
  {
    ROS_WARN_STREAM("Did not load distanceThreshold. Standard value is " << params_.distanceThreshold);
  }

  if (!pnh_.param("length/leavedistanceThreshold", params_.leavedistanceThreshold, 1.0))
  {
    ROS_WARN_STREAM("Did not load distanceThreshold. Standard value is " << params_.leavedistanceThreshold);
  }

  if (!pnh_.param("inverse_flag", params_.inverse_flag, 1))
  {
    ROS_WARN_STREAM("Did not load topic name. Standard value is: " << params_.inverse_flag);
  }

  if (!pnh_.param("length/stopdistance", params_.stopdistance, 5.0))
  {
    ROS_WARN_STREAM("Did not load topic name. Standard value is: " << params_.stopdistance);
  }

  if (!pnh_.param<std::string>("topics/cone", cone_topic_, "perception/lidar_cluster/detections"))
  {
    ROS_WARN_STREAM("Did not load topics/cone. Standard value is: " << cone_topic_);
  }
  if (!pnh_.param<std::string>("topics/car_state", car_state_topic_, "localization/car_state"))
  {
    ROS_WARN_STREAM("Did not load topics/car_state. Standard value is: " << car_state_topic_);
  }
  if (!pnh_.param<std::string>("topics/log_path", log_path_topic_, "planning/skidpad/log_path"))
  {
    ROS_WARN_STREAM("Did not load topics/log_path. Standard value is: " << log_path_topic_);
  }
  if (!pnh_.param<std::string>("topics/approaching_goal", approaching_goal_topic_, "planning/skidpad/approaching_goal"))
  {
    ROS_WARN_STREAM("Did not load topics/approaching_goal. Standard value is: " << approaching_goal_topic_);
  }
}

void SkidpadDetectionNode::ConeCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &skidpad_msg)
{
  std::vector<planning_core::ConePoint> cones;
  cones.reserve(skidpad_msg->points.size());

  for (const geometry_msgs::Point32 &point : skidpad_msg->points)
  {
    planning_core::ConePoint cone;
    cone.x = point.x;
    cone.y = point.y;
    cone.z = point.z;
    cones.push_back(cone);
  }

  core_.ProcessConeDetections(cones);
}

void SkidpadDetectionNode::CarStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &carposition)
{
  planning_core::Trajectory state;
  state.x = carposition->car_state.x;
  state.y = carposition->car_state.y;
  state.yaw = carposition->car_state.theta;
  state.v = carposition->V;
  core_.UpdateVehicleState(state);
}

void SkidpadDetectionNode::PublishPath(const std::vector<planning_core::Pose> &path_points)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "world";  // 全局坐标系
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

  log_path_pub_.publish(path_msg);
}

void SkidpadDetectionNode::PublishApproachingGoal(bool approaching)
{
  std_msgs::Bool msg;
  msg.data = approaching;
  approaching_goal_pub_.publish(msg);
}

void SkidpadDetectionNode::RunOnce()
{
  core_.RunAlgorithm();

  if (core_.HasNewPath())
  {
    PublishPath(core_.GetPath());
    core_.ClearPathUpdated();
  }

  PublishApproachingGoal(core_.IsApproachingGoal());
}

} // namespace planning_ros
