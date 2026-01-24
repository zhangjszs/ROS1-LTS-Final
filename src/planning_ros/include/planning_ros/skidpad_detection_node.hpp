#ifndef PLANNING_ROS_SKIDPAD_DETECTION_NODE_HPP_
#define PLANNING_ROS_SKIDPAD_DETECTION_NODE_HPP_

#include <string>
#include <vector>

#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "planning_core/skidpad_detection_core.hpp"

namespace planning_ros
{

class SkidpadDetectionNode
{
public:
  explicit SkidpadDetectionNode(ros::NodeHandle &nh);

  void RunOnce();

private:
  void LoadParameters();
  void ConeCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &skidpad_msg);
  void CarStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &carposition);
  void PublishPath(const std::vector<planning_core::Pose> &path_points);
  void PublishApproachingGoal(bool approaching);

  ros::NodeHandle nh_;
  ros::Subscriber cone_sub_;
  ros::Subscriber car_state_sub_;
  ros::Publisher log_path_pub_;
  ros::Publisher approaching_goal_pub_;

  planning_core::SkidpadParams params_{};
  planning_core::SkidpadDetectionCore core_;

  std::string filtered_topic_name_;
  int inverse_flag_{1};
};

} // namespace planning_ros

#endif // PLANNING_ROS_SKIDPAD_DETECTION_NODE_HPP_
