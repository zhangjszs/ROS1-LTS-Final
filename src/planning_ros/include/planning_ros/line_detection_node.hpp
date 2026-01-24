#ifndef PLANNING_ROS_LINE_DETECTION_NODE_HPP_
#define PLANNING_ROS_LINE_DETECTION_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "planning_core/line_detection_core.hpp"

namespace planning_ros
{

class LineDetectionNode
{
public:
  explicit LineDetectionNode(ros::NodeHandle &nh);

  void RunOnce();
  bool IsFinished() const { return core_.IsFinished(); }

private:
  void LoadParameters();
  void ConeCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &cone_msg);
  void CarStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &car_state);
  void PublishPath(const std::vector<planning_core::Pose> &path_points);
  void PublishFinishOnce();

  ros::NodeHandle nh_;
  ros::Subscriber cone_sub_;
  ros::Subscriber car_state_sub_;
  ros::Publisher path_pub_;
  ros::Publisher finish_pub_;

  planning_core::LineDetectionParams params_;
  planning_core::LineDetectionCore core_;

  std::string cone_topic_;
  std::string car_state_topic_;
  std::string path_topic_;

  bool finish_published_{false};
  std::mutex data_mutex_;
};

} // namespace planning_ros

#endif // PLANNING_ROS_LINE_DETECTION_NODE_HPP_
