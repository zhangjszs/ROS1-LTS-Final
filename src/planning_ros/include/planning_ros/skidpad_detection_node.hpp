#ifndef PLANNING_ROS_SKIDPAD_DETECTION_NODE_HPP_
#define PLANNING_ROS_SKIDPAD_DETECTION_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_PathLimits.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
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
  using ConeMsg = autodrive_msgs::HUAT_ConeDetections;
  using StateMsg = autodrive_msgs::HUAT_CarState;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ConeMsg, StateMsg>;

  void LoadParameters();
  void SyncCallback(const ConeMsg::ConstPtr &cone_msg,
                    const StateMsg::ConstPtr &car_state);
  void PublishPath(const std::vector<planning_core::Pose> &path_points);
  void PublishPathLimits(const std::vector<planning_core::Pose> &path_points);
  void FillPathDynamics(autodrive_msgs::HUAT_PathLimits &msg) const;
  void PublishApproachingGoal(bool approaching);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 消息同步订阅
  message_filters::Subscriber<ConeMsg> cone_sub_;
  message_filters::Subscriber<StateMsg> car_state_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  ros::Publisher log_path_pub_;
  ros::Publisher pathlimits_pub_;
  ros::Publisher approaching_goal_pub_;

  planning_core::SkidpadParams params_{};
  planning_core::SkidpadDetectionCore core_;

  std::string cone_topic_;
  std::string car_state_topic_;
  std::string log_path_topic_;
  std::string pathlimits_topic_;
  std::string approaching_goal_topic_;
  double max_data_age_ = 0.5;  // 数据过期阈值 (秒)
  double speed_cap_ = 8.0;
  double max_lateral_acc_ = 6.5;
  double max_accel_ = 2.5;
  double max_brake_ = 4.0;
  double min_speed_ = 1.0;
  double curvature_epsilon_ = 1e-3;
  ros::Time latest_sync_time_;
  std::mutex data_mutex_;
};

} // namespace planning_ros

#endif // PLANNING_ROS_SKIDPAD_DETECTION_NODE_HPP_
