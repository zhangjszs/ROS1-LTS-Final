#ifndef PLANNING_ROS_LINE_DETECTION_NODE_HPP_
#define PLANNING_ROS_LINE_DETECTION_NODE_HPP_

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
  using ConeMsg = autodrive_msgs::HUAT_ConeDetections;
  using StateMsg = autodrive_msgs::HUAT_CarState;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ConeMsg, StateMsg>;

  void LoadParameters();
  void SyncCallback(const ConeMsg::ConstPtr &cone_msg,
                    const StateMsg::ConstPtr &car_state);
  void PublishPath(const std::vector<planning_core::Pose> &path_points);
  void PublishPathLimits(const std::vector<planning_core::Pose> &path_points);
  void FillPathDynamics(autodrive_msgs::HUAT_PathLimits &msg) const;
  void PublishFinishOnce();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 消息同步订阅
  message_filters::Subscriber<ConeMsg> cone_sub_;
  message_filters::Subscriber<StateMsg> car_state_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  ros::Publisher path_pub_;
  ros::Publisher pathlimits_pub_;
  ros::Publisher finish_pub_;

  planning_core::LineDetectionParams params_;
  planning_core::LineDetectionCore core_;

  std::string cone_topic_;
  std::string car_state_topic_;
  std::string path_topic_;
  std::string pathlimits_topic_;
  std::string finish_topic_;
  std::string expected_cone_frame_;
  std::string output_frame_;

  ros::Time latest_sync_time_;
  double max_data_age_ = 0.5;  // 数据过期阈值 (秒)
  double speed_cap_ = 20.0;
  double max_lateral_acc_ = 6.5;
  double max_accel_ = 3.0;
  double max_brake_ = 4.0;
  double min_speed_ = 1.0;
  double curvature_epsilon_ = 1e-3;

  bool finish_published_{false};
  std::mutex data_mutex_;
};

} // namespace planning_ros

#endif // PLANNING_ROS_LINE_DETECTION_NODE_HPP_
