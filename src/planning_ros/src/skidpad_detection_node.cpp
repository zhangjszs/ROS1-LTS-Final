#include "planning_ros/skidpad_detection_node.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace
{
double PointDistance(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

double SignedCurvature(const geometry_msgs::Point &p0,
                       const geometry_msgs::Point &p1,
                       const geometry_msgs::Point &p2)
{
  const double a = PointDistance(p0, p1);
  const double b = PointDistance(p1, p2);
  const double c = PointDistance(p0, p2);
  const double denom = a * b * c;
  if (denom < 1e-6)
  {
    return 0.0;
  }
  const double cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
  return 2.0 * cross / denom;
}
} // namespace

namespace planning_ros
{

SkidpadDetectionNode::SkidpadDetectionNode(ros::NodeHandle &nh)
  : nh_(nh), pnh_("~"), core_(params_)
{
  LoadParameters();
  core_.SetParams(params_);

  // ApproximateTime消息同步：确保锥桶检测和车辆状态时间对齐
  cone_sub_.subscribe(nh_, cone_topic_, 10);
  car_state_sub_.subscribe(nh_, car_state_topic_, 10);
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cone_sub_, car_state_sub_);
  sync_->registerCallback(
      boost::bind(&SkidpadDetectionNode::SyncCallback, this, _1, _2));

  log_path_pub_ = nh_.advertise<nav_msgs::Path>(log_path_topic_, 10, true);
  pathlimits_pub_ = nh_.advertise<autodrive_msgs::HUAT_PathLimits>(pathlimits_topic_, 10, true);
  approaching_goal_pub_ = nh_.advertise<std_msgs::Bool>(approaching_goal_topic_, 10);

  pnh_.param("max_data_age", max_data_age_, 0.5);
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
    ROS_WARN_STREAM("Did not load leavedistanceThreshold. Standard value is " << params_.leavedistanceThreshold);
  }

  if (!pnh_.param("inverse_flag", params_.inverse_flag, 1))
  {
    ROS_WARN_STREAM("Did not load inverse_flag. Standard value is: " << params_.inverse_flag);
  }

  if (!pnh_.param("length/stopdistance", params_.stopdistance, 5.0))
  {
    ROS_WARN_STREAM("Did not load stopdistance. Standard value is: " << params_.stopdistance);
  }

  pnh_.param("circle/radius", params_.circle_radius, 9.125);
  pnh_.param("circle/car_length", params_.car_length, 1.87);
  pnh_.param("circle/path_interval", params_.path_interval, 0.05);
  pnh_.param("passthrough/x_min", params_.passthrough_x_min, 0.1);
  pnh_.param("passthrough/x_max", params_.passthrough_x_max, 15.0);
  pnh_.param("passthrough/y_min", params_.passthrough_y_min, -3.0);
  pnh_.param("passthrough/y_max", params_.passthrough_y_max, 3.0);

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
  if (!pnh_.param<std::string>("topics/pathlimits", pathlimits_topic_, "planning/skidpad/pathlimits"))
  {
    ROS_WARN_STREAM("Did not load topics/pathlimits. Standard value is: " << pathlimits_topic_);
  }
  if (!pnh_.param<std::string>("topics/approaching_goal", approaching_goal_topic_, "planning/skidpad/approaching_goal"))
  {
    ROS_WARN_STREAM("Did not load topics/approaching_goal. Standard value is: " << approaching_goal_topic_);
  }

  pnh_.param("speed/speed_cap", speed_cap_, 8.0);
  pnh_.param("speed/max_lateral_acc", max_lateral_acc_, 6.5);
  pnh_.param("speed/max_accel", max_accel_, 2.5);
  pnh_.param("speed/max_brake", max_brake_, 4.0);
  pnh_.param("speed/min_speed", min_speed_, 1.0);
  pnh_.param("speed/curvature_epsilon", curvature_epsilon_, 1e-3);
}

void SkidpadDetectionNode::SyncCallback(const ConeMsg::ConstPtr &cone_msg,
                                         const StateMsg::ConstPtr &car_state)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_sync_time_ = cone_msg->header.stamp;

  // 时间戳验证
  double time_diff = std::abs((cone_msg->header.stamp - car_state->header.stamp).toSec());
  if (time_diff > max_data_age_)
  {
    ROS_WARN_THROTTLE(1.0, "[Skidpad] Large time diff between cone and state: %.3fms", time_diff * 1000.0);
  }

  // 处理锥桶数据
  std::vector<planning_core::ConePoint> cones;
  cones.reserve(cone_msg->points.size());

  for (const geometry_msgs::Point32 &point : cone_msg->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }
    planning_core::ConePoint cone;
    cone.x = point.x;
    cone.y = point.y;
    cone.z = point.z;
    cones.push_back(cone);
  }

  core_.ProcessConeDetections(cones);

  // 处理车辆状态
  planning_core::Trajectory state;
  state.x = car_state->car_state.x;
  state.y = car_state->car_state.y;
  state.yaw = car_state->car_state.theta;
  state.v = car_state->V;
  core_.UpdateVehicleState(state);
}

void SkidpadDetectionNode::PublishPath(const std::vector<planning_core::Pose> &path_points)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "world";  // 全局坐标系
  path_msg.header.stamp = latest_sync_time_;
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

void SkidpadDetectionNode::FillPathDynamics(autodrive_msgs::HUAT_PathLimits &msg) const
{
  const size_t n = msg.path.size();
  msg.curvatures.assign(n, 0.0);
  msg.target_speeds.assign(n, 0.0);
  if (n == 0)
  {
    return;
  }

  if (n >= 3)
  {
    for (size_t i = 1; i + 1 < n; ++i)
    {
      msg.curvatures[i] = SignedCurvature(msg.path[i - 1], msg.path[i], msg.path[i + 1]);
    }
    msg.curvatures.front() = msg.curvatures[1];
    msg.curvatures.back() = msg.curvatures[n - 2];
  }

  const double v_cap = std::max(0.0, speed_cap_);
  const double a_lat = std::max(0.1, max_lateral_acc_);
  const double a_acc = std::max(0.0, max_accel_);
  const double a_brake = std::max(0.0, max_brake_);
  const double kappa_eps = std::max(1e-6, curvature_epsilon_);
  const double v_min = std::max(0.0, std::min(min_speed_, v_cap));

  std::vector<double> ds;
  if (n >= 2)
  {
    ds.resize(n - 1, 1e-3);
    for (size_t i = 0; i + 1 < n; ++i)
    {
      ds[i] = std::max(1e-3, PointDistance(msg.path[i], msg.path[i + 1]));
    }
  }

  std::vector<double> v_ref(n, v_cap);
  for (size_t i = 0; i < n; ++i)
  {
    const double abs_kappa = std::abs(msg.curvatures[i]);
    const double denom = std::max(abs_kappa, kappa_eps);
    const double v_lat_max = std::sqrt(std::max(0.0, a_lat / denom));
    v_ref[i] = std::max(v_min, std::min(v_cap, v_lat_max));
  }

  for (size_t i = 1; i < n; ++i)
  {
    const double reachable = std::sqrt(std::max(0.0, v_ref[i - 1] * v_ref[i - 1] + 2.0 * a_acc * ds[i - 1]));
    v_ref[i] = std::min(v_ref[i], reachable);
  }
  for (size_t i = n - 1; i > 0; --i)
  {
    const double reachable = std::sqrt(std::max(0.0, v_ref[i] * v_ref[i] + 2.0 * a_brake * ds[i - 1]));
    v_ref[i - 1] = std::min(v_ref[i - 1], reachable);
  }

  for (size_t i = 0; i < n; ++i)
  {
    if (!std::isfinite(msg.curvatures[i]))
    {
      msg.curvatures[i] = 0.0;
    }
    msg.target_speeds[i] = (std::isfinite(v_ref[i]) && v_ref[i] > 0.0) ? v_ref[i] : 0.0;
  }
}

void SkidpadDetectionNode::PublishPathLimits(const std::vector<planning_core::Pose> &path_points)
{
  autodrive_msgs::HUAT_PathLimits msg;
  msg.header.stamp = latest_sync_time_;
  msg.header.frame_id = "world";
  msg.stamp = latest_sync_time_;
  msg.replan = true;
  msg.path.reserve(path_points.size());

  for (const auto &pose : path_points)
  {
    geometry_msgs::Point p;
    p.x = pose.x;
    p.y = pose.y;
    p.z = pose.z;
    msg.path.push_back(p);
  }

  msg.tracklimits.stamp = msg.stamp;
  msg.tracklimits.replan = msg.replan;

  FillPathDynamics(msg);
  pathlimits_pub_.publish(msg);
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
    PublishPathLimits(core_.GetPath());
    core_.ClearPathUpdated();
  }

  PublishApproachingGoal(core_.IsApproachingGoal());
}

} // namespace planning_ros
