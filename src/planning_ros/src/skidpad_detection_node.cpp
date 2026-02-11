#include "planning_ros/skidpad_detection_node.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <vector>

#include "planning_core/speed_profile.hpp"

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
  pnh_.param("circle/center_distance_nominal", params_.center_distance_nominal, 18.25);
  pnh_.param("circle/center_distance_tolerance", params_.center_distance_tolerance, 6.0);

  pnh_.param("fit/ransac_iterations", params_.fit_ransac_iterations, 120);
  pnh_.param("fit/inlier_threshold", params_.fit_inlier_threshold, 0.35);
  pnh_.param("fit/min_inliers", params_.fit_min_inliers, 4);
  pnh_.param("fit/radius_min", params_.fit_radius_min, 5.0);
  pnh_.param("fit/radius_max", params_.fit_radius_max, 20.0);

  pnh_.param("phase/min_dwell_frames", params_.phase_min_dwell_frames, 5);
  pnh_.param("phase/entry_switch_dist", params_.phase_entry_switch_dist, 1.2);
  pnh_.param("phase/crossover_switch_dist", params_.phase_crossover_switch_dist, 1.5);
  pnh_.param("phase/exit_switch_dist", params_.phase_exit_switch_dist, 2.0);

  pnh_.param("phase_speed/entry", params_.speed_entry, 6.0);
  pnh_.param("phase_speed/warmup", params_.speed_warmup, 7.0);
  pnh_.param("phase_speed/timed", params_.speed_timed, 8.0);
  pnh_.param("phase_speed/crossover", params_.speed_crossover, 6.5);
  pnh_.param("phase_speed/exit", params_.speed_exit, 5.0);
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

void SkidpadDetectionNode::FillPathDynamics(autodrive_msgs::HUAT_PathLimits &msg) const
{
  const size_t n = msg.path.size();
  if (n == 0)
  {
    msg.curvatures.clear();
    msg.target_speeds.clear();
    return;
  }

  // Convert to planning_core::Point2D for shared module
  std::vector<planning_core::Point2D> pts(n);
  for (size_t i = 0; i < n; ++i)
  {
    pts[i].x = msg.path[i].x;
    pts[i].y = msg.path[i].y;
  }

  // Curvatures via shared module
  planning_core::ComputeCurvatures(pts, msg.curvatures);

  // Speed cap: min of global cap and phase-aware cap from core
  const double phase_cap = std::max(0.0, core_.GetRecommendedSpeedCap());
  const double effective_cap = std::min(speed_cap_, phase_cap > 0.0 ? phase_cap : speed_cap_);

  planning_core::SpeedProfileParams sp;
  sp.speed_cap = effective_cap;
  sp.max_lateral_acc = max_lateral_acc_;
  sp.max_accel = max_accel_;
  sp.max_brake = max_brake_;
  sp.min_speed = min_speed_;
  sp.curvature_epsilon = curvature_epsilon_;
  sp.current_speed = 0.0;  // skidpad doesn't seed from vehicle speed
  planning_core::ComputeSpeedProfile(pts, msg.curvatures, sp, msg.target_speeds);
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

  const planning_core::SkidpadPhase phase = core_.GetPhase();
  if (!phase_initialized_ || phase != last_phase_)
  {
    ROS_INFO_STREAM("[Skidpad] Phase -> " << core_.GetPhaseName()
                    << ", speed_cap=" << core_.GetRecommendedSpeedCap());
    last_phase_ = phase;
    phase_initialized_ = true;
  }

  if (core_.HasNewPath())
  {
    PublishPathLimits(core_.GetPath());
    core_.ClearPathUpdated();
  }

  PublishApproachingGoal(core_.IsApproachingGoal());
}

} // namespace planning_ros
