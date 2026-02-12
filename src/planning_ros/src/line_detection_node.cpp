#include "planning_ros/line_detection_node.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <vector>

#include "planning_core/speed_profile.hpp"
#include "planning_ros/contract_utils.hpp"

namespace
{
double PointDistance(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}
} // namespace

namespace planning_ros
{

LineDetectionNode::LineDetectionNode(ros::NodeHandle &nh)
  : nh_(nh), pnh_("~"), core_(params_)
{
  LoadParameters();
  core_.SetParams(params_);

  // ApproximateTime消息同步：确保锥桶检测和车辆状态时间对齐
  cone_sub_.subscribe(nh_, cone_topic_, 1);
  car_state_sub_.subscribe(nh_, car_state_topic_, 1);
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cone_sub_, car_state_sub_);
  sync_->registerCallback(
      boost::bind(&LineDetectionNode::SyncCallback, this, _1, _2));

  pathlimits_pub_ = nh_.advertise<autodrive_msgs::HUAT_PathLimits>(pathlimits_topic_, 1);
  finish_pub_ = nh_.advertise<std_msgs::Bool>(finish_topic_, 1);

  pnh_.param("max_data_age", max_data_age_, 0.5);

  ROS_INFO("[LineDetection] Node initialized with ApproximateTime sync");
}

void LineDetectionNode::LoadParameters()
{
  pnh_.param("hough/rho_resolution", params_.hough_rho_resolution, 0.1);
  pnh_.param("hough/theta_resolution", params_.hough_theta_resolution, 0.01);
  pnh_.param("hough/min_votes", params_.hough_min_votes, 3);
  pnh_.param("hough/theta_tolerance", params_.theta_tolerance, 0.2);

  pnh_.param("cones/max_distance", params_.max_cone_distance, 50.0);
  pnh_.param("cones/min_distance", params_.min_cone_distance, 2.0);
  pnh_.param("cones/max_lateral_distance", params_.max_cone_lateral_distance, 10.0);
  pnh_.param("cones/min_valid_count", params_.min_valid_cones, 2);
  pnh_.param("cones/hold_on_sparse_cones", params_.hold_on_sparse_cones, true);

  pnh_.param("hough/min_rho_diff", params_.min_rho_diff, 2.0);
  pnh_.param("hough/max_rho_diff", params_.max_rho_diff, 20.0);
  pnh_.param("hough/max_lines_to_check", params_.max_lines_to_check, 10);

  pnh_.param("path/interval", params_.path_interval, 0.1);
  pnh_.param("path/max_distance", params_.max_path_distance, 175.0);
  pnh_.param("path/start_x", params_.path_start_x, 1.0);
  pnh_.param("path/accel_distance", params_.accel_distance, 75.0);
  pnh_.param("path/brake_distance", params_.brake_distance, 100.0);

  pnh_.param("vehicle/imu_offset_x", params_.imu_offset_x, 1.88);

  pnh_.param("finish/threshold", params_.finish_line_threshold, 2.0);

  pnh_.param<std::string>("topics/cone", cone_topic_, "perception/lidar_cluster/detections");
  pnh_.param<std::string>("topics/car_state", car_state_topic_, "localization/car_state");
  pnh_.param<std::string>("topics/pathlimits", pathlimits_topic_, "planning/line_detection/pathlimits");
  pnh_.param<std::string>("topics/finish", finish_topic_, "planning/line_detection/finish_signal");

  pnh_.param<std::string>("frames/expected_cone", expected_cone_frame_, "velodyne");
  pnh_.param<std::string>("frames/output", output_frame_, "world");

  pnh_.param("speed/speed_cap", speed_cap_, 20.0);
  pnh_.param("speed/max_lateral_acc", max_lateral_acc_, 6.5);
  pnh_.param("speed/max_accel", max_accel_, 3.0);
  pnh_.param("speed/max_brake", max_brake_, 4.0);
  pnh_.param("speed/min_speed", min_speed_, 1.0);
  pnh_.param("speed/curvature_epsilon", curvature_epsilon_, 1e-3);
  pnh_.param("speed/accel_zone_length", accel_zone_length_, 75.0);
  pnh_.param("speed/brake_zone_length", brake_zone_length_, 100.0);
  pnh_.param("speed/timing_start_offset", timing_start_offset_, 0.3);

  if (params_.accel_distance <= 0.0)
  {
    params_.accel_distance = accel_zone_length_;
  }
  if (params_.brake_distance <= 0.0)
  {
    params_.brake_distance = brake_zone_length_;
  }
  const double min_total = params_.accel_distance + params_.brake_distance;
  if (params_.max_path_distance < min_total)
  {
    params_.max_path_distance = min_total;
  }
}

void LineDetectionNode::SyncCallback(const ConeMsg::ConstPtr &cone_msg,
                                      const StateMsg::ConstPtr &car_state)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 时间戳验证
  double time_diff = std::abs((cone_msg->header.stamp - car_state->header.stamp).toSec());
  if (time_diff > max_data_age_)
  {
    ROS_WARN_THROTTLE(1.0, "[LineDetection] Large time diff between cone (%.3f) and state (%.3f): %.3fms",
                      cone_msg->header.stamp.toSec(), car_state->header.stamp.toSec(), time_diff * 1000.0);
  }

  latest_sync_time_ = std::max(cone_msg->header.stamp, car_state->header.stamp);

  // 处理锥桶数据
  if (!cone_msg->header.frame_id.empty() &&
      cone_msg->header.frame_id != expected_cone_frame_ &&
      cone_msg->header.frame_id != "velodyne")
  {
    ROS_ERROR_THROTTLE(1.0, "[LineDetection] Unexpected cone frame: %s (expected: %s)",
                       cone_msg->header.frame_id.c_str(), expected_cone_frame_.c_str());
    return;
  }

  if (cone_msg->points.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[LineDetection] Received empty cone message");
    core_.UpdateCones({});
  }
  else
  {
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

    core_.UpdateCones(cones);
  }

  // 处理车辆状态
  if (!std::isfinite(car_state->car_state.x) ||
      !std::isfinite(car_state->car_state.y) ||
      !std::isfinite(car_state->car_state.theta) ||
      !std::isfinite(car_state->V))
  {
    ROS_ERROR("[LineDetection] Invalid vehicle state (NaN/Inf), ignoring update");
    return;
  }

  planning_core::VehicleState state;
  state.x = car_state->car_state.x;
  state.y = car_state->car_state.y;
  state.theta = car_state->car_state.theta;
  state.v = car_state->V;
  latest_vehicle_speed_ = std::isfinite(car_state->V) ? std::max(0.0, static_cast<double>(car_state->V)) : 0.0;

  core_.UpdateVehicleState(state);
}

void LineDetectionNode::FillPathDynamics(autodrive_msgs::HUAT_PathLimits &msg) const
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

  // Base speed profile via shared module
  planning_core::SpeedProfileParams sp;
  sp.speed_cap = speed_cap_;
  sp.max_lateral_acc = max_lateral_acc_;
  sp.max_accel = max_accel_;
  sp.max_brake = max_brake_;
  sp.min_speed = min_speed_;
  sp.curvature_epsilon = curvature_epsilon_;
  sp.current_speed = latest_vehicle_speed_;
  planning_core::ComputeSpeedProfile(pts, msg.curvatures, sp, msg.target_speeds);

  // Acceleration-mission zone overlay: accel zone + brake zone
  const double v_cap = std::max(0.0, speed_cap_);
  const double a_brake = std::max(0.0, max_brake_);
  std::vector<double> ds(n >= 2 ? n - 1 : 0, 1e-3);
  for (size_t i = 0; i + 1 < n; ++i)
    ds[i] = std::max(1e-3, PointDistance(msg.path[i], msg.path[i + 1]));

  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; ++i)
    s[i] = s[i - 1] + ds[i - 1];

  const double accel_end_s = std::max(0.0, timing_start_offset_ + accel_zone_length_);
  const double brake_end_s = std::max(accel_end_s, accel_end_s + brake_zone_length_);

  for (size_t i = 0; i < n; ++i)
  {
    if (s[i] >= brake_end_s)
    {
      msg.target_speeds[i] = 0.0;
      continue;
    }
    if (s[i] >= accel_end_s)
    {
      const double remaining = std::max(0.0, brake_end_s - s[i]);
      const double brake_cap = std::sqrt(std::max(0.0, 2.0 * a_brake * remaining));
      msg.target_speeds[i] = std::min(msg.target_speeds[i], brake_cap);
    }
    else if (s[i] > accel_end_s)
    {
      msg.target_speeds[i] = std::min(msg.target_speeds[i], v_cap);
    }
  }

  // Sanitize
  for (size_t i = 0; i < n; ++i)
  {
    if (!std::isfinite(msg.curvatures[i]))
      msg.curvatures[i] = 0.0;
    if (!std::isfinite(msg.target_speeds[i]) || msg.target_speeds[i] < 0.0)
      msg.target_speeds[i] = 0.0;
  }
}

void LineDetectionNode::PublishPathLimits(const std::vector<planning_core::Pose> &path_points)
{
  autodrive_msgs::HUAT_PathLimits msg;
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

  msg.tracklimits.replan = msg.replan;

  FillPathDynamics(msg);
  contract::EnforcePathDynamicsShape(msg);
  contract::FinalizePathLimitsMessage(msg, latest_sync_time_, output_frame_);
  pathlimits_pub_.publish(msg);
}

void LineDetectionNode::PublishFinishOnce()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

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
  std::string error;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    core_.RunAlgorithm();
    finished = core_.IsFinished();
    error = core_.GetLastError();
    if (core_.HasPlannedPath())
    {
      planned_path = core_.GetPlannedPath();
    }
  }

  if (!error.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[LineDetection] Planning failed: %s", error.c_str());
  }

  if (!finished && !planned_path.empty())
  {
    PublishPathLimits(planned_path);
  }

  if (finished)
  {
    PublishFinishOnce();
  }
}

} // namespace planning_ros
