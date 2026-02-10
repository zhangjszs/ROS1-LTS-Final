#include "planning_ros/line_detection_node.hpp"

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

  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
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

  pnh_.param("hough/min_rho_diff", params_.min_rho_diff, 2.0);
  pnh_.param("hough/max_rho_diff", params_.max_rho_diff, 20.0);
  pnh_.param("hough/max_lines_to_check", params_.max_lines_to_check, 10);

  pnh_.param("path/interval", params_.path_interval, 0.1);
  pnh_.param("path/max_distance", params_.max_path_distance, 75.0);
  pnh_.param("path/start_x", params_.path_start_x, 1.0);

  pnh_.param("vehicle/imu_offset_x", params_.imu_offset_x, 1.88);

  pnh_.param("finish/threshold", params_.finish_line_threshold, 2.0);

  pnh_.param<std::string>("topics/cone", cone_topic_, "perception/lidar_cluster/detections");
  pnh_.param<std::string>("topics/car_state", car_state_topic_, "localization/car_state");
  pnh_.param<std::string>("topics/path", path_topic_, "planning/line_detection/path");
  pnh_.param<std::string>("topics/pathlimits", pathlimits_topic_, "planning/line_detection/pathlimits");
  pnh_.param<std::string>("topics/finish", finish_topic_, "planning/line_detection/finish_signal");

  pnh_.param<std::string>("frames/expected_cone", expected_cone_frame_, "base_link");
  pnh_.param<std::string>("frames/output", output_frame_, "world");

  pnh_.param("speed/speed_cap", speed_cap_, 20.0);
  pnh_.param("speed/max_lateral_acc", max_lateral_acc_, 6.5);
  pnh_.param("speed/max_accel", max_accel_, 3.0);
  pnh_.param("speed/max_brake", max_brake_, 4.0);
  pnh_.param("speed/min_speed", min_speed_, 1.0);
  pnh_.param("speed/curvature_epsilon", curvature_epsilon_, 1e-3);
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

  core_.UpdateVehicleState(state);
}

void LineDetectionNode::PublishPath(const std::vector<planning_core::Pose> &path_points)
{
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = output_frame_;
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

  path_pub_.publish(path_msg);
}

void LineDetectionNode::FillPathDynamics(autodrive_msgs::HUAT_PathLimits &msg) const
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

void LineDetectionNode::PublishPathLimits(const std::vector<planning_core::Pose> &path_points)
{
  autodrive_msgs::HUAT_PathLimits msg;
  msg.header.stamp = latest_sync_time_;
  msg.header.frame_id = output_frame_;
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
    PublishPath(planned_path);
    PublishPathLimits(planned_path);
  }

  if (finished)
  {
    PublishFinishOnce();
  }
}

} // namespace planning_ros
