#include <localization_ros/location_node.hpp>

#include <algorithm>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>

namespace localization_ros {

namespace {
template <typename T>
bool LoadParam(ros::NodeHandle &pnh, ros::NodeHandle &nh, const std::string &key, T &out, const T &default_value)
{
  if (pnh.param(key, out, default_value))
  {
    return true;
  }
  if (nh.param(key, out, default_value))
  {
    return true;
  }
  return false;
}

geometry_msgs::Quaternion YawToQuaternion(double yaw)
{
  geometry_msgs::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}
}  // namespace

LocationNode::LocationNode(ros::NodeHandle &nh)
    : nh_(nh),
      pnh_("~"),
      mapper_(params_)
{
  loadParameters();

  mapper_.Configure(params_);
  mapper_.SetDataDirectory(ros::package::getPath("localization_ros"));

  if (use_external_carstate_)
  {
    carstate_sub_ = nh_.subscribe<autodrive_msgs::HUAT_CarState>(carstate_in_topic_, 10, &LocationNode::carstateCallback, this);
  }
  else
  {
    ins_sub_ = nh_.subscribe<autodrive_msgs::HUAT_InsP2>(ins_topic_, 10, &LocationNode::imuCallback, this);
  }

  cone_sub_ = nh_.subscribe<autodrive_msgs::HUAT_ConeDetections>(cone_topic_, 10, &LocationNode::coneCallback, this);

  carstate_pub_ = nh_.advertise<autodrive_msgs::HUAT_CarState>(carstate_out_topic_, 10);
  map_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeMap>(cone_map_topic_, 10);
  global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_, 10);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

  // Initialize factor graph backend if configured
  if (backend_ == "factor_graph")
  {
    fg_optimizer_ = std::make_unique<localization_core::FactorGraphOptimizer>(fg_config_);
    ROS_INFO("Factor graph backend enabled (shadow mode)");
  }

}

void LocationNode::loadParameters()
{
  if (!LoadParam(pnh_, nh_, "length/lidarToIMUDist", params_.lidar_to_imu_dist, 1.87))
  {
    ROS_WARN_STREAM("Did not load lidarToIMUDist. Standard value is: " << params_.lidar_to_imu_dist);
  }
  if (!LoadParam(pnh_, nh_, "length/frontToIMUdistanceX", params_.front_to_imu_x, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceX. Standard value is: " << params_.front_to_imu_x);
  }
  if (!LoadParam(pnh_, nh_, "length/frontToIMUdistanceY", params_.front_to_imu_y, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceY. Standard value is " << params_.front_to_imu_y);
  }
  if (!LoadParam(pnh_, nh_, "length/frontToIMUdistanceZ", params_.front_to_imu_z, 0.0))
  {
    ROS_WARN_STREAM("Did not load frontToIMUdistanceZ. Standard value is: " << params_.front_to_imu_z);
  }
  if (!LoadParam(pnh_, nh_, "length/rearToIMUdistanceX", params_.rear_to_imu_x, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceX. Standard value is " << params_.rear_to_imu_x);
  }
  if (!LoadParam(pnh_, nh_, "length/rearToIMUdistanceY", params_.rear_to_imu_y, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceY. Standard value is " << params_.rear_to_imu_y);
  }
  if (!LoadParam(pnh_, nh_, "length/rearToIMUdistanceZ", params_.rear_to_imu_z, 0.0))
  {
    ROS_WARN_STREAM("Did not load rearToIMUdistanceZ. Standard value is " << params_.rear_to_imu_z);
  }

  if (!LoadParam(pnh_, nh_, "topics/ins", ins_topic_, std::string("sensors/ins")))
  {
    ROS_WARN_STREAM("Did not load topics/ins. Standard value is: " << ins_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/car_state_in", carstate_in_topic_, std::string("localization/car_state")))
  {
    ROS_WARN_STREAM("Did not load topics/car_state_in. Standard value is: " << carstate_in_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/cone", cone_topic_, std::string("perception/lidar_cluster/detections")))
  {
    ROS_WARN_STREAM("Did not load topics/cone. Standard value is: " << cone_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/car_state_out", carstate_out_topic_, std::string("localization/car_state")))
  {
    ROS_WARN_STREAM("Did not load topics/car_state_out. Standard value is: " << carstate_out_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/cone_map", cone_map_topic_, std::string("localization/cone_map")))
  {
    ROS_WARN_STREAM("Did not load topics/cone_map. Standard value is: " << cone_map_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/global_map", global_map_topic_, std::string("localization/global_map")))
  {
    ROS_WARN_STREAM("Did not load topics/global_map. Standard value is: " << global_map_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/pose", pose_topic_, std::string("localization/pose")))
  {
    ROS_WARN_STREAM("Did not load topics/pose. Standard value is: " << pose_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/odom", odom_topic_, std::string("localization/odom")))
  {
    ROS_WARN_STREAM("Did not load topics/odom. Standard value is: " << odom_topic_);
  }
  if (!LoadParam(pnh_, nh_, "frames/world", world_frame_, std::string("world")))
  {
    ROS_WARN_STREAM("Did not load frames/world. Standard value is: " << world_frame_);
  }
  if (!LoadParam(pnh_, nh_, "frames/base_link", base_link_frame_, std::string("base_link")))
  {
    ROS_WARN_STREAM("Did not load frames/base_link. Standard value is: " << base_link_frame_);
  }
  if (!LoadParam(pnh_, nh_, "use_external_carstate", use_external_carstate_, false))
  {
    ROS_WARN_STREAM("Did not load use_external_carstate. Standard value is: " << (use_external_carstate_ ? "true" : "false"));
  }

  // 锥桶地图管理参数
  LoadParam(pnh_, nh_, "map/merge_distance", params_.merge_distance, 2.5);
  LoadParam(pnh_, nh_, "map/max_map_size", params_.max_map_size, 500);
  LoadParam(pnh_, nh_, "map/min_obs_to_keep", params_.min_obs_to_keep, 2);
  LoadParam(pnh_, nh_, "map/local_cone_range", params_.local_cone_range, 50.0);

  // 入图过滤参数
  LoadParam(pnh_, nh_, "map/min_confidence_to_add", params_.min_confidence_to_add, 0.3);
  LoadParam(pnh_, nh_, "map/min_confidence_to_merge", params_.min_confidence_to_merge, 0.15);
  LoadParam(pnh_, nh_, "map/max_cone_height", params_.max_cone_height, 0.75);
  LoadParam(pnh_, nh_, "map/max_cone_width", params_.max_cone_width, 0.5);
  LoadParam(pnh_, nh_, "map/min_cone_height", params_.min_cone_height, 0.03);

  // 赛道模式与几何约束
  LoadParam(pnh_, nh_, "map/mode", params_.map_mode, std::string("track"));
  LoadParam(pnh_, nh_, "map/cone_y_max", params_.cone_y_max, 0.0);
  LoadParam(pnh_, nh_, "map/expected_cone_spacing", params_.expected_cone_spacing, 0.0);
  LoadParam(pnh_, nh_, "map/track_width", params_.track_width, 3.0);
  LoadParam(pnh_, nh_, "map/enable_circle_validation", params_.enable_circle_validation, false);
  LoadParam(pnh_, nh_, "map/circle_radius", params_.circle_radius, 15.25);
  LoadParam(pnh_, nh_, "map/circle_center_dist", params_.circle_center_dist, 18.25);
  LoadParam(pnh_, nh_, "map/circle_tolerance", params_.circle_tolerance, 2.0);

  // 模式预设覆盖：从 map/mode_presets/<mode>/ 读取并覆盖对应参数
  const std::string preset_prefix = "map/mode_presets/" + params_.map_mode + "/";
  double tmp_d;
  int tmp_i;
  bool tmp_b;
  if (LoadParam(pnh_, nh_, preset_prefix + "merge_distance", tmp_d, params_.merge_distance))
    params_.merge_distance = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "max_map_size", tmp_i, params_.max_map_size))
    params_.max_map_size = tmp_i;
  if (LoadParam(pnh_, nh_, preset_prefix + "min_obs_to_keep", tmp_i, params_.min_obs_to_keep))
    params_.min_obs_to_keep = tmp_i;
  if (LoadParam(pnh_, nh_, preset_prefix + "local_cone_range", tmp_d, params_.local_cone_range))
    params_.local_cone_range = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "min_confidence_to_add", tmp_d, params_.min_confidence_to_add))
    params_.min_confidence_to_add = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "min_confidence_to_merge", tmp_d, params_.min_confidence_to_merge))
    params_.min_confidence_to_merge = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "cone_y_max", tmp_d, params_.cone_y_max))
    params_.cone_y_max = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "expected_cone_spacing", tmp_d, params_.expected_cone_spacing))
    params_.expected_cone_spacing = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "track_width", tmp_d, params_.track_width))
    params_.track_width = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "enable_circle_validation", tmp_b, params_.enable_circle_validation))
    params_.enable_circle_validation = tmp_b;
  if (LoadParam(pnh_, nh_, preset_prefix + "circle_radius", tmp_d, params_.circle_radius))
    params_.circle_radius = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "circle_center_dist", tmp_d, params_.circle_center_dist))
    params_.circle_center_dist = tmp_d;
  if (LoadParam(pnh_, nh_, preset_prefix + "circle_tolerance", tmp_d, params_.circle_tolerance))
    params_.circle_tolerance = tmp_d;

  ROS_INFO_STREAM("Localization map_mode: " << params_.map_mode);

  // Backend selection: "mapper" (default) or "factor_graph" (shadow mode)
  LoadParam(pnh_, nh_, "backend", backend_, std::string("mapper"));
  ROS_INFO_STREAM("Localization backend: " << backend_);

  // Factor graph config
  if (backend_ == "factor_graph")
  {
    LoadParam(pnh_, nh_, "fg/keyframe_dist", fg_config_.keyframe.dist_threshold, 1.0);
    LoadParam(pnh_, nh_, "fg/keyframe_yaw", fg_config_.keyframe.yaw_threshold, 0.1);
    LoadParam(pnh_, nh_, "fg/keyframe_dt", fg_config_.keyframe.dt_threshold, 0.5);
    LoadParam(pnh_, nh_, "fg/run_extra_update", fg_config_.run_extra_update, false);
    LoadParam(pnh_, nh_, "fg/sigma_imu_xy", fg_config_.sigma_imu_xy, 0.05);
    LoadParam(pnh_, nh_, "fg/sigma_imu_theta", fg_config_.sigma_imu_theta, 0.01);
    LoadParam(pnh_, nh_, "fg/sigma_gnss_good", fg_config_.sigma_gnss_good, 0.5);
    LoadParam(pnh_, nh_, "fg/sigma_gnss_medium", fg_config_.sigma_gnss_medium, 2.0);
    LoadParam(pnh_, nh_, "fg/sigma_gnss_poor", fg_config_.sigma_gnss_poor, 10.0);
    LoadParam(pnh_, nh_, "fg/min_cone_confidence", fg_config_.min_cone_confidence, 0.0);
    LoadParam(pnh_, nh_, "fg/max_cone_factors_per_keyframe", fg_config_.max_cone_factors_per_keyframe, 10);
    LoadParam(pnh_, nh_, "fg/color_mismatch_penalty", fg_config_.color_mismatch_penalty, 2.0);
    LoadParam(pnh_, nh_, "fg/merge_distance", fg_config_.merge_distance, 2.0);
  }
}

void LocationNode::publishState(const localization_core::CarState &state, const ros::Time &stamp, bool publish_carstate)
{
  const geometry_msgs::Quaternion orientation = YawToQuaternion(state.car_state.theta);
  double v_forward = 0.0;
  double yaw_rate = 0.0;
  if (has_last_state_ && stamp == last_stamp_)
  {
    // Avoid duplicate TF publishing
    return;
  }

  if (has_last_state_)
  {
    const double dt = (stamp - last_stamp_).toSec();
    if (dt > 1e-3)
    {
      const double dx = state.car_state.x - last_state_.car_state.x;
      const double dy = state.car_state.y - last_state_.car_state.y;
      const double yaw = state.car_state.theta;
      v_forward = (std::cos(yaw) * dx + std::sin(yaw) * dy) / dt;
      const double dtheta = std::atan2(std::sin(state.car_state.theta - last_state_.car_state.theta),
                                       std::cos(state.car_state.theta - last_state_.car_state.theta));
      yaw_rate = dtheta / dt;
    }
  }

  if (publish_carstate)
  {
    autodrive_msgs::HUAT_CarState out;
    ToRos(state, &out);
    out.header.stamp = stamp;
    out.header.frame_id = world_frame_;
    carstate_pub_.publish(out);
  }

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = world_frame_;
  pose_msg.pose.position.x = state.car_state.x;
  pose_msg.pose.position.y = state.car_state.y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation = orientation;
  pose_pub_.publish(pose_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = world_frame_;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.pose.pose = pose_msg.pose;
  odom_msg.twist.twist.linear.x = v_forward;
  odom_msg.twist.twist.angular.z = yaw_rate;
  odom_pub_.publish(odom_msg);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = world_frame_;
  tf_msg.child_frame_id = base_link_frame_;
  tf_msg.transform.translation.x = state.car_state.x;
  tf_msg.transform.translation.y = state.car_state.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = orientation;
  tf_broadcaster_.sendTransform(tf_msg);

  last_state_ = state;
  last_stamp_ = stamp;
  has_last_state_ = true;
}

void LocationNode::imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr &msg)
{
  localization_core::Asensing core_msg = ToCore(*msg);
  localization_core::CarState state;

  if (mapper_.UpdateFromIns(core_msg, &state))
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    publishState(state, stamp, true);
  }

  // Shadow-mode factor graph feeding
  if (fg_optimizer_)
  {
    feedFactorGraph(*msg);
  }
}

void LocationNode::carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msg)
{
  mapper_.UpdateFromCarState(ToCore(*msg));
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  const bool publish_carstate = carstate_out_topic_ != carstate_in_topic_;
  publishState(mapper_.car_state(), stamp, publish_carstate);
}

void LocationNode::coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &msg)
{
  if (!mapper_.has_carstate())
  {
    ROS_WARN_THROTTLE(5.0, "[location] INS data not updated yet, skipping cone update");
    return;
  }
  if (msg->points.empty())
  {
    ROS_WARN_THROTTLE(5.0, "[location] Cone coordinates empty, skipping");
    return;
  }

  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

  localization_core::ConeDetections detections = ToCore(*msg);
  localization_core::ConeMap map;
  localization_core::PointCloudPtr cloud;

  if (!mapper_.UpdateFromCones(detections, &map, &cloud))
  {
    return;
  }

  autodrive_msgs::HUAT_ConeMap out_map;
  ToRos(map, &out_map);
  out_map.header.stamp = stamp;
  out_map.header.frame_id = world_frame_;
  map_pub_.publish(out_map);

  if (cloud)
  {
    sensor_msgs::PointCloud2 global_cloud_msg;
    pcl::toROSMsg(*cloud, global_cloud_msg);
    global_cloud_msg.header.frame_id = world_frame_;
    global_cloud_msg.header.stamp = stamp;
    global_map_pub_.publish(global_cloud_msg);
  }

  // Shadow-mode factor graph feeding
  if (fg_optimizer_)
  {
    feedFactorGraphCones(*msg);
  }
}

localization_core::Asensing LocationNode::ToCore(const autodrive_msgs::HUAT_InsP2 &msg)
{
  localization_core::Asensing out;

  // 位置 (WGS84)
  out.latitude = msg.Lat;
  out.longitude = msg.Lon;
  out.altitude = msg.Altitude;

  // 速度 (m/s, NED坐标系)
  // HUAT_InsP2.Vd 向下为正
  out.north_velocity = msg.Vn;
  out.east_velocity = msg.Ve;
  out.ground_velocity = msg.Vd;  // Vd(向下)

  // 姿态角 (度)
  out.roll = msg.Roll;
  out.pitch = msg.Pitch;
  out.azimuth = msg.Heading;

  // 角速度 (rad/s, FRD车体系)
  // HUAT_InsP2中已经是 rad/s，直接使用
  out.x_angular_velocity = msg.gyro_x;
  out.y_angular_velocity = msg.gyro_y;
  out.z_angular_velocity = msg.gyro_z;

  // 加速度 (m/s², FRD车体系)
  // HUAT_InsP2中已经是 m/s²，直接使用
  out.x_acc = msg.acc_x;
  out.y_acc = msg.acc_y;
  out.z_acc = msg.acc_z;

  // 质量指标
  out.status = msg.Status;
  out.nsv1 = msg.NSV1;
  out.nsv2 = msg.NSV2;
  out.age = msg.Age;

  return out;
}

localization_core::CarState LocationNode::ToCore(const autodrive_msgs::HUAT_CarState &msg)
{
  localization_core::CarState out;
  out.car_state.x = msg.car_state.x;
  out.car_state.y = msg.car_state.y;
  out.car_state.theta = msg.car_state.theta;
  out.car_state_front.x = msg.car_state_front.x;
  out.car_state_front.y = msg.car_state_front.y;
  out.car_state_front.z = msg.car_state_front.z;
  out.car_state_rear.x = msg.car_state_rear.x;
  out.car_state_rear.y = msg.car_state_rear.y;
  out.car_state_rear.z = msg.car_state_rear.z;
  out.V = msg.V;
  out.W = msg.W;
  out.A = msg.A;
  return out;
}

void LocationNode::ToRos(const localization_core::CarState &state, autodrive_msgs::HUAT_CarState *out)
{
  if (!out)
  {
    return;
  }
  out->car_state.x = state.car_state.x;
  out->car_state.y = state.car_state.y;
  out->car_state.theta = state.car_state.theta;
  out->car_state_front.x = state.car_state_front.x;
  out->car_state_front.y = state.car_state_front.y;
  out->car_state_front.z = state.car_state_front.z;
  out->car_state_rear.x = state.car_state_rear.x;
  out->car_state_rear.y = state.car_state_rear.y;
  out->car_state_rear.z = state.car_state_rear.z;
  out->V = static_cast<float>(state.V);
  out->W = static_cast<float>(state.W);
  out->A = static_cast<float>(state.A);

  // FSSIM风格扩展状态
  out->Vy = static_cast<float>(state.Vy);
  out->Wz = static_cast<float>(state.Wz);
  out->Ax = static_cast<float>(state.Ax);
  out->Ay = static_cast<float>(state.Ay);
}

localization_core::ConeDetections LocationNode::ToCore(const autodrive_msgs::HUAT_ConeDetections &msg)
{
  localization_core::ConeDetections out;
  out.detections.reserve(msg.points.size());
  for (size_t i = 0; i < msg.points.size(); ++i)
  {
    localization_core::ConeDetection det;
    det.point.x = msg.points[i].x;
    det.point.y = msg.points[i].y;
    det.point.z = msg.points[i].z;

    // NaN/Inf 防护：跳过非有限坐标的检测
    if (!std::isfinite(det.point.x) || !std::isfinite(det.point.y) || !std::isfinite(det.point.z))
    {
      ROS_WARN_THROTTLE(5.0, "Skipping cone detection %zu with non-finite coordinates", i);
      continue;
    }

    if (i < msg.confidence.size())
      det.confidence = msg.confidence[i];
    if (i < msg.obj_dist.size())
      det.distance = msg.obj_dist[i];
    if (i < msg.maxPoints.size()) {
      det.bbox_max.x = msg.maxPoints[i].x;
      det.bbox_max.y = msg.maxPoints[i].y;
      det.bbox_max.z = msg.maxPoints[i].z;
    }
    if (i < msg.minPoints.size()) {
      det.bbox_min.x = msg.minPoints[i].x;
      det.bbox_min.y = msg.minPoints[i].y;
      det.bbox_min.z = msg.minPoints[i].z;
    }
    // Clamp non-finite bbox values to zero
    if (!std::isfinite(det.bbox_max.x)) det.bbox_max.x = 0.0;
    if (!std::isfinite(det.bbox_max.y)) det.bbox_max.y = 0.0;
    if (!std::isfinite(det.bbox_max.z)) det.bbox_max.z = 0.0;
    if (!std::isfinite(det.bbox_min.x)) det.bbox_min.x = 0.0;
    if (!std::isfinite(det.bbox_min.y)) det.bbox_min.y = 0.0;
    if (!std::isfinite(det.bbox_min.z)) det.bbox_min.z = 0.0;
    if (i < msg.color_types.size())
      det.color_type = msg.color_types[i];
    out.detections.push_back(det);
  }
  return out;
}

void LocationNode::ToRos(const localization_core::ConeMap &map, autodrive_msgs::HUAT_ConeMap *out)
{
  if (!out)
  {
    return;
  }
  out->cone.clear();
  out->cone.reserve(map.cones.size());
  for (const auto &cone : map.cones)
  {
    autodrive_msgs::HUAT_Cone msg;
    msg.id = cone.id;
    msg.position_global.x = cone.position_global.x;
    msg.position_global.y = cone.position_global.y;
    msg.position_global.z = cone.position_global.z;
    msg.position_baseLink.x = cone.position_base_link.x;
    msg.position_baseLink.y = cone.position_base_link.y;
    msg.position_baseLink.z = cone.position_base_link.z;
    msg.confidence = cone.confidence;
    msg.type = cone.type;
    out->cone.push_back(msg);
  }
}

void LocationNode::feedFactorGraph(const autodrive_msgs::HUAT_InsP2 &msg)
{
  const double stamp = msg.header.stamp.toSec();
  if (fg_start_time_ < 0.0) fg_start_time_ = stamp;
  const double t = stamp - fg_start_time_;

  // IMU measurement (velocity + yaw rate preintegration)
  static double last_imu_time = -1.0;
  const double v_fwd = std::sqrt(msg.Vn * msg.Vn + msg.Ve * msg.Ve);
  if (last_imu_time >= 0.0)
  {
    const double dt = stamp - last_imu_time;
    if (dt > 0.0 && dt < 1.0)
    {
      fg_optimizer_->AddImuMeasurement(v_fwd, msg.gyro_z, dt);
    }
  }
  last_imu_time = stamp;

  // GNSS observation
  localization_core::GnssQuality quality = localization_core::GnssQuality::INVALID;
  if (msg.Status >= 2)
  {
    const uint8_t nsv = std::max(msg.NSV1, msg.NSV2);
    if (nsv >= 12 && msg.Age < 5)
      quality = localization_core::GnssQuality::GOOD;
    else if (nsv >= 8 && msg.Age < 15)
      quality = localization_core::GnssQuality::MEDIUM;
    else
      quality = localization_core::GnssQuality::POOR;
  }
  if (quality != localization_core::GnssQuality::INVALID)
  {
    // Use mapper's current position as ENU proxy (mapper already does WGS84->local)
    const auto &cs = mapper_.car_state();
    fg_optimizer_->SetGnssObservation(cs.car_state.x, cs.car_state.y, quality);
  }

  // Speed observation
  fg_optimizer_->SetSpeedObservation(v_fwd);

  // Try update
  const auto &cs = mapper_.car_state();
  localization_core::Pose2 current_pose;
  current_pose.x = cs.car_state.x;
  current_pose.y = cs.car_state.y;
  current_pose.theta = cs.car_state.theta;

  if (fg_optimizer_->TryUpdate(current_pose, t))
  {
    auto fg_pose = fg_optimizer_->GetOptimizedPose();
    ROS_INFO_THROTTLE(5.0, "[FG shadow] kf=%lu pose=(%.2f,%.2f,%.3f) opt_ms=%.1f lm=%zu",
                      fg_optimizer_->NumKeyframes(),
                      fg_pose.x, fg_pose.y, fg_pose.theta,
                      fg_optimizer_->LastOptTimeMs(),
                      fg_optimizer_->GetLandmarks().size());
  }
}

void LocationNode::feedFactorGraphCones(const autodrive_msgs::HUAT_ConeDetections &msg)
{
  std::vector<localization_core::ConeObservation> obs;
  obs.reserve(msg.points.size());
  for (size_t i = 0; i < msg.points.size(); ++i)
  {
    const double px = msg.points[i].x;
    const double py = msg.points[i].y;
    const double range = std::sqrt(px * px + py * py);
    const double bearing = std::atan2(py, px);

    localization_core::ConeObservation co;
    co.range = range;
    co.bearing = bearing;
    co.color_type = (i < msg.color_types.size()) ? msg.color_types[i] : 4;
    co.confidence = (i < msg.confidence.size()) ? msg.confidence[i] : 0.0;
    obs.push_back(co);
  }
  fg_optimizer_->SetConeObservations(obs);
}

}  // namespace localization_ros
