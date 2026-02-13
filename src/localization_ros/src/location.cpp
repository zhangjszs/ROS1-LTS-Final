#include <localization_ros/location_node.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <autodrive_msgs/topic_contract.hpp>
#include <autodrive_msgs/param_utils.hpp>

namespace localization_ros {

namespace {
using autodrive_msgs::LoadParam;

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

  // Status diagnostics publisher
  LoadParam(pnh_, nh_, "topics/fg_status", status_topic_, std::string(autodrive_msgs::topic_contract::kLocalizationFgStatus));
  status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 10);
  {
    std::string diag_topic, global_diag_topic;
    bool pub_global = true;
    LoadParam(pnh_, nh_, "diagnostics_topic", diag_topic, std::string(autodrive_msgs::topic_contract::kLocalizationDiagnostics));
    LoadParam(pnh_, nh_, "publish_global_diagnostics", pub_global, true);
    LoadParam(pnh_, nh_, "global_diagnostics_topic", global_diag_topic, std::string(autodrive_msgs::topic_contract::kDiagnosticsGlobal));
    LoadParam(pnh_, nh_, "diagnostics_rate_hz", diagnostics_rate_hz_, 1.0);
    autodrive_msgs::DiagnosticsHelper::Config dcfg;
    dcfg.local_topic = diag_topic;
    dcfg.global_topic = global_diag_topic;
    dcfg.publish_global = pub_global;
    dcfg.rate_hz = diagnostics_rate_hz_;
    dcfg.queue_size = 10;
    dcfg.latch_local = true;
    diag_helper_.Init(nh_, dcfg);
  }

  // Initialize factor graph backend if configured
  if (backend_ == "factor_graph")
  {
    fg_optimizer_ = std::make_unique<localization_core::FactorGraphOptimizer>(fg_config_);
    ROS_INFO("Factor graph backend enabled (shadow mode)");
  }

  // Performance statistics
  {
    int pw = static_cast<int>(perf_window_);
    int pl = static_cast<int>(perf_log_every_);
    LoadParam(pnh_, nh_, "perf_stats_enable", perf_enabled_, true);
    LoadParam(pnh_, nh_, "perf_stats_window", pw, 300);
    LoadParam(pnh_, nh_, "perf_stats_log_every", pl, 30);
    perf_window_ = static_cast<size_t>(pw);
    perf_log_every_ = static_cast<size_t>(pl);
    perf_stats_.Configure("localization", perf_enabled_, perf_window_, perf_log_every_);
  }

  publishEntryHealth("startup", ros::Time::now(), true);
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
  if (!LoadParam(pnh_, nh_, "topics/car_state_in", carstate_in_topic_,
                 std::string(autodrive_msgs::topic_contract::kCarState)))
  {
    ROS_WARN_STREAM("Did not load topics/car_state_in. Standard value is: " << carstate_in_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/cone", cone_topic_, std::string(autodrive_msgs::topic_contract::kConeDetections)))
  {
    ROS_WARN_STREAM("Did not load topics/cone. Standard value is: " << cone_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/car_state_out", carstate_out_topic_,
                 std::string(autodrive_msgs::topic_contract::kCarState)))
  {
    ROS_WARN_STREAM("Did not load topics/car_state_out. Standard value is: " << carstate_out_topic_);
  }
  if (!LoadParam(pnh_, nh_, "topics/cone_map", cone_map_topic_, std::string(autodrive_msgs::topic_contract::kConeMap)))
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
  if (!LoadParam(pnh_, nh_, "frames/world", world_frame_,
                 std::string(autodrive_msgs::frame_contract::kWorld)))
  {
    ROS_WARN_STREAM("Did not load frames/world. Standard value is: " << world_frame_);
  }
  if (!LoadParam(pnh_, nh_, "frames/base_link", base_link_frame_,
                 std::string(autodrive_msgs::frame_contract::kBaseLink)))
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

  // INS 质量门控参数
  {
    int tmp_status = static_cast<int>(params_.min_ins_status);
    LoadParam(pnh_, nh_, "ins_quality/min_ins_status", tmp_status, tmp_status);
    params_.min_ins_status = static_cast<std::uint8_t>(tmp_status);

    int tmp_nsv = static_cast<int>(params_.min_satellite_count);
    LoadParam(pnh_, nh_, "ins_quality/min_satellite_count", tmp_nsv, tmp_nsv);
    params_.min_satellite_count = static_cast<std::uint8_t>(tmp_nsv);

    int tmp_age = static_cast<int>(params_.max_diff_age);
    LoadParam(pnh_, nh_, "ins_quality/max_diff_age", tmp_age, tmp_age);
    params_.max_diff_age = static_cast<std::uint8_t>(tmp_age);
  }

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

  // 缺锥补偿参数
  LoadParam(pnh_, nh_, "missing_cone_fallback/enabled", params_.missing_cone_fallback.enabled, false);
  LoadParam(pnh_, nh_, "missing_cone_fallback/max_interpolation_distance", params_.missing_cone_fallback.max_interpolation_distance, 8.0);
  LoadParam(pnh_, nh_, "missing_cone_fallback/expected_spacing", params_.missing_cone_fallback.expected_spacing, 5.0);
  LoadParam(pnh_, nh_, "missing_cone_fallback/min_confidence_for_interpolation", params_.missing_cone_fallback.min_confidence_for_interpolation, 0.15);
  {
    int tmp_max_missing = params_.missing_cone_fallback.max_consecutive_missing;
    LoadParam(pnh_, nh_, "missing_cone_fallback/max_consecutive_missing", tmp_max_missing, 3);
    params_.missing_cone_fallback.max_consecutive_missing = tmp_max_missing;
  }

  // 短路径抑制参数
  LoadParam(pnh_, nh_, "short_path_suppression/enabled", params_.short_path_suppression.enabled, false);
  LoadParam(pnh_, nh_, "short_path_suppression/min_path_length", params_.short_path_suppression.min_path_length, 3.0);
  {
    int tmp_min_count = params_.short_path_suppression.min_cone_count;
    LoadParam(pnh_, nh_, "short_path_suppression/min_cone_count", tmp_min_count, 3);
    params_.short_path_suppression.min_cone_count = tmp_min_count;
  }
  LoadParam(pnh_, nh_, "short_path_suppression/reject_single_cone_paths", params_.short_path_suppression.reject_single_cone_paths, true);

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

    // Color-topology soft association
    LoadParam(pnh_, nh_, "fg/w_maha", fg_config_.w_maha, 1.0);
    LoadParam(pnh_, nh_, "fg/w_color", fg_config_.w_color, 0.3);
    LoadParam(pnh_, nh_, "fg/w_topo", fg_config_.w_topo, 0.2);
    LoadParam(pnh_, nh_, "fg/topo_penalty", fg_config_.topo_penalty, 2.0);
    LoadParam(pnh_, nh_, "fg/neighbor_radius", fg_config_.neighbor_radius, 8.0);
    LoadParam(pnh_, nh_, "fg/gate_threshold", fg_config_.gate_threshold, 15.0);

    // Map mode & geometry priors
    fg_config_.map_mode = params_.map_mode;
    LoadParam(pnh_, nh_, "fg/circle_radius", fg_config_.circle_radius, params_.circle_radius);
    LoadParam(pnh_, nh_, "fg/circle_center1_x", fg_config_.circle_center1_x, 0.0);
    LoadParam(pnh_, nh_, "fg/circle_center1_y", fg_config_.circle_center1_y, -params_.circle_center_dist / 2.0);
    LoadParam(pnh_, nh_, "fg/circle_center2_x", fg_config_.circle_center2_x, 0.0);
    LoadParam(pnh_, nh_, "fg/circle_center2_y", fg_config_.circle_center2_y, params_.circle_center_dist / 2.0);
    LoadParam(pnh_, nh_, "fg/circle_sigma", fg_config_.circle_sigma, 1.0);

    // Anomaly state machine
    LoadParam(pnh_, nh_, "fg/anomaly/chi2_degrade", fg_config_.anomaly.chi2_degrade, 15.0);
    LoadParam(pnh_, nh_, "fg/anomaly/chi2_recover", fg_config_.anomaly.chi2_recover, 5.0);
    LoadParam(pnh_, nh_, "fg/anomaly/chi2_window", fg_config_.anomaly.chi2_window, 10);
    LoadParam(pnh_, nh_, "fg/anomaly/match_ratio_lost", fg_config_.anomaly.match_ratio_lost, 0.3);
    LoadParam(pnh_, nh_, "fg/anomaly/match_lost_duration", fg_config_.anomaly.match_lost_duration, 2.0);
    LoadParam(pnh_, nh_, "fg/anomaly/no_cone_frames_max", fg_config_.anomaly.no_cone_frames_max, 30);
    LoadParam(pnh_, nh_, "fg/anomaly/reloc_a_timeout", fg_config_.anomaly.reloc_a_timeout, 3.0);
    LoadParam(pnh_, nh_, "fg/anomaly/reloc_b_timeout", fg_config_.anomaly.reloc_b_timeout, 5.0);

    // Relocalization
    LoadParam(pnh_, nh_, "fg/reloc/submap_radius", fg_config_.reloc.submap_radius, 15.0);
    LoadParam(pnh_, nh_, "fg/reloc/n_sectors", fg_config_.reloc.n_sectors, 12);
    LoadParam(pnh_, nh_, "fg/reloc/n_rings", fg_config_.reloc.n_rings, 5);
    LoadParam(pnh_, nh_, "fg/reloc/max_descriptors", fg_config_.reloc.max_descriptors, 200);
    LoadParam(pnh_, nh_, "fg/reloc/top_k", fg_config_.reloc.top_k, 5);
    LoadParam(pnh_, nh_, "fg/reloc/ransac_max_iter", fg_config_.reloc.ransac_max_iter, 100);
    LoadParam(pnh_, nh_, "fg/reloc/min_inliers", fg_config_.reloc.min_inliers, 4);
    LoadParam(pnh_, nh_, "fg/reloc/n_particles", fg_config_.reloc.n_particles, 200);
    {
      double tmp;
      LoadParam(pnh_, nh_, "fg/reloc/particle_sigma_xy", tmp, fg_config_.reloc.particle_sigma_xy);
      fg_config_.reloc.particle_sigma_xy = tmp;
      LoadParam(pnh_, nh_, "fg/reloc/converge_sigma", tmp, fg_config_.reloc.converge_sigma_xy);
      fg_config_.reloc.converge_sigma_xy = tmp;
      LoadParam(pnh_, nh_, "fg/reloc/timeout", tmp, fg_config_.reloc.timeout);
      fg_config_.reloc.timeout = tmp;
    }
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

void LocationNode::publishDiagnostics(const diagnostic_msgs::DiagnosticArray &diag_arr)
{
  diag_helper_.Publish(diag_arr);
}

void LocationNode::publishEntryHealth(const std::string &source, const ros::Time &stamp, bool force)
{
  using KV = autodrive_msgs::DiagnosticsHelper;
  const uint8_t level = mapper_.has_carstate() ? diagnostic_msgs::DiagnosticStatus::OK
                                               : diagnostic_msgs::DiagnosticStatus::WARN;
  const std::string message = mapper_.has_carstate() ? "OK" : "WAITING_CARSTATE";

  std::vector<diagnostic_msgs::KeyValue> kvs;
  kvs.push_back(KV::KV("source", source));
  kvs.push_back(KV::KV("backend", backend_));
  kvs.push_back(KV::KV("has_carstate", mapper_.has_carstate() ? "true" : "false"));
  kvs.push_back(KV::KV("use_external_carstate", use_external_carstate_ ? "true" : "false"));
  kvs.push_back(KV::KV("world_frame", world_frame_));
  kvs.push_back(KV::KV("base_link_frame", base_link_frame_));

  diag_helper_.PublishStatus("localization_entry_health", "localization_ros/location_node",
                             level, message, kvs, stamp, force);
}

void LocationNode::imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr &msg)
{
  localization_core::Asensing core_msg = ToCore(*msg);

  // 缓存 INS 状态用于时间戳插值
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    ins_buffer_.push_back({stamp, core_msg});
    while (ins_buffer_.size() > kInsBufferSize)
    {
      ins_buffer_.pop_front();
    }
  }

  localization_core::CarState state;

  if (mapper_.UpdateFromIns(core_msg, &state))
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    publishState(state, stamp, true);

    // Log Heading_init once after first successful INS update
    if (!heading_init_logged_ && mapper_.has_carstate())
    {
      ROS_INFO("[localization] Heading_init (standard_azimuth) = %.4f deg", mapper_.standard_azimuth());
      heading_init_logged_ = true;
    }
  }

  // Shadow-mode factor graph feeding
  if (fg_optimizer_)
  {
    feedFactorGraph(*msg);
  }

  publishEntryHealth("imu", msg->header.stamp);
}

void LocationNode::carstateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &msg)
{
  mapper_.UpdateFromCarState(ToCore(*msg));
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  const bool publish_carstate = carstate_out_topic_ != carstate_in_topic_;
  publishState(mapper_.car_state(), stamp, publish_carstate);
  publishEntryHealth("carstate", stamp);
}

void LocationNode::coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &msg)
{
  if (!mapper_.has_carstate())
  {
    publishEntryHealth("cone_waiting_carstate", msg->header.stamp);
    ROS_WARN_THROTTLE(5.0, "[location] INS data not updated yet, skipping cone update");
    return;
  }
  if (msg->points.empty())
  {
    ROS_WARN_THROTTLE(5.0, "[location] Cone coordinates empty, skipping");
    return;
  }

  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

  // 时间戳对齐：用检测时间戳插值 INS 状态，重新更新 mapper
  localization_core::Asensing interp_ins;
  if (interpolateIns(stamp, interp_ins))
  {
    localization_core::CarState unused;
    mapper_.UpdateFromIns(interp_ins, &unused);
  }

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

  publishEntryHealth("cone", stamp);
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
    const auto anomaly_state = fg_optimizer_->GetAnomalyState();
    const char* state_names[] = {"TRACKING", "DEGRADED", "LOST", "RELOC_A", "RELOC_B"};
    const int state_idx = static_cast<int>(anomaly_state);
    const char* state_name = (state_idx >= 0 && state_idx < 5) ? state_names[state_idx] : "UNKNOWN";

    ROS_INFO_THROTTLE(5.0, "[FG shadow] kf=%lu pose=(%.2f,%.2f,%.3f) opt_ms=%.1f lm=%zu chi2=%.2f match=%.2f state=%s",
                      fg_optimizer_->NumKeyframes(),
                      fg_pose.x, fg_pose.y, fg_pose.theta,
                      fg_optimizer_->LastOptTimeMs(),
                      fg_optimizer_->GetLandmarks().size(),
                      fg_optimizer_->GetChi2Normalized(),
                      fg_optimizer_->GetConeMatchRatio(),
                      state_name);

    // Publish diagnostics status
    std_msgs::String status_msg;
    status_msg.data = std::string("state=") + state_name +
                      " chi2=" + std::to_string(fg_optimizer_->GetChi2Normalized()) +
                      " match=" + std::to_string(fg_optimizer_->GetConeMatchRatio()) +
                      " kf=" + std::to_string(fg_optimizer_->NumKeyframes()) +
                      " lm=" + std::to_string(fg_optimizer_->GetLandmarks().size());
    status_pub_.publish(status_msg);

    // Publish structured diagnostics
    {
      diagnostic_msgs::DiagnosticArray diag_arr;
      diag_arr.header.stamp = ros::Time::now();

      diagnostic_msgs::DiagnosticStatus ds;
      ds.name = "localization/factor_graph";
      ds.hardware_id = "localization";

      if (anomaly_state == localization_core::AnomalyState::TRACKING)
        ds.level = diagnostic_msgs::DiagnosticStatus::OK;
      else if (anomaly_state == localization_core::AnomalyState::DEGRADED)
        ds.level = diagnostic_msgs::DiagnosticStatus::WARN;
      else
        ds.level = diagnostic_msgs::DiagnosticStatus::ERROR;

      ds.message = state_name;

      auto kv = [](const std::string &k, const std::string &v) {
        diagnostic_msgs::KeyValue pair;
        pair.key = k;
        pair.value = v;
        return pair;
      };
      ds.values.push_back(kv("anomaly_state", state_name));
      ds.values.push_back(kv("chi2_normalized", std::to_string(fg_optimizer_->GetChi2Normalized())));
      ds.values.push_back(kv("cone_match_ratio", std::to_string(fg_optimizer_->GetConeMatchRatio())));
      ds.values.push_back(kv("num_keyframes", std::to_string(fg_optimizer_->NumKeyframes())));
      ds.values.push_back(kv("num_landmarks", std::to_string(fg_optimizer_->GetLandmarks().size())));
      ds.values.push_back(kv("opt_time_ms", std::to_string(fg_optimizer_->LastOptTimeMs())));

      diag_arr.status.push_back(ds);
      publishDiagnostics(diag_arr);
    }

    // Collect performance sample
    {
      LocPerfSample sample;
      sample.t_opt_ms = fg_optimizer_->LastOptTimeMs();
      sample.t_total_ms = fg_optimizer_->LastOptTimeMs();
      sample.landmark_count = static_cast<double>(fg_optimizer_->GetLandmarks().size());
      sample.chi2_normalized = fg_optimizer_->GetChi2Normalized();
      sample.cone_match_ratio = fg_optimizer_->GetConeMatchRatio();
      sample.gnss_availability = (quality != localization_core::GnssQuality::INVALID) ? 1.0 : 0.0;
      perf_stats_.Add(sample);
    }
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

bool LocationNode::interpolateIns(const ros::Time &target, localization_core::Asensing &out) const
{
  if (ins_buffer_.size() < 2)
  {
    return false;
  }

  // 查找 target 所在的区间 [i-1, i]
  // 缓冲区按时间递增排列
  const double t_target = target.toSec();
  const double t_first = ins_buffer_.front().stamp.toSec();
  const double t_last = ins_buffer_.back().stamp.toSec();

  // 如果 target 在缓冲区范围外，使用最近的样本
  if (t_target <= t_first)
  {
    out = ins_buffer_.front().data;
    return true;
  }
  if (t_target >= t_last)
  {
    out = ins_buffer_.back().data;
    return true;
  }

  // 二分查找第一个 stamp >= target 的元素
  size_t lo = 0;
  size_t hi = ins_buffer_.size();
  while (lo < hi)
  {
    const size_t mid = lo + (hi - lo) / 2;
    if (ins_buffer_[mid].stamp.toSec() < t_target)
      lo = mid + 1;
    else
      hi = mid;
  }

  if (lo == 0)
  {
    out = ins_buffer_.front().data;
    return true;
  }

  const auto &a = ins_buffer_[lo - 1];
  const auto &b = ins_buffer_[lo];
  const double dt = b.stamp.toSec() - a.stamp.toSec();
  if (dt < 1e-9)
  {
    out = a.data;
    return true;
  }

  const double alpha = (t_target - a.stamp.toSec()) / dt;

  // 线性插值标量字段
  auto lerp = [alpha](double va, double vb) { return va + alpha * (vb - va); };

  // 角度插值（处理 0/360 度环绕）
  auto angle_lerp = [alpha](double va, double vb) {
    double diff = vb - va;
    // Normalize to [-180, 180]
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    double result = va + alpha * diff;
    // Normalize result to [0, 360)
    while (result < 0.0) result += 360.0;
    while (result >= 360.0) result -= 360.0;
    return result;
  };

  out.latitude = lerp(a.data.latitude, b.data.latitude);
  out.longitude = lerp(a.data.longitude, b.data.longitude);
  out.altitude = lerp(a.data.altitude, b.data.altitude);
  out.north_velocity = lerp(a.data.north_velocity, b.data.north_velocity);
  out.east_velocity = lerp(a.data.east_velocity, b.data.east_velocity);
  out.ground_velocity = lerp(a.data.ground_velocity, b.data.ground_velocity);
  out.roll = lerp(a.data.roll, b.data.roll);
  out.pitch = lerp(a.data.pitch, b.data.pitch);
  out.azimuth = angle_lerp(a.data.azimuth, b.data.azimuth);
  out.x_angular_velocity = lerp(a.data.x_angular_velocity, b.data.x_angular_velocity);
  out.y_angular_velocity = lerp(a.data.y_angular_velocity, b.data.y_angular_velocity);
  out.z_angular_velocity = lerp(a.data.z_angular_velocity, b.data.z_angular_velocity);
  out.x_acc = lerp(a.data.x_acc, b.data.x_acc);
  out.y_acc = lerp(a.data.y_acc, b.data.y_acc);
  out.z_acc = lerp(a.data.z_acc, b.data.z_acc);

  // 离散字段取较近的样本
  out.status = (alpha < 0.5) ? a.data.status : b.data.status;
  out.nsv1 = (alpha < 0.5) ? a.data.nsv1 : b.data.nsv1;
  out.nsv2 = (alpha < 0.5) ? a.data.nsv2 : b.data.nsv2;
  out.age = (alpha < 0.5) ? a.data.age : b.data.age;

  return true;
}

}  // namespace localization_ros
