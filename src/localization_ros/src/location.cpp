#include <localization_ros/location_node.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <autodrive_msgs/topic_contract.hpp>
#include <autodrive_msgs/param_utils.hpp>
#include <fsd_common/geometry_utils.hpp>

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

std::string ToFixed(double value, int precision = 3)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

double NormalizeDeg360(double deg)
{
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

double ShortestDiffDeg(double current_deg, double prev_deg)
{
  double diff = NormalizeDeg360(current_deg) - NormalizeDeg360(prev_deg);
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
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
  if (debug_topics_cfg_.enabled)
  {
    debug_inlier_map_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeMap>(debug_topics_cfg_.inlier_map_topic, 10);
    debug_outlier_map_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeMap>(debug_topics_cfg_.outlier_map_topic, 10);
    debug_local_map_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeMap>(debug_topics_cfg_.local_map_topic, 10);
    debug_match_pairs_pub_ = nh_.advertise<std_msgs::String>(debug_topics_cfg_.match_pairs_topic, 10);
    debug_relocalization_pub_ = nh_.advertise<std_msgs::String>(debug_topics_cfg_.relocalization_topic, 10);
  }

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
  if (map_persistence_cfg_.enabled)
  {
    save_map_srv_ = nh_.advertiseService(map_persistence_cfg_.save_service, &LocationNode::handleSaveMap, this);
    load_map_srv_ = nh_.advertiseService(map_persistence_cfg_.load_service, &LocationNode::handleLoadMap, this);
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

  // 地图冻结参数（第一圈建图 -> 冻结）
  LoadParam(pnh_, nh_, "map/freeze/enabled", params_.map_freeze.enabled, false);
  LoadParam(pnh_, nh_, "map/freeze/freeze_after_frames", params_.map_freeze.freeze_after_frames, 300);
  LoadParam(pnh_, nh_, "map/freeze/freeze_after_cones", params_.map_freeze.freeze_after_cones, 120);
  LoadParam(pnh_, nh_, "map/freeze/allow_merge_when_frozen", params_.map_freeze.allow_merge_when_frozen, true);
  LoadParam(pnh_, nh_, "map/freeze/allow_new_cones_when_frozen", params_.map_freeze.allow_new_cones_when_frozen, false);
  params_.map_freeze.freeze_after_frames = std::max(0, params_.map_freeze.freeze_after_frames);
  params_.map_freeze.freeze_after_cones = std::max(0, params_.map_freeze.freeze_after_cones);

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

  // Mapper runtime protection & recovery state machine (only for backend=mapper)
  LoadParam(pnh_, nh_, "mapper_recovery/enabled", mapper_recovery_cfg_.enabled, true);
  LoadParam(pnh_, nh_, "mapper_recovery/fail_frames_to_degraded",
            mapper_recovery_cfg_.fail_frames_to_degraded, 3);
  LoadParam(pnh_, nh_, "mapper_recovery/fail_frames_to_ins_only",
            mapper_recovery_cfg_.fail_frames_to_ins_only, 8);
  LoadParam(pnh_, nh_, "mapper_recovery/success_frames_to_tracking",
            mapper_recovery_cfg_.success_frames_to_tracking, 3);
  LoadParam(pnh_, nh_, "mapper_recovery/recovery_cooldown_frames",
            mapper_recovery_cfg_.recovery_cooldown_frames, 20);
  LoadParam(pnh_, nh_, "mapper_recovery/recovery_probe_interval_frames",
            mapper_recovery_cfg_.recovery_probe_interval_frames, 3);
  LoadParam(pnh_, nh_, "mapper_recovery/min_local_cones_for_tracking",
            mapper_recovery_cfg_.min_local_cones_for_tracking, 3);
  LoadParam(pnh_, nh_, "mapper_recovery/min_match_ratio_for_tracking",
            mapper_recovery_cfg_.min_match_ratio_for_tracking, 0.25);
  LoadParam(pnh_, nh_, "mapper_recovery/publish_stale_map_on_failure",
            mapper_recovery_cfg_.publish_stale_map_on_failure, true);

  mapper_recovery_cfg_.fail_frames_to_degraded = std::max(1, mapper_recovery_cfg_.fail_frames_to_degraded);
  mapper_recovery_cfg_.fail_frames_to_ins_only =
      std::max(mapper_recovery_cfg_.fail_frames_to_degraded + 1,
               mapper_recovery_cfg_.fail_frames_to_ins_only);
  mapper_recovery_cfg_.success_frames_to_tracking = std::max(1, mapper_recovery_cfg_.success_frames_to_tracking);
  mapper_recovery_cfg_.recovery_cooldown_frames = std::max(0, mapper_recovery_cfg_.recovery_cooldown_frames);
  mapper_recovery_cfg_.recovery_probe_interval_frames = std::max(1, mapper_recovery_cfg_.recovery_probe_interval_frames);
  mapper_recovery_cfg_.min_local_cones_for_tracking = std::max(1, mapper_recovery_cfg_.min_local_cones_for_tracking);
  mapper_recovery_cfg_.min_match_ratio_for_tracking =
      std::max(0.0, std::min(1.0, mapper_recovery_cfg_.min_match_ratio_for_tracking));

  LoadParam(pnh_, nh_, "tf_monitor/max_delay_sec", tf_monitor_cfg_.max_delay_sec, 0.10);
  LoadParam(pnh_, nh_, "tf_monitor/max_future_sec", tf_monitor_cfg_.max_future_sec, 0.02);
  LoadParam(pnh_, nh_, "tf_monitor/max_gap_sec", tf_monitor_cfg_.max_gap_sec, 0.20);
  tf_monitor_cfg_.max_delay_sec = std::max(0.0, tf_monitor_cfg_.max_delay_sec);
  tf_monitor_cfg_.max_future_sec = std::max(0.0, tf_monitor_cfg_.max_future_sec);
  tf_monitor_cfg_.max_gap_sec = std::max(0.0, tf_monitor_cfg_.max_gap_sec);

  LoadParam(pnh_, nh_, "heading_sanity/enabled", heading_sanity_cfg_.enabled, true);
  LoadParam(pnh_, nh_, "heading_sanity/abs_max_deg", heading_sanity_cfg_.abs_max_deg, 360.0);
  LoadParam(pnh_, nh_, "heading_sanity/max_step_deg", heading_sanity_cfg_.max_step_deg, 45.0);
  LoadParam(pnh_, nh_, "heading_sanity/max_rate_deg_per_sec",
            heading_sanity_cfg_.max_rate_deg_per_sec, 180.0);
  LoadParam(pnh_, nh_, "heading_sanity/max_heading_gyro_residual_rad_s",
            heading_sanity_cfg_.max_heading_gyro_residual_rad_s, 0.35);
  heading_sanity_cfg_.abs_max_deg = std::max(0.0, heading_sanity_cfg_.abs_max_deg);
  heading_sanity_cfg_.max_step_deg = std::max(0.0, heading_sanity_cfg_.max_step_deg);
  heading_sanity_cfg_.max_rate_deg_per_sec = std::max(0.0, heading_sanity_cfg_.max_rate_deg_per_sec);
  heading_sanity_cfg_.max_heading_gyro_residual_rad_s =
      std::max(0.0, heading_sanity_cfg_.max_heading_gyro_residual_rad_s);

  // Debug topics (匹配对 / inlier / outlier / local map / 重定位评分)
  LoadParam(pnh_, nh_, "debug_topics/enabled", debug_topics_cfg_.enabled, true);
  LoadParam(pnh_, nh_, "debug_topics/inlier_map", debug_topics_cfg_.inlier_map_topic,
            std::string("localization/debug/inlier_map"));
  LoadParam(pnh_, nh_, "debug_topics/outlier_map", debug_topics_cfg_.outlier_map_topic,
            std::string("localization/debug/outlier_map"));
  LoadParam(pnh_, nh_, "debug_topics/local_map", debug_topics_cfg_.local_map_topic,
            std::string("localization/debug/local_map"));
  LoadParam(pnh_, nh_, "debug_topics/match_pairs", debug_topics_cfg_.match_pairs_topic,
            std::string("localization/debug/match_pairs"));
  LoadParam(pnh_, nh_, "debug_topics/relocalization", debug_topics_cfg_.relocalization_topic,
            std::string("localization/debug/relocalization"));
  LoadParam(pnh_, nh_, "debug_topics/max_pairs", debug_topics_cfg_.max_pairs, 20);
  debug_topics_cfg_.max_pairs = std::max(1, debug_topics_cfg_.max_pairs);

  // 地图漂移监控与恢复
  LoadParam(pnh_, nh_, "map_drift/enabled", drift_monitor_cfg_.enabled, true);
  LoadParam(pnh_, nh_, "map_drift/min_match_ratio", drift_monitor_cfg_.min_match_ratio, 0.15);
  LoadParam(pnh_, nh_, "map_drift/max_mean_match_distance", drift_monitor_cfg_.max_mean_match_distance, 2.0);
  LoadParam(pnh_, nh_, "map_drift/min_local_cones", drift_monitor_cfg_.min_local_cones, 2);
  LoadParam(pnh_, nh_, "map_drift/bad_frames_to_trigger", drift_monitor_cfg_.bad_frames_to_trigger, 15);
  LoadParam(pnh_, nh_, "map_drift/cooldown_frames", drift_monitor_cfg_.cooldown_frames, 50);
  LoadParam(pnh_, nh_, "map_drift/trigger_only_when_frozen", drift_monitor_cfg_.trigger_only_when_frozen, true);
  LoadParam(pnh_, nh_, "map_drift/action", drift_monitor_cfg_.action, std::string("rollback"));
  drift_monitor_cfg_.min_match_ratio = std::max(0.0, std::min(1.0, drift_monitor_cfg_.min_match_ratio));
  drift_monitor_cfg_.max_mean_match_distance = std::max(0.0, drift_monitor_cfg_.max_mean_match_distance);
  drift_monitor_cfg_.min_local_cones = std::max(1, drift_monitor_cfg_.min_local_cones);
  drift_monitor_cfg_.bad_frames_to_trigger = std::max(1, drift_monitor_cfg_.bad_frames_to_trigger);
  drift_monitor_cfg_.cooldown_frames = std::max(0, drift_monitor_cfg_.cooldown_frames);
  if (drift_monitor_cfg_.action != "rollback" &&
      drift_monitor_cfg_.action != "reset" &&
      drift_monitor_cfg_.action != "freeze")
  {
    drift_monitor_cfg_.action = "rollback";
  }

  // 地图保存/加载接口
  LoadParam(pnh_, nh_, "map_persistence/enabled", map_persistence_cfg_.enabled, true);
  LoadParam(pnh_, nh_, "map_persistence/save_path", map_persistence_cfg_.save_path,
            std::string("/tmp/localization_map.csv"));
  LoadParam(pnh_, nh_, "map_persistence/version_tag", map_persistence_cfg_.version_tag,
            std::string("v1"));
  LoadParam(pnh_, nh_, "map_persistence/save_service", map_persistence_cfg_.save_service,
            std::string("localization/map/save"));
  LoadParam(pnh_, nh_, "map_persistence/load_service", map_persistence_cfg_.load_service,
            std::string("localization/map/load"));

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

const char *LocationNode::mapperStateName(int state)
{
  switch (static_cast<MapperRuntimeState>(state))
  {
    case MapperRuntimeState::TRACKING:
      return "TRACKING";
    case MapperRuntimeState::DEGRADED:
      return "DEGRADED";
    case MapperRuntimeState::INS_ONLY:
      return "INS_ONLY";
    default:
      return "UNKNOWN";
  }
}

bool LocationNode::shouldProcessConeFusion()
{
  if (backend_ != "mapper" || !mapper_recovery_cfg_.enabled)
  {
    return true;
  }
  if (mapper_state_ != MapperRuntimeState::INS_ONLY)
  {
    return true;
  }

  ++mapper_ins_only_frames_;
  if (mapper_ins_only_frames_ <= mapper_recovery_cfg_.recovery_cooldown_frames)
  {
    return false;
  }

  const int probe_idx = mapper_ins_only_frames_ - mapper_recovery_cfg_.recovery_cooldown_frames;
  return (probe_idx % mapper_recovery_cfg_.recovery_probe_interval_frames) == 0;
}

void LocationNode::updateMapperStateMachine(bool frame_good)
{
  if (backend_ != "mapper" || !mapper_recovery_cfg_.enabled)
  {
    return;
  }

  if (frame_good)
  {
    mapper_consecutive_failures_ = 0;
    ++mapper_consecutive_successes_;
  }
  else
  {
    mapper_consecutive_successes_ = 0;
    ++mapper_consecutive_failures_;
  }

  const MapperRuntimeState prev_state = mapper_state_;
  switch (mapper_state_)
  {
    case MapperRuntimeState::TRACKING:
      if (!frame_good &&
          mapper_consecutive_failures_ >= mapper_recovery_cfg_.fail_frames_to_degraded)
      {
        mapper_state_ = MapperRuntimeState::DEGRADED;
      }
      break;

    case MapperRuntimeState::DEGRADED:
      if (!frame_good &&
          mapper_consecutive_failures_ >= mapper_recovery_cfg_.fail_frames_to_ins_only)
      {
        mapper_state_ = MapperRuntimeState::INS_ONLY;
        mapper_ins_only_frames_ = 0;
      }
      else if (frame_good &&
               mapper_consecutive_successes_ >= mapper_recovery_cfg_.success_frames_to_tracking)
      {
        mapper_state_ = MapperRuntimeState::TRACKING;
      }
      break;

    case MapperRuntimeState::INS_ONLY:
      if (frame_good)
      {
        mapper_state_ = (mapper_consecutive_successes_ >= mapper_recovery_cfg_.success_frames_to_tracking)
                            ? MapperRuntimeState::TRACKING
                            : MapperRuntimeState::DEGRADED;
      }
      break;
  }

  if (mapper_state_ != MapperRuntimeState::INS_ONLY)
  {
    mapper_ins_only_frames_ = 0;
  }

  if (prev_state != mapper_state_)
  {
    ROS_WARN_STREAM("[location] mapper_state transition "
                    << mapperStateName(static_cast<int>(prev_state))
                    << " -> " << mapperStateName(static_cast<int>(mapper_state_))
                    << " (frame_good=" << (frame_good ? "true" : "false")
                    << ", fail=" << mapper_consecutive_failures_
                    << ", succ=" << mapper_consecutive_successes_ << ")");
  }
}

int LocationNode::carStateQualityLevel() const
{
  if (!mapper_.has_carstate())
  {
    return 0;
  }

  int level = 2;
  if (backend_ == "mapper" && mapper_recovery_cfg_.enabled)
  {
    if (mapper_state_ == MapperRuntimeState::INS_ONLY)
    {
      level = 0;
    }
    else if (mapper_state_ == MapperRuntimeState::DEGRADED)
    {
      level = 1;
    }
  }

  if (tf_monitor_cfg_.max_delay_sec > 0.0 && tf_last_lag_sec_ > tf_monitor_cfg_.max_delay_sec)
  {
    level = std::min(level, 1);
  }
  if (tf_monitor_cfg_.max_future_sec > 0.0 && tf_last_lag_sec_ < -tf_monitor_cfg_.max_future_sec)
  {
    level = std::min(level, 1);
  }

  return level;
}

std::string LocationNode::carStateQualityLabel() const
{
  const int level = carStateQualityLevel();
  if (level >= 2) return "GOOD";
  if (level == 1) return "DEGRADED";
  return "POOR";
}

void LocationNode::updateHeadingSanity(const autodrive_msgs::HUAT_InsP2 &msg, const ros::Time &stamp)
{
  if (!heading_sanity_cfg_.enabled)
  {
    return;
  }
  if (!std::isfinite(msg.Heading))
  {
    ++heading_range_violation_count_;
    ROS_WARN_THROTTLE(2.0, "[location] heading sanity: non-finite heading detected");
    return;
  }

  const double heading_raw_deg = static_cast<double>(msg.Heading);
  if (std::abs(heading_raw_deg) > heading_sanity_cfg_.abs_max_deg)
  {
    ++heading_range_violation_count_;
    ROS_WARN_THROTTLE(2.0, "[location] heading sanity: heading out of range %.3f deg (limit %.3f)",
                      heading_raw_deg, heading_sanity_cfg_.abs_max_deg);
  }

  const double heading_deg = NormalizeDeg360(heading_raw_deg);
  if (has_last_heading_ && stamp > last_heading_stamp_)
  {
    const double dt = (stamp - last_heading_stamp_).toSec();
    if (dt > 1e-3)
    {
      const double signed_step_deg = ShortestDiffDeg(heading_deg, last_heading_deg_);
      const double step_deg = std::abs(signed_step_deg);
      if (step_deg > heading_sanity_cfg_.max_step_deg)
      {
        ++heading_jump_violation_count_;
        ROS_WARN_THROTTLE(2.0,
                          "[location] heading sanity: step too large %.3f deg (limit %.3f)",
                          step_deg, heading_sanity_cfg_.max_step_deg);
      }

      const double rate_deg_s = step_deg / dt;
      if (rate_deg_s > heading_sanity_cfg_.max_rate_deg_per_sec)
      {
        ++heading_rate_violation_count_;
        ROS_WARN_THROTTLE(2.0,
                          "[location] heading sanity: rate too large %.3f deg/s (limit %.3f)",
                          rate_deg_s, heading_sanity_cfg_.max_rate_deg_per_sec);
      }

      constexpr double kPi = 3.14159265358979323846;
      const double heading_rate_rad_s = (signed_step_deg * kPi / 180.0) / dt;
      last_heading_rate_rad_s_ = heading_rate_rad_s;
      const double gyro_residual = std::abs(heading_rate_rad_s - static_cast<double>(msg.gyro_z));
      if (gyro_residual > heading_sanity_cfg_.max_heading_gyro_residual_rad_s)
      {
        ++heading_gyro_mismatch_count_;
        ROS_WARN_THROTTLE(
            2.0,
            "[location] heading sanity: heading/gyro mismatch %.3f rad/s (limit %.3f)",
            gyro_residual, heading_sanity_cfg_.max_heading_gyro_residual_rad_s);
      }
    }
  }

  last_heading_deg_ = heading_deg;
  last_heading_stamp_ = stamp;
  has_last_heading_ = true;
}

void LocationNode::updateConeMapLifecycleStats(const autodrive_msgs::HUAT_ConeMap &map_msg)
{
  cone_last_map_size_ = static_cast<int>(map_msg.cone.size());
  for (const auto &cone : map_msg.cone)
  {
    const std::uint32_t id = cone.id;
    if (id == 0)
    {
      ++cone_zero_id_count_;
      continue;
    }
    const bool is_new = seen_cone_ids_.insert(id).second;
    if (!is_new)
    {
      continue;
    }
    ++cone_new_id_count_;
    if (id <= cone_max_id_seen_)
    {
      ++cone_non_monotonic_new_id_count_;
    }
    cone_max_id_seen_ = std::max(cone_max_id_seen_, id);
  }
}

void LocationNode::publishMapperDebugTopics(const localization_core::MapUpdateStats &stats, const ros::Time &stamp)
{
  if (!debug_topics_cfg_.enabled)
  {
    return;
  }

  localization_core::ConeMap inlier_map;
  inlier_map.cones = stats.inlier_cones;
  autodrive_msgs::HUAT_ConeMap inlier_msg;
  ToRos(inlier_map, &inlier_msg);
  inlier_msg.header.stamp = stamp;
  inlier_msg.header.frame_id = world_frame_;
  debug_inlier_map_pub_.publish(inlier_msg);

  localization_core::ConeMap outlier_map;
  outlier_map.cones = stats.outlier_cones;
  autodrive_msgs::HUAT_ConeMap outlier_msg;
  ToRos(outlier_map, &outlier_msg);
  outlier_msg.header.stamp = stamp;
  outlier_msg.header.frame_id = world_frame_;
  debug_outlier_map_pub_.publish(outlier_msg);

  if (has_last_map_msg_)
  {
    autodrive_msgs::HUAT_ConeMap local_msg = last_map_msg_;
    local_msg.header.stamp = stamp;
    local_msg.header.frame_id = world_frame_;
    debug_local_map_pub_.publish(local_msg);
  }

  std_msgs::String pair_msg;
  {
    std::ostringstream oss;
    oss << "seq=" << stats.update_seq
        << ",merged=" << stats.merged_count
        << ",inserted=" << stats.inserted_count
        << ",outlier=" << (stats.bbox_reject_count + stats.geometry_reject_count +
                           stats.conf_add_reject_count + stats.conf_merge_reject_count +
                           stats.frozen_reject_count)
        << ",map_frozen=" << (stats.map_frozen ? "true" : "false")
        << ",mean_match_distance=" << ToFixed(stats.mean_match_distance, 3);
    const int max_pairs = std::min(debug_topics_cfg_.max_pairs, static_cast<int>(stats.associations.size()));
    for (int i = 0; i < max_pairs; ++i)
    {
      const auto &a = stats.associations[i];
      oss << ";pair" << i
          << "{id=" << a.matched_id
          << ",d=" << ToFixed(a.distance, 3)
          << ",merged=" << (a.merged ? "1" : "0")
          << ",obs=(" << ToFixed(a.detection_base.x, 2) << "," << ToFixed(a.detection_base.y, 2) << ")"
          << ",map=(" << ToFixed(a.matched_global.x, 2) << "," << ToFixed(a.matched_global.y, 2) << ")}";
    }
    pair_msg.data = oss.str();
  }
  debug_match_pairs_pub_.publish(pair_msg);

  std_msgs::String reloc_msg;
  {
    std::ostringstream oss;
    oss << "backend=" << backend_
        << ",state=" << mapperStateName(static_cast<int>(mapper_state_))
        << ",candidate_score=" << ToFixed(last_cone_match_ratio_, 3)
        << ",mode=proxy_match_ratio";
    reloc_msg.data = oss.str();
  }
  debug_relocalization_pub_.publish(reloc_msg);
}

bool LocationNode::evaluateDriftAndRecover(const localization_core::MapUpdateStats &stats, const ros::Time &stamp)
{
  if (!drift_monitor_cfg_.enabled)
  {
    return false;
  }
  if (drift_cooldown_frames_ > 0)
  {
    --drift_cooldown_frames_;
    drift_bad_frames_ = 0;
    drift_last_frame_bad_ = false;
    return false;
  }

  const bool low_match = last_cone_match_ratio_ < drift_monitor_cfg_.min_match_ratio;
  const bool high_residual =
      (stats.merged_count > 0 && stats.mean_match_distance > drift_monitor_cfg_.max_mean_match_distance);
  const bool low_local = stats.local_output_count < drift_monitor_cfg_.min_local_cones;

  bool frame_bad = low_match || high_residual || low_local;
  if (drift_monitor_cfg_.trigger_only_when_frozen && !stats.map_frozen)
  {
    frame_bad = false;
  }
  drift_last_frame_bad_ = frame_bad;
  if (!frame_bad)
  {
    drift_bad_frames_ = 0;
    return false;
  }

  ++drift_bad_frames_;
  if (drift_bad_frames_ < drift_monitor_cfg_.bad_frames_to_trigger)
  {
    return false;
  }

  ++drift_event_count_;
  bool recovered = false;
  if (drift_monitor_cfg_.action == "rollback")
  {
    recovered = mapper_.RollbackToCheckpoint();
    if (recovered)
    {
      ++drift_rollback_count_;
    }
  }
  if (!recovered && drift_monitor_cfg_.action == "reset")
  {
    mapper_.ResetMap(false);
    recovered = true;
    ++drift_reset_count_;
  }
  if (!recovered && drift_monitor_cfg_.action == "freeze")
  {
    mapper_state_ = MapperRuntimeState::INS_ONLY;
    mapper_ins_only_frames_ = 0;
    recovered = true;
    ++drift_freeze_count_;
  }
  if (!recovered)
  {
    mapper_.ResetMap(false);
    recovered = true;
    ++drift_reset_count_;
  }

  if (recovered)
  {
    drift_bad_frames_ = 0;
    drift_cooldown_frames_ = drift_monitor_cfg_.cooldown_frames;
    has_last_map_msg_ = false;
    has_last_map_stats_ = false;
    seen_cone_ids_.clear();
    cone_max_id_seen_ = 0;
    cone_last_map_size_ = 0;
    ROS_WARN_STREAM("[location] map drift recovery triggered, action=" << drift_monitor_cfg_.action
                    << ", match_ratio=" << ToFixed(last_cone_match_ratio_, 3)
                    << ", mean_match_distance=" << ToFixed(stats.mean_match_distance, 3));
    publishEntryHealth("drift_recovery", stamp, true);
  }
  return recovered;
}

bool LocationNode::handleSaveMap(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
  std::string error_msg;
  const bool ok = mapper_.SaveMapToFile(map_persistence_cfg_.save_path, &error_msg);
  if (ok)
  {
    ++map_save_count_;
    map_last_save_time_sec_ = ros::Time::now().toSec();
    res.success = true;
    res.message = "saved map to " + map_persistence_cfg_.save_path;
  }
  else
  {
    ++map_save_fail_count_;
    res.success = false;
    res.message = "save map failed: " + error_msg;
  }
  return true;
}

bool LocationNode::handleLoadMap(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
{
  std::string error_msg;
  const bool ok = mapper_.LoadMapFromFile(map_persistence_cfg_.save_path, &error_msg);
  if (ok)
  {
    ++map_load_count_;
    map_last_load_time_sec_ = ros::Time::now().toSec();
    has_last_map_msg_ = false;
    has_last_map_stats_ = false;
    seen_cone_ids_.clear();
    cone_max_id_seen_ = 0;
    cone_last_map_size_ = 0;
    res.success = true;
    res.message = "loaded map from " + map_persistence_cfg_.save_path;
  }
  else
  {
    ++map_load_fail_count_;
    res.success = false;
    res.message = "load map failed: " + error_msg;
  }
  return true;
}

void LocationNode::publishState(const localization_core::CarState &state, const ros::Time &stamp, bool publish_carstate)
{
  const geometry_msgs::Quaternion orientation = YawToQuaternion(state.car_state.theta);
  double v_forward = 0.0;
  double yaw_rate = 0.0;
  const ros::Time now = ros::Time::now();
  tf_last_lag_sec_ = (now - stamp).toSec();
  tf_max_lag_sec_ = std::max(tf_max_lag_sec_, tf_last_lag_sec_);

  if (tf_monitor_cfg_.max_delay_sec > 0.0 && tf_last_lag_sec_ > tf_monitor_cfg_.max_delay_sec)
  {
    ++tf_delay_exceed_count_;
    ROS_WARN_THROTTLE(2.0,
                      "[location] TF lag exceeds threshold: lag=%.4fs limit=%.4fs",
                      tf_last_lag_sec_, tf_monitor_cfg_.max_delay_sec);
  }
  if (tf_monitor_cfg_.max_future_sec > 0.0 && tf_last_lag_sec_ < -tf_monitor_cfg_.max_future_sec)
  {
    ++tf_future_stamp_count_;
    ROS_WARN_THROTTLE(2.0,
                      "[location] TF stamp is in future: lag=%.4fs tolerance=%.4fs",
                      tf_last_lag_sec_, tf_monitor_cfg_.max_future_sec);
  }
  if (has_last_state_ && stamp < last_stamp_)
  {
    ++tf_stamp_regression_count_;
    ROS_WARN_THROTTLE(2.0,
                      "[location] Non-monotonic TF stamp detected (current %.6f < previous %.6f)",
                      stamp.toSec(), last_stamp_.toSec());
  }
  if (has_last_state_ && tf_monitor_cfg_.max_gap_sec > 0.0)
  {
    const double dt = (stamp - last_stamp_).toSec();
    if (dt > tf_monitor_cfg_.max_gap_sec)
    {
      ++tf_gap_exceed_count_;
      ROS_WARN_THROTTLE(2.0,
                        "[location] TF publish gap exceeds threshold: dt=%.4fs limit=%.4fs",
                        dt, tf_monitor_cfg_.max_gap_sec);
    }
  }

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
      const double dtheta = fsd_common::AngleDiff(state.car_state.theta, last_state_.car_state.theta);
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
  const int quality_level = carStateQualityLevel();
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  if (!mapper_.has_carstate())
  {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
  }
  else if (quality_level <= 0)
  {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }
  else if (quality_level == 1)
  {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
  }
  const std::string quality_label = carStateQualityLabel();
  const std::string message = mapper_.has_carstate() ? quality_label : "WAITING_CARSTATE";

  std::vector<diagnostic_msgs::KeyValue> kvs;
  kvs.push_back(KV::KV("source", source));
  kvs.push_back(KV::KV("backend", backend_));
  kvs.push_back(KV::KV("has_carstate", mapper_.has_carstate() ? "true" : "false"));
  kvs.push_back(KV::KV("use_external_carstate", use_external_carstate_ ? "true" : "false"));
  kvs.push_back(KV::KV("world_frame", world_frame_));
  kvs.push_back(KV::KV("base_link_frame", base_link_frame_));
  kvs.push_back(KV::KV("car_state_quality_level", std::to_string(quality_level)));
  kvs.push_back(KV::KV("car_state_quality_label", quality_label));
  kvs.push_back(KV::KV("car_state_covariance_available", "false"));
  kvs.push_back(KV::KV("car_state_quality_contract", "diagnostics_only"));
  kvs.push_back(KV::KV("mapper_state", mapperStateName(static_cast<int>(mapper_state_))));
  kvs.push_back(KV::KV("mapper_consecutive_failures", std::to_string(mapper_consecutive_failures_)));
  kvs.push_back(KV::KV("mapper_consecutive_successes", std::to_string(mapper_consecutive_successes_)));
  kvs.push_back(KV::KV("mapper_ins_only_frames", std::to_string(mapper_ins_only_frames_)));
  kvs.push_back(KV::KV("cone_last_update_success", last_cone_update_success_ ? "true" : "false"));
  kvs.push_back(KV::KV("cone_last_frame_good", last_cone_frame_good_ ? "true" : "false"));
  kvs.push_back(KV::KV("cone_last_input_count", std::to_string(last_input_cone_count_)));
  kvs.push_back(KV::KV("cone_last_local_count", std::to_string(last_local_cone_count_)));
  kvs.push_back(KV::KV("cone_last_match_ratio", ToFixed(last_cone_match_ratio_, 3)));
  kvs.push_back(KV::KV("cone_drop_count", std::to_string(mapper_drop_count_)));
  kvs.push_back(KV::KV("tf_lag_sec", ToFixed(tf_last_lag_sec_, 4)));
  kvs.push_back(KV::KV("tf_max_lag_sec", ToFixed(tf_max_lag_sec_, 4)));
  kvs.push_back(KV::KV("tf_delay_exceed_count", std::to_string(tf_delay_exceed_count_)));
  kvs.push_back(KV::KV("tf_future_stamp_count", std::to_string(tf_future_stamp_count_)));
  kvs.push_back(KV::KV("tf_stamp_regression_count", std::to_string(tf_stamp_regression_count_)));
  kvs.push_back(KV::KV("tf_gap_exceed_count", std::to_string(tf_gap_exceed_count_)));
  kvs.push_back(KV::KV("heading_sanity_enabled", heading_sanity_cfg_.enabled ? "true" : "false"));
  kvs.push_back(KV::KV("heading_last_deg", ToFixed(last_heading_deg_, 3)));
  kvs.push_back(KV::KV("heading_last_rate_rad_s", ToFixed(last_heading_rate_rad_s_, 3)));
  kvs.push_back(KV::KV("heading_range_violation_count", std::to_string(heading_range_violation_count_)));
  kvs.push_back(KV::KV("heading_step_violation_count", std::to_string(heading_jump_violation_count_)));
  kvs.push_back(KV::KV("heading_rate_violation_count", std::to_string(heading_rate_violation_count_)));
  kvs.push_back(KV::KV("heading_gyro_mismatch_count", std::to_string(heading_gyro_mismatch_count_)));
  // B9: INS timestamp rollback protection
  kvs.push_back(KV::KV("ins_stamp_rollback_count", std::to_string(ins_stamp_rollback_count_)));
  kvs.push_back(KV::KV("cone_last_map_size", std::to_string(cone_last_map_size_)));
  kvs.push_back(KV::KV("cone_unique_id_seen", std::to_string(seen_cone_ids_.size())));
  kvs.push_back(KV::KV("cone_new_id_count", std::to_string(cone_new_id_count_)));
  kvs.push_back(KV::KV("cone_max_id_seen", std::to_string(cone_max_id_seen_)));
  kvs.push_back(KV::KV("cone_non_monotonic_new_id_count", std::to_string(cone_non_monotonic_new_id_count_)));
  kvs.push_back(KV::KV("cone_zero_id_count", std::to_string(cone_zero_id_count_)));
  kvs.push_back(KV::KV("cone_stale_publish_count", std::to_string(cone_stale_publish_count_)));
  kvs.push_back(KV::KV("debug_topics_enabled", debug_topics_cfg_.enabled ? "true" : "false"));
  kvs.push_back(KV::KV("map_frozen", mapper_.map_frozen() ? "true" : "false"));
  kvs.push_back(KV::KV("map_size", std::to_string(mapper_.map_size())));
  kvs.push_back(KV::KV("map_update_seq", std::to_string(mapper_.map_update_seq())));
  kvs.push_back(KV::KV("map_checkpoint_available", mapper_.has_checkpoint() ? "true" : "false"));
  kvs.push_back(KV::KV("map_persistence_enabled", map_persistence_cfg_.enabled ? "true" : "false"));
  kvs.push_back(KV::KV("map_version_tag", map_persistence_cfg_.version_tag));
  kvs.push_back(KV::KV("map_save_path", map_persistence_cfg_.save_path));
  kvs.push_back(KV::KV("map_save_count", std::to_string(map_save_count_)));
  kvs.push_back(KV::KV("map_load_count", std::to_string(map_load_count_)));
  kvs.push_back(KV::KV("map_save_fail_count", std::to_string(map_save_fail_count_)));
  kvs.push_back(KV::KV("map_load_fail_count", std::to_string(map_load_fail_count_)));
  kvs.push_back(KV::KV("map_last_save_time_sec", ToFixed(map_last_save_time_sec_, 3)));
  kvs.push_back(KV::KV("map_last_load_time_sec", ToFixed(map_last_load_time_sec_, 3)));
  kvs.push_back(KV::KV("drift_monitor_enabled", drift_monitor_cfg_.enabled ? "true" : "false"));
  kvs.push_back(KV::KV("drift_bad_frames", std::to_string(drift_bad_frames_)));
  kvs.push_back(KV::KV("drift_last_frame_bad", drift_last_frame_bad_ ? "true" : "false"));
  kvs.push_back(KV::KV("drift_cooldown_frames", std::to_string(drift_cooldown_frames_)));
  kvs.push_back(KV::KV("drift_event_count", std::to_string(drift_event_count_)));
  kvs.push_back(KV::KV("drift_rollback_count", std::to_string(drift_rollback_count_)));
  kvs.push_back(KV::KV("drift_reset_count", std::to_string(drift_reset_count_)));
  kvs.push_back(KV::KV("drift_freeze_count", std::to_string(drift_freeze_count_)));
  if (has_last_map_stats_)
  {
    kvs.push_back(KV::KV("mapper_input_count", std::to_string(last_map_stats_.input_count)));
    kvs.push_back(KV::KV("mapper_bbox_reject_count", std::to_string(last_map_stats_.bbox_reject_count)));
    kvs.push_back(KV::KV("mapper_geometry_reject_count", std::to_string(last_map_stats_.geometry_reject_count)));
    kvs.push_back(KV::KV("mapper_conf_add_reject_count", std::to_string(last_map_stats_.conf_add_reject_count)));
    kvs.push_back(KV::KV("mapper_conf_merge_reject_count", std::to_string(last_map_stats_.conf_merge_reject_count)));
    kvs.push_back(KV::KV("mapper_frozen_reject_count", std::to_string(last_map_stats_.frozen_reject_count)));
    kvs.push_back(KV::KV("mapper_merged_count", std::to_string(last_map_stats_.merged_count)));
    kvs.push_back(KV::KV("mapper_inserted_count", std::to_string(last_map_stats_.inserted_count)));
    kvs.push_back(KV::KV("mapper_local_output_count", std::to_string(last_map_stats_.local_output_count)));
    kvs.push_back(KV::KV("mapper_mean_match_distance", ToFixed(last_map_stats_.mean_match_distance, 3)));
    kvs.push_back(KV::KV("mapper_max_match_distance", ToFixed(last_map_stats_.max_match_distance, 3)));
  }

  diag_helper_.PublishStatus("localization_entry_health", "localization_ros/location_node",
                             level, message, kvs, stamp, force);
}

void LocationNode::imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr &msg)
{
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

  // B9: INS timestamp rollback protection
  if (has_last_ins_stamp_ && stamp < last_ins_stamp_)
  {
    ++ins_stamp_rollback_count_;
    ROS_WARN_THROTTLE(1.0,
                      "[localization] INS timestamp rollback detected: current=%.6f < previous=%.6f (delta=%.6f sec). "
                      "Rejecting message to prevent state corruption.",
                      stamp.toSec(), last_ins_stamp_.toSec(), (last_ins_stamp_ - stamp).toSec());
    return;  // Reject message with rollback timestamp
  }

  updateHeadingSanity(*msg, stamp);

  localization_core::Asensing core_msg = ToCore(*msg);

  // 缓存 INS 状态用于时间戳插值
  {
    ins_buffer_.push_back({stamp, core_msg});
    while (ins_buffer_.size() > kInsBufferSize)
    {
      ins_buffer_.pop_front();
    }
  }

  localization_core::CarState state;

  if (mapper_.UpdateFromIns(core_msg, &state))
  {
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

  // B9: Update last INS timestamp after successful processing
  last_ins_stamp_ = stamp;
  has_last_ins_stamp_ = true;

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
  const int input_count = static_cast<int>(detections.detections.size());
  last_input_cone_count_ = input_count;
  localization_core::ConeMap map;
  localization_core::PointCloudPtr cloud;
  localization_core::MapUpdateStats map_stats;

  if (!shouldProcessConeFusion())
  {
    ++mapper_drop_count_;
    has_last_map_stats_ = false;
    last_cone_update_success_ = false;
    last_cone_frame_good_ = false;
    last_local_cone_count_ = 0;
    last_cone_match_ratio_ = 0.0;
    if (mapper_recovery_cfg_.publish_stale_map_on_failure && has_last_map_msg_)
    {
      autodrive_msgs::HUAT_ConeMap stale_map = last_map_msg_;
      stale_map.header.stamp = stamp;
      stale_map.header.frame_id = world_frame_;
      map_pub_.publish(stale_map);
      ++cone_stale_publish_count_;
    }
    publishEntryHealth("cone_ins_only", stamp);
    return;
  }

  const bool update_success = mapper_.UpdateFromCones(detections, &map, &cloud, &map_stats);
  if (update_success)
  {
    last_map_stats_ = map_stats;
    has_last_map_stats_ = true;
  }
  last_cone_update_success_ = update_success;
  last_local_cone_count_ = update_success ? map_stats.local_output_count : 0;
  last_cone_match_ratio_ = (input_count > 0 && update_success)
                               ? static_cast<double>(last_local_cone_count_) / static_cast<double>(input_count)
                               : 0.0;

  const bool frame_good = update_success &&
                          (last_local_cone_count_ >= mapper_recovery_cfg_.min_local_cones_for_tracking) &&
                          (last_cone_match_ratio_ >= mapper_recovery_cfg_.min_match_ratio_for_tracking);
  last_cone_frame_good_ = frame_good;
  updateMapperStateMachine(frame_good);

  if (!update_success)
  {
    ++mapper_drop_count_;
    has_last_map_stats_ = false;
    if (mapper_recovery_cfg_.publish_stale_map_on_failure && has_last_map_msg_)
    {
      autodrive_msgs::HUAT_ConeMap stale_map = last_map_msg_;
      stale_map.header.stamp = stamp;
      stale_map.header.frame_id = world_frame_;
      map_pub_.publish(stale_map);
      ++cone_stale_publish_count_;
    }
    publishEntryHealth("cone_update_failed", stamp);
    return;
  }

  if (evaluateDriftAndRecover(map_stats, stamp))
  {
    publishEntryHealth("cone_drift_recovered", stamp);
    return;
  }

  if (!frame_good)
  {
    ROS_WARN_THROTTLE(2.0,
                      "[location] weak cone frame: local=%d input=%d match_ratio=%.3f state=%s",
                      last_local_cone_count_, input_count, last_cone_match_ratio_,
                      mapperStateName(static_cast<int>(mapper_state_)));
  }

  autodrive_msgs::HUAT_ConeMap out_map;
  ToRos(map, &out_map);
  out_map.header.stamp = stamp;
  out_map.header.frame_id = world_frame_;
  updateConeMapLifecycleStats(out_map);
  map_pub_.publish(out_map);
  last_map_msg_ = out_map;
  has_last_map_msg_ = true;
  publishMapperDebugTopics(map_stats, stamp);

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

  publishEntryHealth(frame_good ? "cone" : "cone_degraded", stamp);
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
