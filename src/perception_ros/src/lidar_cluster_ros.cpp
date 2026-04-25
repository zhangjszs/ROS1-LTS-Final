#include <ros/serialization.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/core/types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <perception_ros/lidar_cluster_ros.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace perception_ros {

namespace {

// Updated cone type constants: Removed ORANGE, replaced with YELLOW_SMALL/YELLOW_BIG
// 0=BLUE, 1=YELLOW_SMALL, 2=YELLOW_BIG, 3=RED, 4=NONE
constexpr std::uint8_t kConeBlue = 0;
constexpr std::uint8_t kConeYellowSmall = 1;
constexpr std::uint8_t kConeYellowBig = 2;
constexpr std::uint8_t kConeRed = 3;
constexpr std::uint8_t kConeNone = 4;

std::uint8_t classifyConeTypeBySize(const ConeDetection& det, bool enable_size_typing,
                                    double big_height_threshold, double big_area_threshold,
                                    bool enable_position_coloring = false) {
  // LiDAR-only pipeline: geometry typing emits YELLOW_SMALL/YELLOW_BIG/NONE.
  // When enable_position_coloring is true (no vision available), use position-based coloring:
  //   Y > 0 (left side)  -> RED
  //   Y < 0 (right side) -> BLUE
  // This provides "left-red right-blue" visualization when camera data is unavailable.
  if (!enable_size_typing) {
    return kConeNone;
  }

  if (enable_position_coloring) {
    const double y_pos = static_cast<double>(det.centroid.y);
    if (y_pos > 0.0) {
      return kConeRed;
    }
    return kConeBlue;
  }

  const double height = std::max(0.0, static_cast<double>(det.max.z - det.min.z));
  const double width_x = std::max(0.0, static_cast<double>(det.max.x - det.min.x));
  const double width_y = std::max(0.0, static_cast<double>(det.max.y - det.min.y));
  const double footprint_area = width_x * width_y;

  if (height >= big_height_threshold || footprint_area >= big_area_threshold) {
    return kConeYellowBig;
  }
  return kConeYellowSmall;
}

bool LoadIntVector(const ros::NodeHandle& nh, const std::string& key, std::vector<int>& out) {
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(key, value)) {
    return false;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  out.clear();
  out.reserve(static_cast<size_t>(value.size()));
  for (int i = 0; i < value.size(); ++i) {
    if (value[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      out.push_back(static_cast<int>(value[i]));
    } else if (value[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      out.push_back(static_cast<int>(static_cast<double>(value[i])));
    } else {
      return false;
    }
  }
  return true;
}

bool LoadDoubleVector(const ros::NodeHandle& nh, const std::string& key, std::vector<double>& out) {
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(key, value)) {
    return false;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  out.clear();
  out.reserve(static_cast<size_t>(value.size()));
  for (int i = 0; i < value.size(); ++i) {
    if (value[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      out.push_back(static_cast<double>(value[i]));
    } else if (value[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      out.push_back(static_cast<double>(static_cast<int>(value[i])));
    } else {
      return false;
    }
  }
  return true;
}

}  // namespace

LidarClusterRos::LidarClusterRos(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)), tf_listener_(tf_buffer_) {
  loadParams();
  core_.Configure(config_);

  sub_point_cloud_ = nh_.subscribe(input_topic_, 2, &LidarClusterRos::pointCallback, this);

  passthrough_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(passthrough_topic_, 1);
  no_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(no_ground_topic_, 1);
  cones_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cones_topic_, 1);
  detections_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeDetections>(detections_topic_, 10);
  fused_detections_pub_ =
      nh_.advertise<autodrive_msgs::HUAT_FusedConeDetections>(fused_detections_topic_, 10);

  // G6: Debug visualization (optional, default off)
  private_nh_.param<bool>("debug/publish_markers", debug_publish_markers_, false);
  if (debug_publish_markers_) {
    debug_marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("perception/debug/cone_markers", 1);
    ROS_INFO("Debug cone markers enabled on perception/debug/cone_markers");
  }

  if (fusion_enabled_) {
    camera_info_sub_ =
        nh_.subscribe(fusion_camera_info_topic_, 1, &LidarClusterRos::cameraInfoCallback, this);

    fusion_cloud_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
        nh_, input_topic_, fusion_sync_queue_size_);
    fusion_vision_sub_ =
        std::make_unique<message_filters::Subscriber<autodrive_msgs::HUAT_VisionDetections>>(
            nh_, fusion_vision_topic_, fusion_sync_queue_size_);

    if (fusion_sync_policy_ == "exact") {
      fusion_sync_exact_ = std::make_unique<message_filters::Synchronizer<ExactFusionSyncPolicy>>(
          ExactFusionSyncPolicy(fusion_sync_queue_size_), *fusion_cloud_sub_, *fusion_vision_sub_);
      fusion_sync_exact_->registerCallback(
          boost::bind(&LidarClusterRos::syncPairCallback, this, _1, _2));
    } else {
      fusion_sync_approx_ = std::make_unique<message_filters::Synchronizer<ApproxFusionSyncPolicy>>(
          ApproxFusionSyncPolicy(fusion_sync_queue_size_), *fusion_cloud_sub_, *fusion_vision_sub_);
      fusion_sync_approx_->setMaxIntervalDuration(ros::Duration(fusion_sync_slop_sec_));
      fusion_sync_approx_->registerCallback(
          boost::bind(&LidarClusterRos::syncPairCallback, this, _1, _2));
    }

    ROS_INFO(
        "[perception] Fusion enabled: topic='%s' camera_info='%s' policy=%s "
        "queue=%d slop=%.3fs",
        fusion_vision_topic_.c_str(), fusion_camera_info_topic_.c_str(),
        fusion_sync_policy_.c_str(), fusion_sync_queue_size_, fusion_sync_slop_sec_);
  }

  // Initialize distortion compensator V2 (支持time字段、预积分、外参)
  auto compensator_config = DistortionCompensatorV2Config::LoadFromRos(private_nh_);
  if (compensator_config.enable) {
    compensator_ = std::make_unique<DistortionCompensatorV2>(nh_, compensator_config);
  }

  // ── Diagnostics 初始化 ──────────────────────────────────────────
  {
    std::string diag_topic, global_diag_topic;
    private_nh_.param<std::string>("diagnostics_topic", diag_topic,
                                   fsd_common::topic_contract::kPerceptionDiagnostics);
    private_nh_.param<std::string>("global_diagnostics_topic", global_diag_topic,
                                   fsd_common::topic_contract::kDiagnosticsGlobal);
    private_nh_.param<double>("diagnostics/rate_hz", diag_rate_hz_, 2.0);
    private_nh_.param<int>("diagnostics/low_detection_threshold", diag_low_detection_threshold_, 2);
    private_nh_.param<int>("diagnostics/zero_frames_to_warn", diag_zero_frames_to_warn_, 3);
    private_nh_.param<int>("diagnostics/zero_frames_to_error", diag_zero_frames_to_error_, 8);
    private_nh_.param<int>("diagnostics/low_frames_to_warn", diag_low_frames_to_warn_, 5);
    private_nh_.param<int>("diagnostics/recovery_frames", diag_recovery_frames_, 3);
    private_nh_.param<double>("diagnostics/latency_warn_ms", diag_latency_warn_ms_, 15.0);
    private_nh_.param<double>("stamp/max_drift_sec", stamp_max_drift_sec_,
                              fsd_common::stamp_contract::kMaxStampDriftSec);

    fsd_common::DiagnosticsHelper::Config dcfg;
    dcfg.local_topic = diag_topic;
    dcfg.global_topic = global_diag_topic;
    dcfg.publish_global = true;
    dcfg.rate_hz = diag_rate_hz_;
    diag_helper_.Init(nh_, dcfg);
  }

  ROS_INFO("lidar cluster finished initialization");
}

bool LidarClusterRos::IsLegacyPollMode() const {
  return pipeline_mode_ == "legacy_poll";
}

double LidarClusterRos::LegacyPollHz() const {
  return legacy_poll_hz_;
}

void LidarClusterRos::loadParams() {
  if (!private_nh_.param<std::string>("topics/input", input_topic_, "points/raw")) {
    ROS_DEBUG_STREAM("Did not load topics/input. Standard value is: " << input_topic_);
  }
  if (!private_nh_.param<std::string>("topics/points/passthrough", passthrough_topic_,
                                      "points/passthrough")) {
    ROS_DEBUG_STREAM(
        "Did not load topics/points/passthrough. Standard value is: " << passthrough_topic_);
  }
  if (!private_nh_.param<std::string>("topics/points/no_ground", no_ground_topic_,
                                      "points/no_ground")) {
    ROS_DEBUG_STREAM(
        "Did not load topics/points/no_ground. Standard value is: " << no_ground_topic_);
  }
  if (!private_nh_.param<std::string>("topics/points/cones", cones_topic_, "points/cones")) {
    ROS_DEBUG_STREAM("Did not load topics/points/cones. Standard value is: " << cones_topic_);
  }
  if (!private_nh_.param<std::string>("topics/detections", detections_topic_, "detections")) {
    ROS_DEBUG_STREAM("Did not load topics/detections. Standard value is: " << detections_topic_);
  }
  private_nh_.param<std::string>("topics/fused_detections", fused_detections_topic_,
                                 std::string(fsd_common::topic_contract::kFusedConeDetections));

  // 性能优化选项
  if (!private_nh_.param<bool>("use_point_cloud_pool", use_point_cloud_pool_, false)) {
    ROS_DEBUG_STREAM(
        "Did not load use_point_cloud_pool. Standard value is: " << use_point_cloud_pool_);
  }
  if (use_point_cloud_pool_) {
    ROS_INFO("Point cloud pool enabled for memory optimization");
  }

  if (!private_nh_.param<std::string>("pipeline_mode", pipeline_mode_, "event_driven")) {
    ROS_DEBUG_STREAM("Did not load pipeline_mode. Standard value is: " << pipeline_mode_);
  }
  if (pipeline_mode_ != "event_driven" && pipeline_mode_ != "legacy_poll") {
    ROS_WARN_STREAM("Invalid pipeline_mode: " << pipeline_mode_ << ", fallback to event_driven");
    pipeline_mode_ = "event_driven";
  }
  if (!private_nh_.param<double>("legacy_poll_hz", legacy_poll_hz_, 10.0)) {
    ROS_DEBUG_STREAM("Did not load legacy_poll_hz. Standard value is: " << legacy_poll_hz_);
  }
  if (legacy_poll_hz_ <= 0.0) {
    ROS_WARN_STREAM("Invalid legacy_poll_hz: " << legacy_poll_hz_ << ", fallback to 10.0");
    legacy_poll_hz_ = 10.0;
  }
  if (!private_nh_.param<bool>("input_guard/enable", input_guard_enable_, true)) {
    ROS_DEBUG_STREAM("Did not load input_guard/enable. Standard value is: " << input_guard_enable_);
  }
  if (!private_nh_.param<int>("input_guard/max_points", input_guard_max_points_, 500000)) {
    ROS_DEBUG_STREAM(
        "Did not load input_guard/max_points. Standard value is: " << input_guard_max_points_);
  }
  if (!private_nh_.param<bool>("input_guard/filter_invalid_points",
                               input_guard_filter_invalid_points_, true)) {
    ROS_DEBUG_STREAM("Did not load input_guard/filter_invalid_points. Standard value is: "
                     << input_guard_filter_invalid_points_);
  }
  if (input_guard_max_points_ <= 0) {
    ROS_WARN_STREAM("Invalid input_guard/max_points: " << input_guard_max_points_
                                                       << ", fallback to 500000");
    input_guard_max_points_ = 500000;
  }

  if (!private_nh_.param<double>("sensor_height", config_.sensor_height, 0.135)) {
    ROS_DEBUG_STREAM("Did not load sensor_height. Standard value is: " << config_.sensor_height);
  }
  if (!private_nh_.param<int>("sensor_model", config_.sensor_model, 16)) {
    ROS_DEBUG_STREAM("Did not load sensor_model. Standard value is: " << config_.sensor_model);
  }
  if (!private_nh_.param<int>("ransac/num_iter", config_.ransac.num_iter, 3)) {
    ROS_DEBUG_STREAM(
        "Did not load ransac/num_iter. Standard value is: " << config_.ransac.num_iter);
  }
  if (!private_nh_.param<int>("ransac/num_lpr", config_.ransac.num_lpr, 5)) {
    ROS_DEBUG_STREAM("Did not load ransac/num_lpr. Standard value is: " << config_.ransac.num_lpr);
  }
  if (!private_nh_.param<double>("ransac/th_seeds", config_.ransac.th_seeds, 0.03)) {
    ROS_DEBUG_STREAM(
        "Did not load ransac/th_seeds. Standard value is: " << config_.ransac.th_seeds);
  }
  if (!private_nh_.param<double>("ransac/th_dist", config_.ransac.th_dist, 0.03)) {
    ROS_DEBUG_STREAM("Did not load ransac/th_dist. Standard value is: " << config_.ransac.th_dist);
  }
  // RANSAC 优化参数
  private_nh_.param<bool>("ransac/enable_zone", config_.ransac.enable_zone, true);
  if (!LoadDoubleVector(private_nh_, "ransac/zone_boundaries", config_.ransac.zone_boundaries)) {
    config_.ransac.zone_boundaries = {10.0, 20.0, 30.0};  // 默认值
  }
  private_nh_.param<bool>("ransac/adaptive_threshold", config_.ransac.adaptive_threshold, true);
  private_nh_.param<double>("ransac/th_dist_far_scale", config_.ransac.th_dist_far_scale, 2.0);
  private_nh_.param<double>("ransac/min_normal_z", config_.ransac.min_normal_z, 0.8);
  private_nh_.param<bool>("ransac/progressive_iteration", config_.ransac.progressive_iteration,
                          true);
  if (!private_nh_.param<int>("road_type", config_.road_type, 2)) {
    ROS_DEBUG_STREAM("Did not load road_type. Standard value is: " << config_.road_type);
  }
  if (!private_nh_.param<std::string>("ground_method", config_.ground_method, "fgs")) {
    ROS_DEBUG_STREAM("Did not load ground_method. Standard value is: " << config_.ground_method);
  }
  if (!private_nh_.param<bool>("ground_watchdog/enable", ground_watchdog_enable_, true)) {
    ROS_DEBUG_STREAM(
        "Did not load ground_watchdog/enable. Standard value is: " << ground_watchdog_enable_);
  }
  if (!private_nh_.param<double>("ground_watchdog/warn_ms", ground_watchdog_warn_ms_, 8.0)) {
    ROS_DEBUG_STREAM(
        "Did not load ground_watchdog/warn_ms. Standard value is: " << ground_watchdog_warn_ms_);
  }
  if (!private_nh_.param<int>("ground_watchdog/warn_consecutive_frames",
                              ground_watchdog_warn_frames_, 5)) {
    ROS_DEBUG_STREAM("Did not load ground_watchdog/warn_consecutive_frames. Standard value is: "
                     << ground_watchdog_warn_frames_);
  }
  if (ground_watchdog_warn_frames_ < 1) {
    ROS_WARN_STREAM("Invalid ground_watchdog/warn_consecutive_frames: "
                    << ground_watchdog_warn_frames_ << ", fallback to 1");
    ground_watchdog_warn_frames_ = 1;
  }
  if (!private_nh_.param<std::string>("roi/mode", config_.roi.mode, "track")) {
    ROS_DEBUG_STREAM("Did not load roi/mode. Standard value is: " << config_.roi.mode);
  }
  if (!private_nh_.param<bool>("roi/use_point_clip", config_.roi.use_point_clip, false)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/use_point_clip. Standard value is: " << config_.roi.use_point_clip);
  }
  if (!private_nh_.param<double>("roi/z_min", config_.roi.z_min, -1.0)) {
    ROS_DEBUG_STREAM("Did not load roi/z_min. Standard value is: " << config_.roi.z_min);
  }
  if (!private_nh_.param<double>("roi/z_max", config_.roi.z_max, 0.7)) {
    ROS_DEBUG_STREAM("Did not load roi/z_max. Standard value is: " << config_.roi.z_max);
  }
  if (!private_nh_.param<double>("roi/skidpad/x_min", config_.roi.skidpad.x_min, 0.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/skidpad/x_min. Standard value is: " << config_.roi.skidpad.x_min);
  }
  if (!private_nh_.param<double>("roi/skidpad/x_max", config_.roi.skidpad.x_max, 10.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/skidpad/x_max. Standard value is: " << config_.roi.skidpad.x_max);
  }
  if (!private_nh_.param<double>("roi/skidpad/y_min", config_.roi.skidpad.y_min, -3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/skidpad/y_min. Standard value is: " << config_.roi.skidpad.y_min);
  }
  if (!private_nh_.param<double>("roi/skidpad/y_max", config_.roi.skidpad.y_max, 3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/skidpad/y_max. Standard value is: " << config_.roi.skidpad.y_max);
  }
  if (!private_nh_.param<double>("roi/accel/x_min", config_.roi.accel.x_min, 0.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/accel/x_min. Standard value is: " << config_.roi.accel.x_min);
  }
  if (!private_nh_.param<double>("roi/accel/x_max", config_.roi.accel.x_max, 100.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/accel/x_max. Standard value is: " << config_.roi.accel.x_max);
  }
  if (!private_nh_.param<double>("roi/accel/y_min", config_.roi.accel.y_min, -3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/accel/y_min. Standard value is: " << config_.roi.accel.y_min);
  }
  if (!private_nh_.param<double>("roi/accel/y_max", config_.roi.accel.y_max, 3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/accel/y_max. Standard value is: " << config_.roi.accel.y_max);
  }
  if (!private_nh_.param<double>("roi/track/x_min", config_.roi.track.x_min, 0.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/track/x_min. Standard value is: " << config_.roi.track.x_min);
  }
  if (!private_nh_.param<double>("roi/track/x_max", config_.roi.track.x_max, 20.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/track/x_max. Standard value is: " << config_.roi.track.x_max);
  }
  if (!private_nh_.param<double>("roi/track/y_min", config_.roi.track.y_min, -3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/track/y_min. Standard value is: " << config_.roi.track.y_min);
  }
  if (!private_nh_.param<double>("roi/track/y_max", config_.roi.track.y_max, 3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/track/y_max. Standard value is: " << config_.roi.track.y_max);
  }
  if (!private_nh_.param<double>("roi/custom/x_min", config_.roi.custom.x_min, 0.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/custom/x_min. Standard value is: " << config_.roi.custom.x_min);
  }
  if (!private_nh_.param<double>("roi/custom/x_max", config_.roi.custom.x_max, 20.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/custom/x_max. Standard value is: " << config_.roi.custom.x_max);
  }
  if (!private_nh_.param<double>("roi/custom/y_min", config_.roi.custom.y_min, -3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/custom/y_min. Standard value is: " << config_.roi.custom.y_min);
  }
  if (!private_nh_.param<double>("roi/custom/y_max", config_.roi.custom.y_max, 3.0)) {
    ROS_DEBUG_STREAM(
        "Did not load roi/custom/y_max. Standard value is: " << config_.roi.custom.y_max);
  }
  if (!private_nh_.param<bool>("filters/sor/enable", config_.filters.sor.enable, false)) {
    ROS_DEBUG_STREAM(
        "Did not load filters/sor/enable. Standard value is: " << config_.filters.sor.enable);
  }
  if (!private_nh_.param<int>("filters/sor/mean_k", config_.filters.sor.mean_k, 50)) {
    ROS_DEBUG_STREAM(
        "Did not load filters/sor/mean_k. Standard value is: " << config_.filters.sor.mean_k);
  }
  if (!private_nh_.param<double>("filters/sor/stddev_mul", config_.filters.sor.stddev_mul, 1.0)) {
    ROS_DEBUG_STREAM("Did not load filters/sor/stddev_mul. Standard value is: "
                     << config_.filters.sor.stddev_mul);
  }
  // 距离自适应体素滤波参数（唯一体素滤波路径）
  if (!private_nh_.param<bool>("filters/distance_adaptive_voxel/enable",
                               config_.filters.distance_adaptive_voxel.enable, true)) {
    ROS_DEBUG_STREAM("Did not load filters/distance_adaptive_voxel/enable. Standard value is: "
                     << config_.filters.distance_adaptive_voxel.enable);
  }
  if (!private_nh_.param<double>("filters/distance_adaptive_voxel/near_leaf",
                                 config_.filters.distance_adaptive_voxel.near_leaf, 0.08)) {
    ROS_DEBUG_STREAM("Did not load filters/distance_adaptive_voxel/near_leaf. Standard value is: "
                     << config_.filters.distance_adaptive_voxel.near_leaf);
  }
  if (!private_nh_.param<double>("filters/distance_adaptive_voxel/far_leaf",
                                 config_.filters.distance_adaptive_voxel.far_leaf, 0.03)) {
    ROS_DEBUG_STREAM("Did not load filters/distance_adaptive_voxel/far_leaf. Standard value is: "
                     << config_.filters.distance_adaptive_voxel.far_leaf);
  }
  if (!private_nh_.param<double>("filters/distance_adaptive_voxel/dist_threshold",
                                 config_.filters.distance_adaptive_voxel.dist_threshold, 10.0)) {
    ROS_DEBUG_STREAM(
        "Did not load filters/distance_adaptive_voxel/dist_threshold. Standard value is: "
        << config_.filters.distance_adaptive_voxel.dist_threshold);
  }
  // 强度滤波参数
  if (!private_nh_.param<bool>("filters/intensity/enable", config_.filters.intensity.enable,
                               false)) {
    ROS_DEBUG_STREAM("Did not load filters/intensity/enable. Standard value is: "
                     << config_.filters.intensity.enable);
  }
  {
    double min_intensity_d = static_cast<double>(config_.filters.intensity.min_intensity);
    if (!private_nh_.param<double>("filters/intensity/min_intensity", min_intensity_d, 5.0)) {
      ROS_DEBUG_STREAM(
          "Did not load filters/intensity/min_intensity. Standard value is: " << min_intensity_d);
    }
    config_.filters.intensity.min_intensity = static_cast<float>(min_intensity_d);
  }
  // CropBox优化开关
  if (!private_nh_.param<bool>("filters/use_cropbox", config_.filters.use_cropbox, true)) {
    ROS_DEBUG_STREAM(
        "Did not load filters/use_cropbox. Standard value is: " << config_.filters.use_cropbox);
  }
  // 柱状障碍物滤波参数（基于局部z跨度去除树/墙壁）
  private_nh_.param<bool>("filters/obstacle_height/enable", config_.filters.obstacle_height.enable,
                          true);
  private_nh_.param<double>("filters/obstacle_height/grid_size",
                            config_.filters.obstacle_height.grid_size, 0.5);
  private_nh_.param<double>("filters/obstacle_height/max_z_span",
                            config_.filters.obstacle_height.max_z_span, 0.5);
  private_nh_.param<int>("filters/obstacle_height/min_points_to_judge",
                         config_.filters.obstacle_height.min_points_to_judge, 3);
  private_nh_.param<double>("filters/obstacle_height/min_distance",
                            config_.filters.obstacle_height.min_distance, 10.0);

  // FGS (Fast Ground Segmentation) 参数
  private_nh_.param<int>("fgs/num_sectors", config_.fgs.num_sectors, 32);
  private_nh_.param<int>("fgs/num_bins", config_.fgs.num_bins, 80);
  private_nh_.param<double>("fgs/max_range", config_.fgs.max_range, 80.0);
  private_nh_.param<double>("fgs/min_range", config_.fgs.min_range, 0.1);
  // fgs/sensor_height inherits top-level sensor_height if not explicitly set
  private_nh_.param<double>("fgs/sensor_height", config_.fgs.sensor_height, config_.sensor_height);
  private_nh_.param<double>("fgs/th_ground", config_.fgs.th_ground, 0.08);
  private_nh_.param<double>("fgs/th_ground_far", config_.fgs.th_ground_far, 0.15);
  private_nh_.param<double>("fgs/far_distance", config_.fgs.far_distance, 20.0);
  // 近距离参数（解决正前方地面分割问题）
  private_nh_.param<double>("fgs/near_distance", config_.fgs.near_distance, 2.0);
  private_nh_.param<double>("fgs/th_ground_near", config_.fgs.th_ground_near, 0.12);
  private_nh_.param<double>("fgs/max_slope", config_.fgs.max_slope, 0.3);
  private_nh_.param<double>("fgs/min_normal_z", config_.fgs.min_normal_z, 0.85);
  private_nh_.param<double>("fgs/max_height_diff", config_.fgs.max_height_diff, 0.3);
  private_nh_.param<bool>("fgs/use_neighbor_model", config_.fgs.use_neighbor_model, true);
  // 增量线段生长参数 (Zermas 2017)
  private_nh_.param<int>("fgs/max_segments_per_sector", config_.fgs.max_segments_per_sector, 4);
  private_nh_.param<double>("fgs/segment_merge_dist", config_.fgs.segment_merge_dist, 0.05);
  // 地面点精细化 (Zermas 2017)
  private_nh_.param<bool>("fgs/enable_refinement", config_.fgs.enable_refinement, false);
  // 代表点选择 (Himmelsbach 2010)
  private_nh_.param<bool>("fgs/use_lowest_n_mean", config_.fgs.use_lowest_n_mean, true);
  private_nh_.param<int>("fgs/lowest_n", config_.fgs.lowest_n, 3);
  // 扇区间平滑 (Himmelsbach 2010)
  private_nh_.param<bool>("fgs/enable_sector_smoothing", config_.fgs.enable_sector_smoothing, true);
  // 帧间时序平滑 (抑制闪烁)
  private_nh_.param<bool>("fgs/enable_temporal_smoothing", config_.fgs.enable_temporal_smoothing,
                          true);
  private_nh_.param<double>("fgs/temporal_alpha", config_.fgs.temporal_alpha, 0.3);
  // 地面判定阈值对称性
  private_nh_.param<double>("fgs/ground_below_factor", config_.fgs.ground_below_factor, 1.5);
  // temporal alpha 自适应（S弯/快速变向）
  private_nh_.param<bool>("fgs/enable_adaptive_alpha", config_.fgs.enable_adaptive_alpha, true);
  private_nh_.param<double>("fgs/adaptive_alpha_max", config_.fgs.adaptive_alpha_max, 0.9);
  private_nh_.param<double>("fgs/adaptive_alpha_threshold", config_.fgs.adaptive_alpha_threshold,
                            0.05);

  if (!private_nh_.param<bool>("patchworkpp/enable_rnr", config_.patchworkpp.enable_rnr, true)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/enable_rnr. Standard value is: "
                     << config_.patchworkpp.enable_rnr);
  }
  if (!private_nh_.param<bool>("patchworkpp/enable_rvpf", config_.patchworkpp.enable_rvpf, true)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/enable_rvpf. Standard value is: "
                     << config_.patchworkpp.enable_rvpf);
  }
  if (!private_nh_.param<bool>("patchworkpp/enable_tgr", config_.patchworkpp.enable_tgr, true)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/enable_tgr. Standard value is: "
                     << config_.patchworkpp.enable_tgr);
  }
  if (!private_nh_.param<int>("patchworkpp/num_iter", config_.patchworkpp.num_iter, 3)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/num_iter. Standard value is: " << config_.patchworkpp.num_iter);
  }
  if (!private_nh_.param<int>("patchworkpp/num_lpr", config_.patchworkpp.num_lpr, 20)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/num_lpr. Standard value is: " << config_.patchworkpp.num_lpr);
  }
  if (!private_nh_.param<int>("patchworkpp/num_min_pts", config_.patchworkpp.num_min_pts, 10)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/num_min_pts. Standard value is: "
                     << config_.patchworkpp.num_min_pts);
  }
  if (!private_nh_.param<int>("patchworkpp/num_zones", config_.patchworkpp.num_zones, 4)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/num_zones. Standard value is: " << config_.patchworkpp.num_zones);
  }
  if (!private_nh_.param<int>("patchworkpp/num_rings_of_interest",
                              config_.patchworkpp.num_rings_of_interest, 4)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/num_rings_of_interest. Standard value is: "
                     << config_.patchworkpp.num_rings_of_interest);
  }
  if (!private_nh_.param<double>("patchworkpp/rnr_ver_angle_thr",
                                 config_.patchworkpp.rnr_ver_angle_thr, -15.0)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/rnr_ver_angle_thr. Standard value is: "
                     << config_.patchworkpp.rnr_ver_angle_thr);
  }
  if (!private_nh_.param<double>("patchworkpp/rnr_intensity_thr",
                                 config_.patchworkpp.rnr_intensity_thr, 0.2)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/rnr_intensity_thr. Standard value is: "
                     << config_.patchworkpp.rnr_intensity_thr);
  }
  if (!private_nh_.param<double>("patchworkpp/th_seeds", config_.patchworkpp.th_seeds, 0.125)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/th_seeds. Standard value is: " << config_.patchworkpp.th_seeds);
  }
  if (!private_nh_.param<double>("patchworkpp/th_dist", config_.patchworkpp.th_dist, 0.125)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/th_dist. Standard value is: " << config_.patchworkpp.th_dist);
  }
  if (!private_nh_.param<double>("patchworkpp/th_seeds_v", config_.patchworkpp.th_seeds_v, 0.25)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/th_seeds_v. Standard value is: "
                     << config_.patchworkpp.th_seeds_v);
  }
  if (!private_nh_.param<double>("patchworkpp/th_dist_v", config_.patchworkpp.th_dist_v, 0.1)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/th_dist_v. Standard value is: " << config_.patchworkpp.th_dist_v);
  }
  if (!private_nh_.param<double>("patchworkpp/max_range", config_.patchworkpp.max_range, 80.0)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/max_range. Standard value is: " << config_.patchworkpp.max_range);
  }
  if (!private_nh_.param<double>("patchworkpp/min_range", config_.patchworkpp.min_range, 2.7)) {
    ROS_DEBUG_STREAM(
        "Did not load patchworkpp/min_range. Standard value is: " << config_.patchworkpp.min_range);
  }
  if (!private_nh_.param<double>("patchworkpp/uprightness_thr", config_.patchworkpp.uprightness_thr,
                                 0.707)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/uprightness_thr. Standard value is: "
                     << config_.patchworkpp.uprightness_thr);
  }
  if (!private_nh_.param<double>("patchworkpp/adaptive_seed_selection_margin",
                                 config_.patchworkpp.adaptive_seed_selection_margin, -1.2)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/adaptive_seed_selection_margin. Standard value is: "
                     << config_.patchworkpp.adaptive_seed_selection_margin);
  }
  if (!private_nh_.param<int>("patchworkpp/max_flatness_storage",
                              config_.patchworkpp.max_flatness_storage, 1000)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/max_flatness_storage. Standard value is: "
                     << config_.patchworkpp.max_flatness_storage);
  }
  if (!private_nh_.param<int>("patchworkpp/max_elevation_storage",
                              config_.patchworkpp.max_elevation_storage, 1000)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/max_elevation_storage. Standard value is: "
                     << config_.patchworkpp.max_elevation_storage);
  }
  if (!LoadIntVector(private_nh_, "patchworkpp/num_sectors_each_zone",
                     config_.patchworkpp.num_sectors_each_zone)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/num_sectors_each_zone. Standard value is used.");
  }
  if (!LoadIntVector(private_nh_, "patchworkpp/num_rings_each_zone",
                     config_.patchworkpp.num_rings_each_zone)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/num_rings_each_zone. Standard value is used.");
  }
  if (!LoadDoubleVector(private_nh_, "patchworkpp/elevation_thr",
                        config_.patchworkpp.elevation_thr)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/elevation_thr. Standard value is used.");
  }
  if (!LoadDoubleVector(private_nh_, "patchworkpp/flatness_thr",
                        config_.patchworkpp.flatness_thr)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/flatness_thr. Standard value is used.");
  }
  // Patchwork++ 优化参数
  if (!private_nh_.param<double>("patchworkpp/th_dist_far_scale",
                                 config_.patchworkpp.th_dist_far_scale, 1.5)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/th_dist_far_scale. Standard value is: "
                     << config_.patchworkpp.th_dist_far_scale);
  }
  if (!private_nh_.param<double>("patchworkpp/min_normal_z", config_.patchworkpp.min_normal_z,
                                 0.7)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/min_normal_z. Standard value is: "
                     << config_.patchworkpp.min_normal_z);
  }
  if (!private_nh_.param<double>("patchworkpp/far_zone_min_pts_scale",
                                 config_.patchworkpp.far_zone_min_pts_scale, 2.0)) {
    ROS_DEBUG_STREAM("Did not load patchworkpp/far_zone_min_pts_scale. Standard value is: "
                     << config_.patchworkpp.far_zone_min_pts_scale);
  }
  if (!private_nh_.param<std::string>("str_range", config_.str_range, "15,30,45,60")) {
    ROS_DEBUG_STREAM("Did not load str_range. Standard value is: " << config_.str_range);
  }
  if (!private_nh_.param<std::string>("str_seg_distance", config_.str_seg_distance,
                                      "0.5,1.1,1.6,2.1,2.6")) {
    ROS_DEBUG_STREAM(
        "Did not load str_seg_distance. Standard value is: " << config_.str_seg_distance);
  }

  // 聚类配置
  if (!LoadDoubleVector(private_nh_, "cluster/distance_segments",
                        config_.cluster.distance_segments)) {
    ROS_DEBUG_STREAM("Did not load cluster/distance_segments. Using defaults.");
  }
  if (!LoadDoubleVector(private_nh_, "cluster/cluster_tolerance",
                        config_.cluster.cluster_tolerance)) {
    ROS_DEBUG_STREAM("Did not load cluster/cluster_tolerance. Using defaults.");
  }
  if (!private_nh_.param<bool>("cluster/adaptive_size/enable", config_.cluster.adaptive_size.enable,
                               true)) {
    ROS_DEBUG_STREAM("Did not load cluster/adaptive_size/enable. Standard value is: "
                     << config_.cluster.adaptive_size.enable);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/near_min_size",
                              config_.cluster.adaptive_size.near_min_size, 3)) {
    ROS_DEBUG_STREAM("Did not load cluster/adaptive_size/near_min_size. Standard value is: "
                     << config_.cluster.adaptive_size.near_min_size);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/near_max_size",
                              config_.cluster.adaptive_size.near_max_size, 100)) {
    ROS_DEBUG_STREAM("Did not load cluster/adaptive_size/near_max_size. Standard value is: "
                     << config_.cluster.adaptive_size.near_max_size);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/far_min_size",
                              config_.cluster.adaptive_size.far_min_size, 1)) {
    ROS_DEBUG_STREAM("Did not load cluster/adaptive_size/far_min_size. Standard value is: "
                     << config_.cluster.adaptive_size.far_min_size);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/far_max_size",
                              config_.cluster.adaptive_size.far_max_size, 30)) {
    ROS_DEBUG_STREAM("Did not load cluster/adaptive_size/far_max_size. Standard value is: "
                     << config_.cluster.adaptive_size.far_max_size);
  }
  if (!private_nh_.param<int>("cluster/min_cluster_size", config_.cluster.min_cluster_size, 1)) {
    ROS_DEBUG_STREAM("Did not load cluster/min_cluster_size. Standard value is: "
                     << config_.cluster.min_cluster_size);
  }
  if (!private_nh_.param<int>("cluster/max_cluster_size", config_.cluster.max_cluster_size, 50)) {
    ROS_DEBUG_STREAM("Did not load cluster/max_cluster_size. Standard value is: "
                     << config_.cluster.max_cluster_size);
  }
  // 聚类方法选择
  if (!private_nh_.param<std::string>("cluster/method", config_.cluster.method, "euclidean")) {
    ROS_DEBUG_STREAM("Did not load cluster/method. Standard value is: " << config_.cluster.method);
  }
  // DBSCAN 参数
  if (!private_nh_.param<double>("cluster/dbscan/eps", config_.cluster.dbscan.eps, 0.3)) {
    ROS_DEBUG_STREAM(
        "Did not load cluster/dbscan/eps. Standard value is: " << config_.cluster.dbscan.eps);
  }
  if (!private_nh_.param<int>("cluster/dbscan/min_pts", config_.cluster.dbscan.min_pts, 3)) {
    ROS_DEBUG_STREAM("Did not load cluster/dbscan/min_pts. Standard value is: "
                     << config_.cluster.dbscan.min_pts);
  }
  if (!private_nh_.param<bool>("cluster/dbscan/adaptive_eps", config_.cluster.dbscan.adaptive_eps,
                               true)) {
    ROS_DEBUG_STREAM("Did not load cluster/dbscan/adaptive_eps. Standard value is: "
                     << config_.cluster.dbscan.adaptive_eps);
  }
  if (!private_nh_.param<double>("cluster/dbscan/eps_near", config_.cluster.dbscan.eps_near,
                                 0.15)) {
    ROS_DEBUG_STREAM("Did not load cluster/dbscan/eps_near. Standard value is: "
                     << config_.cluster.dbscan.eps_near);
  }
  if (!private_nh_.param<double>("cluster/dbscan/eps_far", config_.cluster.dbscan.eps_far, 0.5)) {
    ROS_DEBUG_STREAM("Did not load cluster/dbscan/eps_far. Standard value is: "
                     << config_.cluster.dbscan.eps_far);
  }
  // VLP-16 参数
  if (!private_nh_.param<double>("cluster/vlp16/cluster_tolerance",
                                 config_.cluster.vlp16.cluster_tolerance, 0.3)) {
    ROS_DEBUG_STREAM("Did not load cluster/vlp16/cluster_tolerance. Standard value is: "
                     << config_.cluster.vlp16.cluster_tolerance);
  }
  if (!private_nh_.param<double>("cluster/vlp16/max_bbox_x", config_.cluster.vlp16.max_bbox_x,
                                 0.4)) {
    ROS_DEBUG_STREAM("Did not load cluster/vlp16/max_bbox_x. Standard value is: "
                     << config_.cluster.vlp16.max_bbox_x);
  }
  if (!private_nh_.param<double>("cluster/vlp16/max_bbox_y", config_.cluster.vlp16.max_bbox_y,
                                 0.4)) {
    ROS_DEBUG_STREAM("Did not load cluster/vlp16/max_bbox_y. Standard value is: "
                     << config_.cluster.vlp16.max_bbox_y);
  }
  if (!private_nh_.param<double>("cluster/vlp16/max_bbox_z", config_.cluster.vlp16.max_bbox_z,
                                 0.5)) {
    ROS_DEBUG_STREAM("Did not load cluster/vlp16/max_bbox_z. Standard value is: "
                     << config_.cluster.vlp16.max_bbox_z);
  }
  // point_clip 参数
  if (!private_nh_.param<double>("cluster/point_clip/min_distance",
                                 config_.cluster.point_clip.min_distance, 1.0)) {
    ROS_DEBUG_STREAM("Did not load cluster/point_clip/min_distance. Standard value is: "
                     << config_.cluster.point_clip.min_distance);
  }
  if (!private_nh_.param<double>("cluster/point_clip/max_distance",
                                 config_.cluster.point_clip.max_distance, 15.0)) {
    ROS_DEBUG_STREAM("Did not load cluster/point_clip/max_distance. Standard value is: "
                     << config_.cluster.point_clip.max_distance);
  }
  // FEC (Fast Euclidean Clustering) 参数
  if (!private_nh_.param<bool>("cluster/fec/enable", config_.cluster.fec.enable, true)) {
    ROS_DEBUG_STREAM(
        "Did not load cluster/fec/enable. Standard value is: " << config_.cluster.fec.enable);
  }
  if (!private_nh_.param<double>("cluster/fec/quality", config_.cluster.fec.quality, 0.3)) {
    ROS_DEBUG_STREAM(
        "Did not load cluster/fec/quality. Standard value is: " << config_.cluster.fec.quality);
  }
  // 多帧累积参数
  if (!private_nh_.param<bool>("cluster/multi_frame/enable", config_.cluster.multi_frame.enable,
                               false)) {
    ROS_DEBUG_STREAM("Did not load cluster/multi_frame/enable. Standard value is: "
                     << config_.cluster.multi_frame.enable);
  }
  if (!private_nh_.param<int>("cluster/multi_frame/num_frames",
                              config_.cluster.multi_frame.num_frames, 2)) {
    ROS_DEBUG_STREAM("Did not load cluster/multi_frame/num_frames. Standard value is: "
                     << config_.cluster.multi_frame.num_frames);
  }
  if (!private_nh_.param<double>("cluster/multi_frame/max_distance",
                                 config_.cluster.multi_frame.max_distance, 10.0)) {
    ROS_DEBUG_STREAM("Did not load cluster/multi_frame/max_distance. Standard value is: "
                     << config_.cluster.multi_frame.max_distance);
  }

  if (!private_nh_.param<double>("min_height", config_.min_height, -1)) {
    ROS_DEBUG("Did not load min_height.");
  }
  if (!private_nh_.param<double>("max_height", config_.max_height, -1)) {
    ROS_DEBUG("Did not load max_height.");
  }
  if (!private_nh_.param<double>("min_area", config_.min_area, -1)) {
    ROS_DEBUG("Did not load min_area.");
  }
  if (!private_nh_.param<double>("max_area", config_.max_area, -1)) {
    ROS_DEBUG("Did not load max_area.");
  }

  if (!private_nh_.param<double>("max_box_altitude", config_.max_box_altitude, 0)) {
    ROS_DEBUG("Did not load max_box_altitude.");
  }

  // ---- Confidence 参数加载（修复：之前YAML中的confidence/*参数从未被读取） ----
  private_nh_.param<double>("confidence/min_height", config_.confidence.min_height, 0.15);
  private_nh_.param<double>("confidence/max_height", config_.confidence.max_height, 0.5);
  private_nh_.param<double>("confidence/min_area", config_.confidence.min_area, 0.01);
  private_nh_.param<double>("confidence/max_area", config_.confidence.max_area, 0.15);
  private_nh_.param<double>("confidence/max_box_altitude", config_.confidence.max_box_altitude,
                            0.5);
  private_nh_.param<double>("confidence/min_aspect_ratio", config_.confidence.min_aspect_ratio,
                            1.5);
  private_nh_.param<double>("confidence/min_verticality", config_.confidence.min_verticality, 0.8);
  private_nh_.param<double>("confidence/max_linearity", config_.confidence.max_linearity, 0.85);
  private_nh_.param<double>("confidence/min_density_near", config_.confidence.min_density_near,
                            50.0);
  private_nh_.param<double>("confidence/min_density_far", config_.confidence.min_density_far, 10.0);
  private_nh_.param<double>("confidence/distance_threshold", config_.confidence.distance_threshold,
                            5.0);
  private_nh_.param<double>("confidence/min_intensity_mean", config_.confidence.min_intensity_mean,
                            30.0);
  private_nh_.param<double>("confidence/weight_size", config_.confidence.weight_size, 0.3);
  private_nh_.param<double>("confidence/weight_shape", config_.confidence.weight_shape, 0.25);
  private_nh_.param<double>("confidence/weight_density", config_.confidence.weight_density, 0.2);
  private_nh_.param<double>("confidence/weight_intensity", config_.confidence.weight_intensity,
                            0.15);
  private_nh_.param<double>("confidence/weight_position", config_.confidence.weight_position, 0.1);
  private_nh_.param<bool>("confidence/enable_model_fitting",
                          config_.confidence.enable_model_fitting, true);
  private_nh_.param<double>("confidence/model_fit_bonus", config_.confidence.model_fit_bonus, 0.2);
  private_nh_.param<double>("confidence/model_fit_penalty", config_.confidence.model_fit_penalty,
                            0.15);
  // 距离自适应置信度门槛
  private_nh_.param<double>("confidence/min_confidence_near",
                            config_.confidence.min_confidence_near, 0.0);
  private_nh_.param<double>("confidence/min_confidence_far", config_.confidence.min_confidence_far,
                            0.3);
  private_nh_.param<double>("confidence/confidence_ramp_start",
                            config_.confidence.confidence_ramp_start, 10.0);
  private_nh_.param<double>("confidence/confidence_ramp_end",
                            config_.confidence.confidence_ramp_end, 30.0);

  // Track semantic confidence (neighbor-context scoring)
  private_nh_.param<bool>("confidence/track_semantic/enable",
                          config_.confidence.track_semantic.enable, false);
  private_nh_.param<double>("confidence/track_semantic/weight",
                            config_.confidence.track_semantic.weight, 0.0);
  private_nh_.param<double>("confidence/track_semantic/expected_track_width",
                            config_.confidence.track_semantic.expected_track_width, 3.0);
  private_nh_.param<double>("confidence/track_semantic/expected_cone_spacing",
                            config_.confidence.track_semantic.expected_cone_spacing, 5.0);
  private_nh_.param<double>("confidence/track_semantic/spacing_tolerance",
                            config_.confidence.track_semantic.spacing_tolerance, 2.0);
  private_nh_.param<double>("confidence/track_semantic/width_tolerance",
                            config_.confidence.track_semantic.width_tolerance, 1.0);
  private_nh_.param<double>("confidence/track_semantic/isolation_radius",
                            config_.confidence.track_semantic.isolation_radius, 8.0);
  // Hard rejection thresholds for neighborhood filtering
  private_nh_.param<int>("confidence/track_semantic/min_neighbors_hard",
                         config_.confidence.track_semantic.min_neighbors_hard, 0);
  private_nh_.param<double>("confidence/track_semantic/max_isolation_distance",
                            config_.confidence.track_semantic.max_isolation_distance, 12.0);

  // 锥桶类型参数（当前仅根据几何尺寸区分小橙桶/大橙桶）
  private_nh_.param<bool>("cone_size_typing/enable", enable_cone_size_typing_, true);
  private_nh_.param<double>("cone_size_typing/big_height_threshold", big_cone_height_threshold_,
                            0.45);
  private_nh_.param<double>("cone_size_typing/big_area_threshold", big_cone_area_threshold_, 0.09);
  // Position-based coloring: when no vision available, color by Y position (left=RED, right=BLUE)
  private_nh_.param<bool>("cone_size_typing/position_coloring", enable_position_coloring_, true);

  // Fusion params (new contract). Legacy vision_inject keys are still supported.
  const bool has_fusion_cfg = private_nh_.hasParam("fusion/enabled");
  if (has_fusion_cfg) {
    private_nh_.param<bool>("fusion/enabled", fusion_enabled_, true);
    private_nh_.param<std::string>("fusion/topics/vision_detections", fusion_vision_topic_,
                                   std::string(fsd_common::topic_contract::kVisionDetections));
    private_nh_.param<std::string>("fusion/topics/camera_info", fusion_camera_info_topic_,
                                   "/camera/camera_info");
    private_nh_.param<std::string>("fusion/camera_frame", fusion_camera_frame_, "");

    private_nh_.param<std::string>("fusion/sync/policy", fusion_sync_policy_, "approximate");
    private_nh_.param<int>("fusion/sync/queue_size", fusion_sync_queue_size_, 20);
    private_nh_.param<double>("fusion/sync/slop_sec", fusion_sync_slop_sec_, 0.08);
    private_nh_.param<int>("fusion/sync/max_cache_size", fusion_max_cache_size_, 100);

    private_nh_.param<float>("fusion/projection/min_confidence", fusion_min_vision_confidence_,
                             300.0f);
    private_nh_.param<double>("fusion/projection/max_pixel_distance", fusion_max_pixel_distance_,
                              80.0);
    private_nh_.param<double>("fusion/projection/tf_timeout_sec", fusion_tf_timeout_sec_, 0.03);
    private_nh_.param<bool>("fusion/projection/require_camera_info", fusion_require_camera_info_,
                            true);
    private_nh_.param<bool>("fusion/projection/require_tf", fusion_require_tf_, true);
    private_nh_.param<bool>("fusion/projection/fallback_to_lidar_color",
                            fusion_fallback_to_lidar_color_, true);

    private_nh_.param<bool>("fusion/legacy_hfov_fallback/enable",
                            fusion_legacy_hfov_fallback_enable_, false);
    private_nh_.param<double>("fusion/legacy_hfov_fallback/match_angle_deg",
                              fusion_legacy_match_angle_deg_, 5.0);
    private_nh_.param<double>("fusion/legacy_hfov_fallback/camera_hfov_deg",
                              fusion_legacy_camera_hfov_deg_, 60.0);
    private_nh_.param<int>("fusion/legacy_hfov_fallback/camera_width_px",
                           fusion_legacy_camera_width_px_, 640);
  } else {
    // Legacy compatibility: map old vision_inject parameters to fusion behavior.
    private_nh_.param<bool>("vision_inject/enabled", fusion_enabled_, false);
    private_nh_.param<double>("vision_inject/max_age_sec", fusion_sync_slop_sec_, 0.15);
    private_nh_.param<float>("vision_inject/min_confidence", fusion_min_vision_confidence_, 300.0f);
    private_nh_.param<double>("vision_inject/match_angle_deg", fusion_legacy_match_angle_deg_, 5.0);
    private_nh_.param<double>("vision_inject/camera_hfov_deg", fusion_legacy_camera_hfov_deg_,
                              60.0);
    private_nh_.param<int>("vision_inject/camera_width_px", fusion_legacy_camera_width_px_, 640);
    fusion_sync_policy_ = "approximate";
    fusion_sync_queue_size_ = 20;
    fusion_max_cache_size_ = 100;
    fusion_vision_topic_ = fsd_common::topic_contract::kVisionDetections;
    fusion_camera_info_topic_ = "/camera/camera_info";
    fusion_require_camera_info_ = false;
    fusion_require_tf_ = false;
    fusion_fallback_to_lidar_color_ = true;
    fusion_legacy_hfov_fallback_enable_ = true;
    ROS_WARN_THROTTLE(5.0,
                      "[perception] Legacy vision_inject config detected. "
                      "Please migrate to fusion/* for calibrated projection fusion.");
  }

  if (fusion_sync_policy_ != "approximate" && fusion_sync_policy_ != "exact") {
    ROS_WARN_STREAM("Invalid fusion/sync/policy='" << fusion_sync_policy_
                                                   << "', fallback to approximate");
    fusion_sync_policy_ = "approximate";
  }
  if (fusion_sync_queue_size_ < 1) {
    fusion_sync_queue_size_ = 1;
  }
  if (fusion_max_cache_size_ < 10) {
    fusion_max_cache_size_ = 10;
  }
  if (fusion_sync_slop_sec_ < 0.0) {
    fusion_sync_slop_sec_ = 0.0;
  }

  // ---- Cone Tracker 参数加载 ----
  private_nh_.param<bool>("tracker/enable", config_.tracker.enable, false);
  private_nh_.param<double>("tracker/association_threshold", config_.tracker.association_threshold,
                            0.5);
  private_nh_.param<double>("tracker/association_threshold_far",
                            config_.tracker.association_threshold_far, 1.0);
  private_nh_.param<double>("tracker/association_distance_threshold",
                            config_.tracker.association_distance_threshold, 35.0);
  private_nh_.param<int>("tracker/confirm_frames", config_.tracker.confirm_frames, 3);
  private_nh_.param<int>("tracker/confirm_frames_far", config_.tracker.confirm_frames_far, 2);
  private_nh_.param<double>("tracker/confirm_distance_threshold",
                            config_.tracker.confirm_distance_threshold, 30.0);
  private_nh_.param<int>("tracker/delete_frames", config_.tracker.delete_frames, 5);
  private_nh_.param<int>("tracker/delete_frames_far", config_.tracker.delete_frames_far, 8);
  private_nh_.param<double>("tracker/process_noise", config_.tracker.process_noise, 0.1);
  private_nh_.param<double>("tracker/measurement_noise", config_.tracker.measurement_noise, 0.05);
  private_nh_.param<bool>("tracker/only_output_confirmed", config_.tracker.only_output_confirmed,
                          true);
  private_nh_.param<double>("tracker/confirmed_confidence_boost",
                            config_.tracker.confirmed_confidence_boost, 0.1);

  // ---- Topology Repair 参数加载 ----
  private_nh_.param<bool>("topology/enable", config_.topology.enable, false);
  private_nh_.param<double>("topology/max_same_side_spacing",
                            config_.topology.max_same_side_spacing, 5.0);
  private_nh_.param<double>("topology/min_track_width", config_.topology.min_track_width, 2.5);
  private_nh_.param<double>("topology/max_track_width", config_.topology.max_track_width, 4.0);
  private_nh_.param<double>("topology/max_repair_range", config_.topology.max_repair_range, 15.0);
  private_nh_.param<double>("topology/outlier_lateral_threshold",
                            config_.topology.outlier_lateral_threshold, 5.0);
  private_nh_.param<double>("topology/interpolated_confidence",
                            config_.topology.interpolated_confidence, 0.2);

  // ---- Proximity Deduplication 参数加载 ----
  private_nh_.param<bool>("dedup/enable", config_.dedup.enable, true);
  private_nh_.param<double>("dedup/radius", config_.dedup.radius, 0.5);

  // ---- Stacked Cone Detection 参数加载 ----
  private_nh_.param<bool>("stacked_cone_detection/enabled", config_.dedup.stacked_enable, false);
  private_nh_.param<double>("stacked_cone_detection/vertical_layers/layer_height",
                            config_.dedup.layer_height, 0.25);
  private_nh_.param<int>("stacked_cone_detection/vertical_layers/max_layers",
                         config_.dedup.max_layers, 3);
  private_nh_.param<double>("stacked_cone_detection/identification/max_xy_distance",
                            config_.dedup.stacked_xy_threshold, 0.4);
  private_nh_.param<double>("stacked_cone_detection/identification/height_variation_threshold",
                            config_.dedup.z_height_threshold, 0.3);

  // ---- 距离自适应Y轴ROI参数 ----
  private_nh_.param<bool>("roi/adaptive_y/enable", config_.roi.adaptive_y.enable, false);
  private_nh_.param<double>("roi/adaptive_y/near_y_half", config_.roi.adaptive_y.near_y_half, 5.0);
  private_nh_.param<double>("roi/adaptive_y/far_y_half", config_.roi.adaptive_y.far_y_half, 2.0);
  private_nh_.param<double>("roi/adaptive_y/ramp_start_x", config_.roi.adaptive_y.ramp_start_x,
                            5.0);

  // ---- 远距离中心线排除参数（抑制前方护栏/墙壁导致的中心线假锥）----
  private_nh_.param<bool>("roi/center_exclusion/enable", config_.roi.center_exclusion.enable,
                          false);
  private_nh_.param<double>("roi/center_exclusion/y_half", config_.roi.center_exclusion.y_half,
                            1.0);
  private_nh_.param<double>("roi/center_exclusion/start_distance",
                            config_.roi.center_exclusion.start_distance, 18.0);

  // 根据 roi.mode 覆盖对应赛道预设参数
  applyModePreset();

  ROS_INFO("sensor_height: %f", config_.sensor_height);
  ROS_INFO("sensor_model: %d", config_.sensor_model);
  ROS_INFO("ransac/num_iter: %d", config_.ransac.num_iter);
  ROS_INFO("ransac/num_lpr: %d", config_.ransac.num_lpr);
  ROS_INFO("ransac/th_seeds: %f", config_.ransac.th_seeds);
  ROS_INFO("ransac/th_dist: %f", config_.ransac.th_dist);
  ROS_INFO("ground_method: %s", config_.ground_method.c_str());
  ROS_INFO("roi/mode: %s", config_.roi.mode.c_str());
  ROS_INFO("roi/z_min: %f, roi/z_max: %f", config_.roi.z_min, config_.roi.z_max);
  ROS_INFO("filters/sor: %s (mean_k=%d, stddev_mul=%f)", config_.filters.sor.enable ? "on" : "off",
           config_.filters.sor.mean_k, config_.filters.sor.stddev_mul);
  ROS_INFO("filters/voxel: %s (leaf=%f)", config_.filters.voxel.enable ? "on" : "off",
           config_.filters.voxel.leaf_size);
  ROS_INFO("filters/adaptive_voxel: %s (leaf=%f, density_thr=%d)",
           config_.filters.adaptive_voxel.enable ? "on" : "off",
           config_.filters.adaptive_voxel.leaf_size, config_.filters.adaptive_voxel.density_thr);
  ROS_INFO("filters/distance_adaptive_voxel: %s (near_leaf=%f, far_leaf=%f, dist_thr=%f)",
           config_.filters.distance_adaptive_voxel.enable ? "on" : "off",
           config_.filters.distance_adaptive_voxel.near_leaf,
           config_.filters.distance_adaptive_voxel.far_leaf,
           config_.filters.distance_adaptive_voxel.dist_threshold);
  ROS_INFO("filters/intensity: %s (min=%f)", config_.filters.intensity.enable ? "on" : "off",
           config_.filters.intensity.min_intensity);
  ROS_INFO("filters/use_cropbox: %s", config_.filters.use_cropbox ? "on" : "off");
  ROS_INFO("filters/obstacle_height: %s (grid=%.2f, max_z_span=%.2f, min_pts=%d, min_dist=%.1f)",
           config_.filters.obstacle_height.enable ? "on" : "off",
           config_.filters.obstacle_height.grid_size, config_.filters.obstacle_height.max_z_span,
           config_.filters.obstacle_height.min_points_to_judge,
           config_.filters.obstacle_height.min_distance);
  ROS_INFO("cluster/method: %s", config_.cluster.method.c_str());
  ROS_INFO("ground_method: %s (FGS is the sole supported path)", config_.ground_method.c_str());
  ROS_INFO("ground_watchdog: %s (warn_ms=%.3f, consecutive=%d)",
           ground_watchdog_enable_ ? "on" : "off", ground_watchdog_warn_ms_,
           ground_watchdog_warn_frames_);
  ROS_INFO("cluster/adaptive_size: %s (near: %d-%d, far: %d-%d)",
           config_.cluster.adaptive_size.enable ? "on" : "off",
           config_.cluster.adaptive_size.near_min_size, config_.cluster.adaptive_size.near_max_size,
           config_.cluster.adaptive_size.far_min_size, config_.cluster.adaptive_size.far_max_size);
  ROS_INFO("cluster/fixed_size: min=%d, max=%d", config_.cluster.min_cluster_size,
           config_.cluster.max_cluster_size);
  ROS_INFO("cluster/fec: %s (quality=%.2f)", config_.cluster.fec.enable ? "on" : "off",
           config_.cluster.fec.quality);
  ROS_INFO("cluster/multi_frame: %s (num_frames=%d, max_distance=%.1f)",
           config_.cluster.multi_frame.enable ? "on" : "off",
           config_.cluster.multi_frame.num_frames, config_.cluster.multi_frame.max_distance);
  ROS_INFO(
      "confidence: weights(size=%.2f, shape=%.2f, density=%.2f, intensity=%.2f, position=%.2f)",
      config_.confidence.weight_size, config_.confidence.weight_shape,
      config_.confidence.weight_density, config_.confidence.weight_intensity,
      config_.confidence.weight_position);
  ROS_INFO("confidence: ramp(near=%.2f, far=%.2f, start=%.1fm, end=%.1fm)",
           config_.confidence.min_confidence_near, config_.confidence.min_confidence_far,
           config_.confidence.confidence_ramp_start, config_.confidence.confidence_ramp_end);
  ROS_INFO("roi/adaptive_y: %s (near_y_half=%.1f, far_y_half=%.1f, ramp_start_x=%.1f)",
           config_.roi.adaptive_y.enable ? "on" : "off", config_.roi.adaptive_y.near_y_half,
           config_.roi.adaptive_y.far_y_half, config_.roi.adaptive_y.ramp_start_x);
  ROS_INFO("roi/center_exclusion: %s (y_half=%.2f, start_distance=%.1fm)",
           config_.roi.center_exclusion.enable ? "on" : "off", config_.roi.center_exclusion.y_half,
           config_.roi.center_exclusion.start_distance);
  ROS_INFO("cone_size_typing: %s (big_height_thr=%.2f, big_area_thr=%.3f, position_coloring=%s)",
           enable_cone_size_typing_ ? "on" : "off", big_cone_height_threshold_,
           big_cone_area_threshold_, enable_position_coloring_ ? "on" : "off");
  if (config_.cluster.method == "dbscan") {
    ROS_INFO("cluster/dbscan: eps=%f, min_pts=%d, adaptive_eps=%s (near=%f, far=%f)",
             config_.cluster.dbscan.eps, config_.cluster.dbscan.min_pts,
             config_.cluster.dbscan.adaptive_eps ? "on" : "off", config_.cluster.dbscan.eps_near,
             config_.cluster.dbscan.eps_far);
  }

  if (!private_nh_.param<bool>("perf_stats_enable", perf_enabled_, true)) {
    ROS_DEBUG_STREAM("Did not load perf_stats_enable. Standard value is: " << perf_enabled_);
  }
  int perf_window = static_cast<int>(perf_window_);
  if (!private_nh_.param<int>("perf_stats_window", perf_window, 300)) {
    ROS_DEBUG_STREAM("Did not load perf_stats_window. Standard value is: " << perf_window);
  }
  perf_window_ = static_cast<size_t>(perf_window);
  int perf_log_every = static_cast<int>(perf_log_every_);
  if (!private_nh_.param<int>("perf_stats_log_every", perf_log_every, 30)) {
    ROS_DEBUG_STREAM("Did not load perf_stats_log_every. Standard value is: " << perf_log_every);
  }
  perf_log_every_ = static_cast<size_t>(perf_log_every);
  perf_stats_.Configure("lidar_cluster", perf_enabled_, perf_window_, perf_log_every_);

  sensor_model_ = config_.sensor_model;
  ROS_INFO_STREAM("pipeline_mode: " << pipeline_mode_ << ", legacy_poll_hz: " << legacy_poll_hz_);
}

void LidarClusterRos::applyModePreset() {
  std::string mode = config_.roi.mode;
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  std::string prefix = "mode_presets/" + mode;

  if (!private_nh_.hasParam(prefix)) {
    // mode_presets 是可选项：缺失时直接使用文件内顶层参数
    ROS_INFO("No mode_preset found for '%s', using top-level parameters", mode.c_str());
    return;
  }

  ROS_INFO("Applying mode preset: %s", mode.c_str());

  // 覆盖 confidence ramp 参数
  private_nh_.param<double>(prefix + "/confidence/min_confidence_near",
                            config_.confidence.min_confidence_near,
                            config_.confidence.min_confidence_near);
  private_nh_.param<double>(prefix + "/confidence/min_confidence_far",
                            config_.confidence.min_confidence_far,
                            config_.confidence.min_confidence_far);
  private_nh_.param<double>(prefix + "/confidence/confidence_ramp_start",
                            config_.confidence.confidence_ramp_start,
                            config_.confidence.confidence_ramp_start);
  private_nh_.param<double>(prefix + "/confidence/confidence_ramp_end",
                            config_.confidence.confidence_ramp_end,
                            config_.confidence.confidence_ramp_end);

  // 覆盖 adaptive_y 参数
  private_nh_.param<bool>(prefix + "/adaptive_y/enable", config_.roi.adaptive_y.enable,
                          config_.roi.adaptive_y.enable);
  if (config_.roi.adaptive_y.enable) {
    private_nh_.param<double>(prefix + "/adaptive_y/near_y_half",
                              config_.roi.adaptive_y.near_y_half,
                              config_.roi.adaptive_y.near_y_half);
    private_nh_.param<double>(prefix + "/adaptive_y/far_y_half", config_.roi.adaptive_y.far_y_half,
                              config_.roi.adaptive_y.far_y_half);
    private_nh_.param<double>(prefix + "/adaptive_y/ramp_start_x",
                              config_.roi.adaptive_y.ramp_start_x,
                              config_.roi.adaptive_y.ramp_start_x);
  }

  // 覆盖 center_exclusion 参数
  private_nh_.param<bool>(prefix + "/center_exclusion/enable", config_.roi.center_exclusion.enable,
                          config_.roi.center_exclusion.enable);
  if (config_.roi.center_exclusion.enable) {
    private_nh_.param<double>(prefix + "/center_exclusion/y_half",
                              config_.roi.center_exclusion.y_half,
                              config_.roi.center_exclusion.y_half);
    private_nh_.param<double>(prefix + "/center_exclusion/start_distance",
                              config_.roi.center_exclusion.start_distance,
                              config_.roi.center_exclusion.start_distance);
  }

  // 覆盖 track_semantic 参数
  private_nh_.param<bool>(prefix + "/confidence/track_semantic/enable",
                          config_.confidence.track_semantic.enable,
                          config_.confidence.track_semantic.enable);
  private_nh_.param<double>(prefix + "/confidence/track_semantic/weight",
                            config_.confidence.track_semantic.weight,
                            config_.confidence.track_semantic.weight);
  private_nh_.param<double>(prefix + "/confidence/track_semantic/expected_track_width",
                            config_.confidence.track_semantic.expected_track_width,
                            config_.confidence.track_semantic.expected_track_width);
  private_nh_.param<double>(prefix + "/confidence/track_semantic/expected_cone_spacing",
                            config_.confidence.track_semantic.expected_cone_spacing,
                            config_.confidence.track_semantic.expected_cone_spacing);

  // 覆盖 cluster 分段参数
  std::vector<double> segments, tolerances;
  if (LoadDoubleVector(private_nh_, prefix + "/cluster/distance_segments", segments)) {
    config_.cluster.distance_segments = segments;
  }
  if (LoadDoubleVector(private_nh_, prefix + "/cluster/cluster_tolerance", tolerances)) {
    config_.cluster.cluster_tolerance = tolerances;
  }

  // G13: 覆盖模块开关（tracker / topology / obstacle_height）
  private_nh_.param<bool>(prefix + "/tracker/enable", config_.tracker.enable,
                          config_.tracker.enable);
  private_nh_.param<bool>(prefix + "/topology/enable", config_.topology.enable,
                          config_.topology.enable);
  private_nh_.param<bool>(prefix + "/filters/obstacle_height/enable",
                          config_.filters.obstacle_height.enable,
                          config_.filters.obstacle_height.enable);

  ROS_INFO("Mode preset '%s' applied: tracker=%s, topology=%s, obstacle_height=%s", mode.c_str(),
           config_.tracker.enable ? "on" : "off", config_.topology.enable ? "on" : "off",
           config_.filters.obstacle_height.enable ? "on" : "off");
}

void LidarClusterRos::pointCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  input_guard_frames_seen_.fetch_add(1, std::memory_order_relaxed);
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

  // Apply distortion compensation V2 (自动检测time字段)
  if (compensator_ && compensator_->IsEnabled()) {
    // 使用CompensateFromMsg自动检测点云类型并补偿
    if (!compensator_->CompensateFromMsg(*msg, cloud)) {
      // 补偿失败，回退到普通转换
      pcl::fromROSMsg(*msg, *cloud);
    }
  } else {
    pcl::fromROSMsg(*msg, *cloud);
  }

  // G4: 输入边界防御
  if (input_guard_enable_) {
    if (cloud->empty()) {
      input_guard_drop_empty_.fetch_add(1, std::memory_order_relaxed);
      ROS_WARN_THROTTLE(2.0, "Received empty point cloud, dropping frame");
      return;
    }

    if (cloud->size() > static_cast<size_t>(input_guard_max_points_)) {
      input_guard_drop_oversize_.fetch_add(1, std::memory_order_relaxed);
      ROS_WARN_THROTTLE(2.0, "Point cloud too large (%zu > %d), dropping frame", cloud->size(),
                        input_guard_max_points_);
      return;
    }

    if (input_guard_filter_invalid_points_) {
      const size_t before_size = cloud->points.size();
      cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
                                         [](const PointType& p) {
                                           return !std::isfinite(p.x) || !std::isfinite(p.y) ||
                                                  !std::isfinite(p.z);
                                         }),
                          cloud->points.end());
      const size_t invalid_count = before_size - cloud->points.size();
      if (invalid_count > 0) {
        input_guard_filtered_frames_.fetch_add(1, std::memory_order_relaxed);
        input_guard_filtered_points_total_.fetch_add(invalid_count, std::memory_order_relaxed);
        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;
        ROS_WARN_THROTTLE(2.0, "Filtered %zu invalid points (NaN/Inf) from cloud", invalid_count);
      }
      if (cloud->empty()) {
        input_guard_drop_all_invalid_.fetch_add(1, std::memory_order_relaxed);
        ROS_WARN_THROTTLE(2.0, "Point cloud contains only invalid points, dropping frame");
        return;
      }
    }
  }

  std::lock_guard<std::mutex> lock(seq_mutex_);
  last_header_ = msg->header;
  validateStamp(last_header_);
  last_seq_ = msg->header.seq;
  got_cloud_ = true;
  // 零拷贝：转移点云所有权给core，避免深拷贝
  core_.SetInputCloud(std::move(cloud), msg->header.seq);

  // 事件驱动路径：收到新帧立即处理
  if (!IsLegacyPollMode()) {
    runOnceLocked();
  }
}

void LidarClusterRos::RunOnce() {
  std::lock_guard<std::mutex> lock(seq_mutex_);
  runOnceLocked();
}

void LidarClusterRos::runOnceLocked() {
  if (!got_cloud_) {
    ROS_WARN_THROTTLE(5.0, "Not received Point Cloud!");
    return;
  }

  // 新帧闸门 - 仅处理新帧
  if (last_seq_ == last_processed_seq_) {
    return;  // 已处理过该帧，跳过
  }

  // 超时检查 - 丢弃过旧的点云
  // 使用帧间隔判断：如果当前帧时间戳比上一帧还旧，说明数据有问题
  // 这样可以同时支持实时运行和 rosbag 回放
  if (!last_cloud_stamp_.isZero() && last_header_.stamp < last_cloud_stamp_) {
    // 时间戳回退，可能是 rosbag 重新开始，重置状态
    ROS_INFO("Timestamp reset detected, reinitializing...");
    last_cloud_stamp_ = last_header_.stamp;
  }

  // 计算帧间隔（与上一处理帧的时间差）
  double frame_interval =
      last_cloud_stamp_.isZero() ? 0.0 : (last_header_.stamp - last_cloud_stamp_).toSec();
  if (frame_interval > max_cloud_age_ && frame_interval < 10.0)  // 10秒内的异常间隔
  {
    ROS_WARN_THROTTLE(1.0, "Large frame interval (%.2fs), possible frame drop", frame_interval);
  }

  last_cloud_stamp_ = last_header_.stamp;

  // G10: Extract ego-motion from IMU compensator for tracker prediction
  {
    perception::EgoMotion ego;
    double vx = 0.0, vy = 0.0, yaw_rate = 0.0;
    if (compensator_ && compensator_->GetLatestEgoVelocity(vx, vy, yaw_rate)) {
      const double dt = frame_interval > 0.0 ? frame_interval : 0.1;
      ego.dx = vx * dt;
      ego.dy = vy * dt;
      ego.dyaw = yaw_rate * dt;
    }
    core_.SetEgoMotion(ego);
  }

  if (!core_.Process(&output_)) {
    return;
  }

  updateGroundWatchdogLocked(output_.t_ground_ms);
  last_processed_seq_ = last_seq_;
  publishOutput(output_);
}

void LidarClusterRos::updateGroundWatchdogLocked(double t_ground_ms) {
  if (!ground_watchdog_enable_) {
    return;
  }

  if (t_ground_ms > ground_watchdog_warn_ms_) {
    ++ground_watchdog_overrun_count_;
  } else {
    ground_watchdog_overrun_count_ = 0;
    return;
  }

  if (ground_watchdog_overrun_count_ < ground_watchdog_warn_frames_) {
    return;
  }

  ROS_WARN_THROTTLE(
      1.0, "Ground watchdog triggered: t_ground_ms=%.3f exceeds %.3f for %d consecutive frames",
      t_ground_ms, ground_watchdog_warn_ms_, ground_watchdog_warn_frames_);
  ground_watchdog_overrun_count_ = 0;
}

void LidarClusterRos::publishOutput(const LidarClusterOutput& output) {
  size_t bytes_pub = 0;

  // 复用消息缓冲区，避免每帧重新分配
  if (output.passthrough) {
    pcl::toROSMsg(*output.passthrough, pub_pc_msg_);
    pub_pc_msg_.header = last_header_;
    passthrough_pub_.publish(pub_pc_msg_);
    bytes_pub += pub_pc_msg_.data.size();
  }

  if (output.not_ground) {
    pcl::toROSMsg(*output.not_ground, pub_pc_msg_);
    pub_pc_msg_.header = last_header_;
    no_ground_pub_.publish(pub_pc_msg_);
    bytes_pub += pub_pc_msg_.data.size();
  }

  if (output.cones_cloud) {
    pcl::toROSMsg(*output.cones_cloud, pub_cones_msg_);
    pub_cones_msg_.header = last_header_;
    cones_pub_.publish(pub_cones_msg_);
    bytes_pub += pub_cones_msg_.data.size();
  }

  autodrive_msgs::HUAT_ConeDetections detections;
  detections.header = last_header_;
  if (output.cones_cloud) {
    detections.pc_whole = pub_cones_msg_;
  }

  // G11: 按角度排序，保证帧间输出顺序稳定
  const auto& cones = output.cones;
  std::vector<size_t> order(cones.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&cones](size_t a, size_t b) {
    return std::atan2(cones[a].centroid.y, cones[a].centroid.x) <
           std::atan2(cones[b].centroid.y, cones[b].centroid.x);
  });

  for (const size_t idx : order) {
    const auto& det = cones[idx];
    // Defense-in-depth: skip detections with non-finite centroids
    if (!std::isfinite(det.centroid.x) || !std::isfinite(det.centroid.y) ||
        !std::isfinite(det.centroid.z)) {
      continue;
    }

    geometry_msgs::Point32 center;
    center.x = det.centroid.x;
    center.y = det.centroid.y;
    center.z = det.centroid.z;
    detections.points.push_back(center);

    geometry_msgs::Point32 max;
    max.x = det.max.x;
    max.y = det.max.y;
    max.z = det.max.z;
    detections.maxPoints.push_back(max);

    geometry_msgs::Point32 min;
    min.x = det.min.x;
    min.y = det.min.y;
    min.z = det.min.z;
    detections.minPoints.push_back(min);

    // 始终填充 confidence，保持数组长度一致
    if (sensor_model_ != 16) {
      detections.confidence.push_back(static_cast<float>(det.confidence));
    } else {
      detections.confidence.push_back(0.0f);  // VLP-16 填充默认值
    }
    detections.obj_dist.push_back(static_cast<float>(det.distance));
    detections.track_ids.push_back(static_cast<int32_t>(det.track_id));

    // When fusion is disabled or no vision available, use position-based coloring
    const bool use_position_coloring = enable_position_coloring_ && !fusion_enabled_;
    uint8_t geo_color =
        classifyConeTypeBySize(det, enable_cone_size_typing_, big_cone_height_threshold_,
                               big_cone_area_threshold_, use_position_coloring);
    detections.color_types.push_back(geo_color);

    if (det.cluster) {
      sensor_msgs::PointCloud2 cluster_msg;
      pcl::toROSMsg(*det.cluster, cluster_msg);
      cluster_msg.header = last_header_;
      detections.pc.push_back(cluster_msg);
    }
  }

  detections_pub_.publish(detections);
  bytes_pub += ros::serialization::serializationLength(detections);

  // Fused topic is published independently after raw detections are cached.
  if (fusion_enabled_) {
    pushRawFrame(detections);
    tryPublishFusedForStamp(detections.header.stamp);
  }

  // G3: Array length invariant — all parallel arrays must match
  {
    const size_t n = detections.points.size();
    if (detections.maxPoints.size() != n || detections.minPoints.size() != n ||
        detections.confidence.size() != n || detections.obj_dist.size() != n ||
        detections.track_ids.size() != n || detections.color_types.size() != n) {
      ROS_ERROR_THROTTLE(1.0,
                         "ConeDetections array length mismatch: points=%zu max=%zu min=%zu "
                         "conf=%zu dist=%zu color=%zu",
                         n, detections.maxPoints.size(), detections.minPoints.size(),
                         detections.confidence.size(), detections.obj_dist.size(),
                         detections.color_types.size());
    }
  }

  const int n_published = static_cast<int>(detections.points.size());

  // Accumulate per-stage pipeline statistics for diagnostics
  stage_input_points_total_.fetch_add(output.input_points, std::memory_order_relaxed);
  stage_roi_points_total_.fetch_add(output.roi_points, std::memory_order_relaxed);
  stage_roi_dropped_total_.fetch_add(output.roi_dropped, std::memory_order_relaxed);
  stage_intensity_dropped_total_.fetch_add(output.intensity_dropped, std::memory_order_relaxed);
  stage_ground_removed_total_.fetch_add(output.ground_removed, std::memory_order_relaxed);
  stage_obstacle_dropped_total_.fetch_add(output.obstacle_dropped, std::memory_order_relaxed);
  stage_clusters_total_total_.fetch_add(output.total_clusters, std::memory_order_relaxed);
  stage_clusters_far_total_.fetch_add(output.clusters_far, std::memory_order_relaxed);

  updateHealthState(n_published, output.t_total_ms);
  publishDiagnostics(output, n_published);
  publishDebugMarkers(output);

  PerfSample sample;
  sample.t_pass_ms = output.t_pass_ms;
  sample.t_ground_ms = output.t_ground_ms;
  sample.t_cluster_ms = output.t_cluster_ms;
  sample.t_total_ms = output.t_total_ms;
  sample.n_points = static_cast<double>(output.input_points);
  sample.n_clusters = static_cast<double>(output.total_clusters);
  sample.n_detections = static_cast<double>(n_published);
  sample.bytes_pub = static_cast<double>(bytes_pub);
  perf_stats_.Add(sample);
}

void LidarClusterRos::validateStamp(std_msgs::Header& header) {
  // 0) frame_id 检查
  if (header.frame_id.empty()) {
    ROS_WARN_ONCE(
        "Point cloud frame_id is empty. "
        "Check LiDAR driver configuration.");
  } else if (header.frame_id != fsd_common::frame_contract::kVelodyne) {
    ROS_WARN_ONCE(
        "Point cloud frame_id is '%s', expected '%s'. "
        "Verify sensor frame configuration.",
        header.frame_id.c_str(), fsd_common::frame_contract::kVelodyne);
  }

  // 1) 零时间戳 → 降级为接收时间
  if (header.stamp.isZero()) {
    ROS_WARN_ONCE(
        "Point cloud stamp is zero, falling back to ros::Time::now(). "
        "Check LiDAR driver timestamp configuration.");
    header.stamp = ros::Time::now();
    return;
  }

  // 2) 非单调递增（排除 rosbag restart，那个在 runOnceLocked 中处理）
  if (!last_cloud_stamp_.isZero() && header.stamp < last_cloud_stamp_) {
    // 小幅回退（< 1s）不是 rosbag restart，是驱动异常
    const double rollback = (last_cloud_stamp_ - header.stamp).toSec();
    if (rollback < 1.0) {
      ROS_WARN_THROTTLE(2.0,
                        "Point cloud stamp went backwards by %.3fs (seq=%u). "
                        "Possible driver timestamp jitter.",
                        rollback, header.seq);
    }
  }

  // 3) 与 wall clock 偏差检查（仅在非 sim_time 模式下有意义）
  if (!ros::Time::isSimTime()) {
    const double drift = std::abs((ros::Time::now() - header.stamp).toSec());
    if (drift > stamp_max_drift_sec_) {
      ROS_WARN_ONCE(
          "Point cloud stamp drifts %.2fs from wall clock (threshold=%.1fs). "
          "Check PPS/NTP sync on LiDAR driver.",
          drift, stamp_max_drift_sec_);
    }
  }
}

void LidarClusterRos::updateHealthState(int n_detections, double t_total_ms) {
  const bool is_zero = (n_detections == 0);
  const bool is_low = (!is_zero && n_detections <= diag_low_detection_threshold_);
  const bool is_normal = (n_detections > diag_low_detection_threshold_);

  // ── 正常帧计数（用于恢复） ──
  if (is_normal && t_total_ms <= diag_latency_warn_ms_) {
    ++consecutive_normal_frames_;
  } else {
    consecutive_normal_frames_ = 0;
  }

  // ── 异常帧计数 ──
  if (is_zero) {
    ++consecutive_zero_detections_;
    consecutive_low_detections_ = 0;
  } else if (is_low) {
    consecutive_zero_detections_ = 0;
    ++consecutive_low_detections_;
  } else {
    consecutive_zero_detections_ = 0;
    consecutive_low_detections_ = 0;
  }

  // ── 状态转移 ──
  HealthLevel prev = health_level_;

  if (consecutive_zero_detections_ >= diag_zero_frames_to_error_) {
    health_level_ = HealthLevel::NO_DETECTION;
  } else if (consecutive_zero_detections_ >= diag_zero_frames_to_warn_ ||
             consecutive_low_detections_ >= diag_low_frames_to_warn_) {
    health_level_ = HealthLevel::LOW_DETECTION;
  }

  // 恢复：连续 N 帧正常才回到 NORMAL
  if (health_level_ != HealthLevel::NORMAL && consecutive_normal_frames_ >= diag_recovery_frames_) {
    health_level_ = HealthLevel::NORMAL;
  }

  if (health_level_ != prev) {
    const char* names[] = {"NORMAL", "LOW_DETECTION", "NO_DETECTION"};
    ROS_WARN("Perception health: %s -> %s (zero=%d, low=%d, normal=%d)",
             names[static_cast<int>(prev)], names[static_cast<int>(health_level_)],
             consecutive_zero_detections_, consecutive_low_detections_, consecutive_normal_frames_);
  }
}

void LidarClusterRos::publishDiagnostics(const LidarClusterOutput& output, int n_published) {
  using DH = fsd_common::DiagnosticsHelper;

  uint8_t level = static_cast<uint8_t>(health_level_);
  const char* messages[] = {"OK", "Low detection count", "No detections"};
  std::string msg = messages[level];

  if (output.t_total_ms > diag_latency_warn_ms_ && health_level_ == HealthLevel::NORMAL) {
    level = 1;  // WARN
    msg = "High latency";
  }

  std::vector<diagnostic_msgs::KeyValue> kvs;
  kvs.reserve(38);
  // B12: Perception quality statistics
  // Detection counts
  kvs.push_back(DH::KV("n_detections", std::to_string(n_published)));
  kvs.push_back(DH::KV("n_input_points", std::to_string(output.input_points)));
  kvs.push_back(DH::KV("n_clusters", std::to_string(output.total_clusters)));
  // Performance metrics
  kvs.push_back(DH::KV("t_total_ms", std::to_string(output.t_total_ms)));
  kvs.push_back(DH::KV("t_ground_ms", std::to_string(output.t_ground_ms)));
  kvs.push_back(DH::KV("t_cluster_ms", std::to_string(output.t_cluster_ms)));
  kvs.push_back(DH::KV("t_pass_ms", std::to_string(output.t_pass_ms)));
  // Health status
  kvs.push_back(DH::KV("health", messages[static_cast<int>(health_level_)]));
  kvs.push_back(DH::KV("consecutive_zero", std::to_string(consecutive_zero_detections_)));
  kvs.push_back(DH::KV("ground_method", config_.ground_method));
  {
    const uint64_t seen = input_guard_frames_seen_.load(std::memory_order_relaxed);
    const uint64_t drop_empty = input_guard_drop_empty_.load(std::memory_order_relaxed);
    const uint64_t drop_oversize = input_guard_drop_oversize_.load(std::memory_order_relaxed);
    const uint64_t drop_all_invalid = input_guard_drop_all_invalid_.load(std::memory_order_relaxed);
    const uint64_t filtered_frames = input_guard_filtered_frames_.load(std::memory_order_relaxed);
    const uint64_t filtered_points =
        input_guard_filtered_points_total_.load(std::memory_order_relaxed);
    const uint64_t drop_total = drop_empty + drop_oversize + drop_all_invalid;
    const double drop_ratio =
        seen > 0 ? static_cast<double>(drop_total) / static_cast<double>(seen) : 0.0;

    kvs.push_back(DH::KV("input_guard_enable", input_guard_enable_ ? "1" : "0"));
    kvs.push_back(DH::KV("input_guard_max_points", std::to_string(input_guard_max_points_)));
    kvs.push_back(
        DH::KV("input_guard_filter_invalid", input_guard_filter_invalid_points_ ? "1" : "0"));
    kvs.push_back(DH::KV("input_guard_frames_seen", std::to_string(seen)));
    kvs.push_back(DH::KV("input_guard_drop_empty", std::to_string(drop_empty)));
    kvs.push_back(DH::KV("input_guard_drop_oversize", std::to_string(drop_oversize)));
    kvs.push_back(DH::KV("input_guard_drop_all_invalid", std::to_string(drop_all_invalid)));
    kvs.push_back(DH::KV("input_guard_drop_total", std::to_string(drop_total)));
    kvs.push_back(DH::KV("input_guard_filtered_frames", std::to_string(filtered_frames)));
    kvs.push_back(DH::KV("input_guard_filtered_points_total", std::to_string(filtered_points)));
    kvs.push_back(DH::KV("input_guard_drop_ratio", std::to_string(drop_ratio)));
  }

  // Per-stage pipeline statistics (per-frame values from current output)
  {
    kvs.push_back(DH::KV("stage_input_points", std::to_string(output.input_points)));
    kvs.push_back(DH::KV("stage_roi_points", std::to_string(output.roi_points)));
    kvs.push_back(DH::KV("stage_roi_dropped", std::to_string(output.roi_dropped)));
    kvs.push_back(DH::KV("stage_intensity_dropped", std::to_string(output.intensity_dropped)));
    kvs.push_back(DH::KV("stage_ground_removed", std::to_string(output.ground_removed)));
    kvs.push_back(DH::KV("stage_obstacle_dropped", std::to_string(output.obstacle_dropped)));
    kvs.push_back(DH::KV("stage_clusters_total", std::to_string(output.total_clusters)));
    kvs.push_back(DH::KV("stage_clusters_far", std::to_string(output.clusters_far)));
  }

  // B12: Per-distance confidence segmentation (quality indicator)
  // Segments: near (<ramp_start), mid (ramp_start~ramp_end), far (>ramp_end)
  // Tracks detection count and average confidence per segment
  {
    int n_near = 0, n_mid = 0, n_far = 0;
    double sum_conf_near = 0.0, sum_conf_mid = 0.0, sum_conf_far = 0.0;
    const double ramp_start = config_.confidence.confidence_ramp_start;
    const double ramp_end = config_.confidence.confidence_ramp_end;
    for (const auto& cone : output.cones) {
      if (cone.distance < ramp_start) {
        ++n_near;
        sum_conf_near += cone.confidence;
      } else if (cone.distance < ramp_end) {
        ++n_mid;
        sum_conf_mid += cone.confidence;
      } else {
        ++n_far;
        sum_conf_far += cone.confidence;
      }
    }
    kvs.push_back(DH::KV("n_near", std::to_string(n_near)));
    kvs.push_back(DH::KV("n_mid", std::to_string(n_mid)));
    kvs.push_back(DH::KV("n_far", std::to_string(n_far)));
    kvs.push_back(DH::KV("conf_near", n_near > 0 ? std::to_string(sum_conf_near / n_near) : "0"));
    kvs.push_back(DH::KV("conf_mid", n_mid > 0 ? std::to_string(sum_conf_mid / n_mid) : "0"));
    kvs.push_back(DH::KV("conf_far", n_far > 0 ? std::to_string(sum_conf_far / n_far) : "0"));
  }

  // B12: Rolling-window performance statistics (quality indicator)
  // Provides p50/p95/max percentiles for latency and mean for throughput
  // Helps identify performance degradation and outliers
  {
    auto snap = perf_stats_.SnapshotStats();
    kvs.push_back(DH::KV("perf_total_p50", std::to_string(snap.t_total_ms.p50)));
    kvs.push_back(DH::KV("perf_total_p95", std::to_string(snap.t_total_ms.p95)));
    kvs.push_back(DH::KV("perf_total_max", std::to_string(snap.t_total_ms.max)));
    kvs.push_back(DH::KV("perf_ground_p95", std::to_string(snap.t_ground_ms.p95)));
    kvs.push_back(DH::KV("perf_cluster_p95", std::to_string(snap.t_cluster_ms.p95)));
    kvs.push_back(DH::KV("perf_points_mean", std::to_string(snap.n_points.mean)));
    kvs.push_back(DH::KV("perf_detections_mean", std::to_string(snap.n_detections.mean)));
    kvs.push_back(DH::KV("perf_bytes_mean", std::to_string(snap.bytes_pub.mean)));
  }

  // Fusion observability
  kvs.push_back(DH::KV("fusion_enabled", fusion_enabled_ ? "1" : "0"));
  if (fusion_enabled_) {
    kvs.push_back(DH::KV("fusion_sync_policy", fusion_sync_policy_));
    kvs.push_back(DH::KV("fusion_sync_queue_size", std::to_string(fusion_sync_queue_size_)));
    kvs.push_back(DH::KV("fusion_sync_slop_sec", std::to_string(fusion_sync_slop_sec_)));
    kvs.push_back(DH::KV("fusion_pair_total",
                         std::to_string(fusion_sync_pair_total_.load(std::memory_order_relaxed))));
    kvs.push_back(DH::KV("fusion_pair_used",
                         std::to_string(fusion_sync_pair_used_.load(std::memory_order_relaxed))));
    kvs.push_back(
        DH::KV("fusion_pair_dropped_no_raw",
               std::to_string(fusion_sync_pair_dropped_no_raw_.load(std::memory_order_relaxed))));
    kvs.push_back(
        DH::KV("fusion_pair_cache_evicted",
               std::to_string(fusion_sync_pair_cache_evicted_.load(std::memory_order_relaxed))));
    kvs.push_back(DH::KV("fusion_raw_cache_evicted", std::to_string(fusion_raw_cache_evicted_.load(
                                                         std::memory_order_relaxed))));
    kvs.push_back(
        DH::KV("fusion_messages_published",
               std::to_string(fusion_messages_published_.load(std::memory_order_relaxed))));
    kvs.push_back(DH::KV("fusion_frames_no_sync",
                         std::to_string(fusion_frames_no_sync_.load(std::memory_order_relaxed))));
    kvs.push_back(
        DH::KV("fusion_frames_camera_missing",
               std::to_string(fusion_frames_camera_missing_.load(std::memory_order_relaxed))));
    kvs.push_back(DH::KV("fusion_frames_tf_missing", std::to_string(fusion_frames_tf_missing_.load(
                                                         std::memory_order_relaxed))));
    kvs.push_back(
        DH::KV("fusion_assoc_matched_total",
               std::to_string(fusion_association_matched_total_.load(std::memory_order_relaxed))));
    kvs.push_back(DH::KV(
        "fusion_assoc_unmatched_total",
        std::to_string(fusion_association_unmatched_total_.load(std::memory_order_relaxed))));
    kvs.push_back(
        DH::KV("fusion_last_sync_delta_ms", std::to_string(fusion_last_sync_delta_sec_ * 1000.0)));
    kvs.push_back(DH::KV("fusion_last_matched", std::to_string(fusion_last_matched_count_)));
    kvs.push_back(DH::KV("fusion_last_unmatched", std::to_string(fusion_last_unmatched_count_)));
    kvs.push_back(
        DH::KV("fusion_camera_info_ready", fusion_last_camera_info_available_ ? "1" : "0"));
    kvs.push_back(DH::KV("fusion_tf_available", fusion_last_tf_available_ ? "1" : "0"));
  }

  // Confidence sub-component averages and rejection reason tallies
  {
    kvs.push_back(DH::KV("rejected_by_roi", std::to_string(output.rejected_by_roi)));
    kvs.push_back(DH::KV("rejected_by_confidence", std::to_string(output.rejected_by_confidence)));
    kvs.push_back(DH::KV("rejected_by_semantic", std::to_string(output.rejected_by_semantic)));
    kvs.push_back(DH::KV("rejected_by_tracker", std::to_string(output.rejected_by_tracker)));
    int scored = output.scored_count;
    kvs.push_back(DH::KV("scored_count", std::to_string(scored)));
    kvs.push_back(DH::KV("avg_size_score",
                         scored > 0 ? std::to_string(output.sum_size_score / scored) : "0"));
    kvs.push_back(DH::KV("avg_shape_score",
                         scored > 0 ? std::to_string(output.sum_shape_score / scored) : "0"));
    kvs.push_back(DH::KV("avg_density_score",
                         scored > 0 ? std::to_string(output.sum_density_score / scored) : "0"));
    kvs.push_back(DH::KV("avg_intensity_score",
                         scored > 0 ? std::to_string(output.sum_intensity_score / scored) : "0"));
    kvs.push_back(DH::KV("avg_position_score",
                         scored > 0 ? std::to_string(output.sum_position_score / scored) : "0"));
    int semantic_scored = output.scored_count;  // semantic scored on same set
    kvs.push_back(DH::KV(
        "avg_semantic_score",
        semantic_scored > 0 ? std::to_string(output.sum_semantic_score / semantic_scored) : "0"));
    kvs.push_back(DH::KV("semantic_kills", std::to_string(output.semantic_kills)));
    // Task 23C: far-range (20-50m) near-threshold diagnostics
    {
      kvs.push_back(DH::KV("far_candidates_total", std::to_string(output.far_candidates_total)));
      kvs.push_back(DH::KV("far_accepted", std::to_string(output.far_accepted)));
      kvs.push_back(
          DH::KV("far_rejected_by_confidence", std::to_string(output.far_rejected_by_confidence)));
      kvs.push_back(DH::KV("far_conf_lt_025", std::to_string(output.far_conf_lt_025)));
      kvs.push_back(DH::KV("far_conf_025_035", std::to_string(output.far_conf_025_035)));
      kvs.push_back(DH::KV("far_conf_035_040", std::to_string(output.far_conf_035_040)));
      kvs.push_back(DH::KV("far_conf_040_045", std::to_string(output.far_conf_040_045)));
      kvs.push_back(DH::KV("far_conf_045_050", std::to_string(output.far_conf_045_050)));
      kvs.push_back(DH::KV("far_conf_gt_050", std::to_string(output.far_conf_gt_050)));
      kvs.push_back(
          DH::KV("far_avg_conf_rejected", output.far_rejected_count_for_avg > 0
                                              ? std::to_string(output.far_sum_confidence_rejected /
                                                               output.far_rejected_count_for_avg)
                                              : "0"));
    }
    // Model fitting stats
    kvs.push_back(DH::KV("fitting_calls_total", std::to_string(output.fitting_calls_total)));
    kvs.push_back(DH::KV("fitting_skipped", std::to_string(output.fitting_skipped)));
    kvs.push_back(DH::KV("fitting_success", std::to_string(output.fitting_success)));
    kvs.push_back(DH::KV("fitting_fail", std::to_string(output.fitting_fail)));
    // Tracker stats
    kvs.push_back(DH::KV("tracker_tentative", std::to_string(output.tracker_tentative)));
    kvs.push_back(DH::KV("tracker_confirmed", std::to_string(output.tracker_confirmed)));
    kvs.push_back(DH::KV("tracker_deleted", std::to_string(output.tracker_deleted)));
    // Task 23D: post-confidence publication funnel
    {
      kvs.push_back(DH::KV("postconf_20_30", std::to_string(output.postconf_20_30)));
      kvs.push_back(DH::KV("postconf_30_40", std::to_string(output.postconf_30_40)));
      kvs.push_back(DH::KV("postconf_40_50", std::to_string(output.postconf_40_50)));
      kvs.push_back(DH::KV("postconf_50_60", std::to_string(output.postconf_50_60)));
      kvs.push_back(DH::KV("postconf_60_80", std::to_string(output.postconf_60_80)));
      kvs.push_back(DH::KV("after_dedup_20_30", std::to_string(output.after_dedup_20_30)));
      kvs.push_back(DH::KV("after_dedup_30_40", std::to_string(output.after_dedup_30_40)));
      kvs.push_back(DH::KV("after_dedup_40_50", std::to_string(output.after_dedup_40_50)));
      kvs.push_back(DH::KV("after_dedup_50_60", std::to_string(output.after_dedup_50_60)));
      kvs.push_back(DH::KV("after_dedup_60_80", std::to_string(output.after_dedup_60_80)));
      kvs.push_back(DH::KV("after_tracker_20_30", std::to_string(output.after_tracker_20_30)));
      kvs.push_back(DH::KV("after_tracker_30_40", std::to_string(output.after_tracker_30_40)));
      kvs.push_back(DH::KV("after_tracker_40_50", std::to_string(output.after_tracker_40_50)));
      kvs.push_back(DH::KV("after_tracker_50_60", std::to_string(output.after_tracker_50_60)));
      kvs.push_back(DH::KV("after_tracker_60_80", std::to_string(output.after_tracker_60_80)));
      kvs.push_back(DH::KV("after_topology_20_30", std::to_string(output.after_topology_20_30)));
      kvs.push_back(DH::KV("after_topology_30_40", std::to_string(output.after_topology_30_40)));
      kvs.push_back(DH::KV("after_topology_40_50", std::to_string(output.after_topology_40_50)));
      kvs.push_back(DH::KV("after_topology_50_60", std::to_string(output.after_topology_50_60)));
      kvs.push_back(DH::KV("after_topology_60_80", std::to_string(output.after_topology_60_80)));
      kvs.push_back(
          DH::KV("tracker_confirmed_20_30", std::to_string(output.tracker_confirmed_20_30)));
      kvs.push_back(
          DH::KV("tracker_confirmed_30_40", std::to_string(output.tracker_confirmed_30_40)));
      kvs.push_back(
          DH::KV("tracker_confirmed_40_50", std::to_string(output.tracker_confirmed_40_50)));
      kvs.push_back(
          DH::KV("tracker_confirmed_50_60", std::to_string(output.tracker_confirmed_50_60)));
      kvs.push_back(
          DH::KV("tracker_confirmed_60_80", std::to_string(output.tracker_confirmed_60_80)));
      kvs.push_back(
          DH::KV("topo_interpolated_20_30", std::to_string(output.topo_interpolated_20_30)));
      kvs.push_back(
          DH::KV("topo_interpolated_30_40", std::to_string(output.topo_interpolated_30_40)));
      kvs.push_back(
          DH::KV("topo_interpolated_40_50", std::to_string(output.topo_interpolated_40_50)));
      kvs.push_back(
          DH::KV("topo_interpolated_50_60", std::to_string(output.topo_interpolated_50_60)));
      kvs.push_back(
          DH::KV("topo_interpolated_60_80", std::to_string(output.topo_interpolated_60_80)));
    }
  }

  diag_helper_.PublishStatus("perception_lidar", "perception_ros/lidar_cluster", level, msg, kvs,
                             last_header_.stamp);
}

void LidarClusterRos::publishDebugMarkers(const LidarClusterOutput& output) {
  if (!debug_publish_markers_ || debug_marker_pub_.getNumSubscribers() == 0) {
    return;
  }

  visualization_msgs::MarkerArray markers;
  // Delete all previous markers
  visualization_msgs::Marker del;
  del.action = visualization_msgs::Marker::DELETEALL;
  del.header = last_header_;
  del.ns = "cone_bbox";
  markers.markers.push_back(del);

  int id = 0;
  for (const auto& cone : output.cones) {
    visualization_msgs::Marker m;
    m.header = last_header_;
    m.ns = "cone_bbox";
    m.id = id++;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0.5 * (cone.min.x + cone.max.x);
    m.pose.position.y = 0.5 * (cone.min.y + cone.max.y);
    m.pose.position.z = 0.5 * (cone.min.z + cone.max.z);
    m.pose.orientation.w = 1.0;
    m.scale.x = std::max(0.05, static_cast<double>(cone.max.x - cone.min.x));
    m.scale.y = std::max(0.05, static_cast<double>(cone.max.y - cone.min.y));
    m.scale.z = std::max(0.05, static_cast<double>(cone.max.z - cone.min.z));
    // Color by confidence: green=high, red=low
    m.color.r = static_cast<float>(1.0 - cone.confidence);
    m.color.g = static_cast<float>(cone.confidence);
    m.color.b = 0.0f;
    m.color.a = 0.6f;
    m.lifetime = ros::Duration(0.2);
    markers.markers.push_back(m);
  }
  debug_marker_pub_.publish(markers);
}

void LidarClusterRos::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
  std::lock_guard<std::mutex> lock(fusion_mutex_);
  fusion_camera_info_msg_ = msg;
  fusion_camera_model_.fromCameraInfo(*msg);
  fusion_camera_model_ready_ = true;
}

void LidarClusterRos::syncPairCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const autodrive_msgs::HUAT_VisionDetections::ConstPtr& vision_msg) {
  SyncedPairCacheEntry entry;
  entry.cloud_stamp = cloud_msg->header.stamp;
  entry.vision_stamp = vision_msg->header.stamp;
  entry.sync_delta_sec = std::abs((entry.cloud_stamp - entry.vision_stamp).toSec());
  entry.vision_msg = vision_msg;

  {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    fusion_synced_pairs_.push_back(entry);
    while (fusion_synced_pairs_.size() > static_cast<size_t>(fusion_max_cache_size_)) {
      fusion_synced_pairs_.pop_front();
      fusion_sync_pair_cache_evicted_.fetch_add(1, std::memory_order_relaxed);
      fusion_sync_pair_dropped_no_raw_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  fusion_sync_pair_total_.fetch_add(1, std::memory_order_relaxed);
  fusion_last_sync_delta_sec_ = entry.sync_delta_sec;
  tryPublishFusedForStamp(entry.cloud_stamp);
}

void LidarClusterRos::pushRawFrame(const autodrive_msgs::HUAT_ConeDetections& detections) {
  RawFrameCacheEntry entry;
  entry.stamp = detections.header.stamp;
  entry.detections = detections;

  std::lock_guard<std::mutex> lock(fusion_mutex_);
  fusion_raw_cache_.push_back(std::move(entry));
  while (fusion_raw_cache_.size() > static_cast<size_t>(fusion_max_cache_size_)) {
    fusion_raw_cache_.pop_front();
    fusion_raw_cache_evicted_.fetch_add(1, std::memory_order_relaxed);
    fusion_frames_no_sync_.fetch_add(1, std::memory_order_relaxed);
  }
}

void LidarClusterRos::tryPublishFusedForStamp(const ros::Time& stamp) {
  RawFrameCacheEntry raw_entry;
  SyncedPairCacheEntry sync_entry;
  bool has_raw = false;
  bool has_sync = false;

  {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    auto raw_it =
        std::find_if(fusion_raw_cache_.begin(), fusion_raw_cache_.end(),
                     [&stamp](const RawFrameCacheEntry& entry) { return entry.stamp == stamp; });
    if (raw_it != fusion_raw_cache_.end()) {
      raw_entry = *raw_it;
      has_raw = true;
    }

    auto sync_it = std::find_if(
        fusion_synced_pairs_.begin(), fusion_synced_pairs_.end(),
        [&stamp](const SyncedPairCacheEntry& entry) { return entry.cloud_stamp == stamp; });
    if (sync_it != fusion_synced_pairs_.end()) {
      sync_entry = *sync_it;
      has_sync = true;
    }

    if (!(has_raw && has_sync)) {
      return;
    }

    fusion_raw_cache_.erase(raw_it);
    fusion_synced_pairs_.erase(sync_it);
  }

  publishFusedDetections(raw_entry.detections, &sync_entry);
  fusion_sync_pair_used_.fetch_add(1, std::memory_order_relaxed);
}

std::string LidarClusterRos::statusToReason(AssociationStatus status) const {
  switch (status) {
    case AssociationStatus::MATCHED:
      return "projected_bbox_match";
    case AssociationStatus::NO_SYNC_PAIR:
      return "sync_pair_missing";
    case AssociationStatus::CAMERA_INFO_MISSING:
      return "camera_info_missing";
    case AssociationStatus::TF_MISSING:
      return "tf_lookup_failed";
    case AssociationStatus::BEHIND_CAMERA:
      return "point_behind_camera";
    case AssociationStatus::OUT_OF_IMAGE:
      return "projection_out_of_image";
    case AssociationStatus::LOW_VISION_CONFIDENCE:
      return "vision_conf_below_threshold";
    case AssociationStatus::NO_BBOX_MATCH:
      return "no_bbox_match_after_projection";
    case AssociationStatus::LEGACY_HFOV_MATCH:
      return "legacy_hfov_match";
  }
  return "unknown";
}

uint8_t LidarClusterRos::matchLegacyHfovColor(
    const autodrive_msgs::HUAT_VisionDetections& vision_msg,
    const geometry_msgs::Point32& cone_point, int32_t* vision_confidence) const {
  const size_t n = std::min(std::min(vision_msg.x.size(), vision_msg.color_types.size()),
                            vision_msg.confidences.size());
  if (n == 0 || fusion_legacy_camera_width_px_ <= 0) {
    return 4;
  }

  const float cone_angle_deg =
      static_cast<float>(std::atan2(cone_point.y, cone_point.x) * 180.0 / M_PI);
  float best_angle_diff = static_cast<float>(fusion_legacy_match_angle_deg_);
  uint8_t best_color = 4;
  int32_t best_conf = 0;

  for (size_t i = 0; i < n; ++i) {
    const int32_t conf = vision_msg.confidences[i];
    if (conf < static_cast<int32_t>(fusion_min_vision_confidence_)) {
      continue;
    }

    const float px = static_cast<float>(vision_msg.x[i]);
    const float vision_angle = (0.5f - px / static_cast<float>(fusion_legacy_camera_width_px_)) *
                               static_cast<float>(fusion_legacy_camera_hfov_deg_);
    const float diff = std::abs(vision_angle - cone_angle_deg);
    if (diff < best_angle_diff) {
      best_angle_diff = diff;
      best_color = vision_msg.color_types[i];
      best_conf = conf;
    }
  }

  if (vision_confidence != nullptr) {
    *vision_confidence = best_conf;
  }
  return best_color;
}

void LidarClusterRos::publishFusedDetections(
    const autodrive_msgs::HUAT_ConeDetections& raw_detections,
    const SyncedPairCacheEntry* sync_pair) {
  autodrive_msgs::HUAT_FusedConeDetections fused;
  fused.header = raw_detections.header;
  fused.lidar_frame = raw_detections.header.frame_id.empty() ? fsd_common::frame_contract::kVelodyne
                                                             : raw_detections.header.frame_id;
  fused.lidar_stamp = raw_detections.header.stamp;

  if (sync_pair != nullptr) {
    fused.vision_stamp = sync_pair->vision_stamp;
    fused.sync_delta_sec = static_cast<float>(sync_pair->sync_delta_sec);
  } else {
    fused.vision_stamp = ros::Time(0);
    fused.sync_delta_sec = 0.0f;
  }

  sensor_msgs::CameraInfoConstPtr camera_info_msg;
  image_geometry::PinholeCameraModel camera_model;
  bool camera_model_ready = false;
  {
    std::lock_guard<std::mutex> lock(fusion_mutex_);
    camera_info_msg = fusion_camera_info_msg_;
    camera_model = fusion_camera_model_;
    camera_model_ready = fusion_camera_model_ready_;
  }
  fused.camera_info_available = camera_model_ready;
  fusion_last_camera_info_available_ = camera_model_ready;

  fused.camera_frame = fusion_camera_frame_;
  if (fused.camera_frame.empty() && camera_info_msg) {
    fused.camera_frame = camera_info_msg->header.frame_id;
  }
  if (fused.camera_frame.empty()) {
    fused.camera_frame = "camera";
  }

  geometry_msgs::TransformStamped lidar_to_camera_tf;
  bool tf_available = false;
  if (camera_model_ready) {
    try {
      lidar_to_camera_tf = tf_buffer_.lookupTransform(fused.camera_frame, fused.lidar_frame,
                                                      raw_detections.header.stamp,
                                                      ros::Duration(fusion_tf_timeout_sec_));
      tf_available = true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[perception] fusion TF unavailable: %s", ex.what());
      tf_available = false;
    }
  }
  fused.tf_available = tf_available;
  fusion_last_tf_available_ = tf_available;

  const autodrive_msgs::HUAT_VisionDetections* vision_msg =
      (sync_pair != nullptr) ? sync_pair->vision_msg.get() : nullptr;
  const size_t vision_n =
      (vision_msg == nullptr)
          ? 0
          : std::min(std::min(vision_msg->x.size(), vision_msg->y.size()),
                     std::min(vision_msg->color_types.size(), vision_msg->confidences.size()));
  std::vector<bool> vision_used(vision_n, false);

  const size_t n = raw_detections.points.size();
  fused.points.reserve(n);
  fused.obj_dist.reserve(n);
  fused.lidar_confidences.reserve(n);
  fused.lidar_color_types.reserve(n);
  fused.track_ids.reserve(n);
  fused.fused_color_types.reserve(n);
  fused.vision_confidences.reserve(n);
  fused.association_status.reserve(n);
  fused.association_scores.reserve(n);
  fused.association_reasons.reserve(n);

  uint32_t matched_count = 0;
  uint32_t unmatched_count = 0;

  for (size_t i = 0; i < n; ++i) {
    fused.points.push_back(raw_detections.points[i]);
    fused.obj_dist.push_back(i < raw_detections.obj_dist.size() ? raw_detections.obj_dist[i]
                                                                : 0.0f);
    fused.lidar_confidences.push_back(
        i < raw_detections.confidence.size() ? raw_detections.confidence[i] : 0.0f);
    const uint8_t raw_color =
        i < raw_detections.color_types.size() ? raw_detections.color_types[i] : 4;
    fused.lidar_color_types.push_back(raw_color);
    fused.track_ids.push_back(i < raw_detections.track_ids.size() ? raw_detections.track_ids[i]
                                                                  : -1);

    uint8_t fused_color = raw_color;
    int32_t matched_conf = -1;
    float score = 0.0f;
    AssociationStatus status = AssociationStatus::NO_SYNC_PAIR;

    if (vision_msg == nullptr) {
      status = AssociationStatus::NO_SYNC_PAIR;
      fusion_frames_no_sync_.fetch_add(1, std::memory_order_relaxed);
    } else if (!camera_model_ready && fusion_require_camera_info_) {
      status = AssociationStatus::CAMERA_INFO_MISSING;
      fusion_frames_camera_missing_.fetch_add(1, std::memory_order_relaxed);
    } else if (!tf_available && fusion_require_tf_) {
      status = AssociationStatus::TF_MISSING;
      fusion_frames_tf_missing_.fetch_add(1, std::memory_order_relaxed);
    } else if (camera_model_ready && tf_available) {
      geometry_msgs::PointStamped point_lidar;
      geometry_msgs::PointStamped point_camera;
      point_lidar.header = raw_detections.header;
      point_lidar.header.frame_id = fused.lidar_frame;
      point_lidar.point.x = raw_detections.points[i].x;
      point_lidar.point.y = raw_detections.points[i].y;
      point_lidar.point.z = raw_detections.points[i].z;
      tf2::doTransform(point_lidar, point_camera, lidar_to_camera_tf);

      if (point_camera.point.z <= 0.0) {
        status = AssociationStatus::BEHIND_CAMERA;
      } else {
        const cv::Point2d uv = camera_model.project3dToPixel(
            cv::Point3d(point_camera.point.x, point_camera.point.y, point_camera.point.z));
        const int image_width = static_cast<int>(camera_model.fullResolution().width);
        const int image_height = static_cast<int>(camera_model.fullResolution().height);
        if (uv.x < 0.0 || uv.y < 0.0 || uv.x >= static_cast<double>(image_width) ||
            uv.y >= static_cast<double>(image_height)) {
          status = AssociationStatus::OUT_OF_IMAGE;
        } else {
          float best_dist = std::numeric_limits<float>::max();
          int best_idx = -1;
          bool saw_low_conf_candidate = false;
          for (size_t j = 0; j < vision_n; ++j) {
            if (vision_used[j]) {
              continue;
            }
            const double dx = static_cast<double>(vision_msg->x[j]) - uv.x;
            const double dy = static_cast<double>(vision_msg->y[j]) - uv.y;
            const float dist = static_cast<float>(std::sqrt(dx * dx + dy * dy));
            if (dist > fusion_max_pixel_distance_) {
              continue;
            }
            const int32_t conf = vision_msg->confidences[j];
            if (conf < static_cast<int32_t>(fusion_min_vision_confidence_)) {
              saw_low_conf_candidate = true;
              continue;
            }
            if (dist < best_dist) {
              best_dist = dist;
              best_idx = static_cast<int>(j);
            }
          }

          if (best_idx >= 0) {
            const size_t idx = static_cast<size_t>(best_idx);
            vision_used[idx] = true;
            fused_color = vision_msg->color_types[idx];
            matched_conf = vision_msg->confidences[idx];
            status = AssociationStatus::MATCHED;
            score =
                std::max(0.0f, 1.0f - best_dist / static_cast<float>(fusion_max_pixel_distance_));

            // If vision detects unified YELLOW (1), classify by LiDAR size
            // YELLOW_SMALL (1) vs YELLOW_BIG (2) based on height/footprint
            if (fused_color == 1)  // YELLOW unified
            {
              const double height =
                  (i < raw_detections.maxPoints.size() && i < raw_detections.minPoints.size())
                      ? static_cast<double>(raw_detections.maxPoints[i].z -
                                            raw_detections.minPoints[i].z)
                      : 0.0;
              const double width_x =
                  (i < raw_detections.maxPoints.size() && i < raw_detections.minPoints.size())
                      ? static_cast<double>(raw_detections.maxPoints[i].x -
                                            raw_detections.minPoints[i].x)
                      : 0.0;
              const double width_y =
                  (i < raw_detections.maxPoints.size() && i < raw_detections.minPoints.size())
                      ? static_cast<double>(raw_detections.maxPoints[i].y -
                                            raw_detections.minPoints[i].y)
                      : 0.0;
              const double footprint_area = width_x * width_y;

              // Use same thresholds as LiDAR-only classification
              if (height >= big_cone_height_threshold_ ||
                  footprint_area >= big_cone_area_threshold_) {
                fused_color = 2;  // YELLOW_BIG
              }
              // else remains 1 (YELLOW_SMALL)
            }
          } else {
            status = saw_low_conf_candidate ? AssociationStatus::LOW_VISION_CONFIDENCE
                                            : AssociationStatus::NO_BBOX_MATCH;
          }
        }
      }
    }

    if (status != AssociationStatus::MATCHED && status != AssociationStatus::LEGACY_HFOV_MATCH &&
        fusion_legacy_hfov_fallback_enable_ && vision_msg != nullptr) {
      const uint8_t legacy_color =
          matchLegacyHfovColor(*vision_msg, raw_detections.points[i], &matched_conf);
      if (legacy_color != 4) {
        fused_color = legacy_color;
        status = AssociationStatus::LEGACY_HFOV_MATCH;
        score = 0.1f;
      }
    }

    if (!fusion_fallback_to_lidar_color_ && status != AssociationStatus::MATCHED &&
        status != AssociationStatus::LEGACY_HFOV_MATCH) {
      fused_color = 4;
    }

    if (status == AssociationStatus::MATCHED || status == AssociationStatus::LEGACY_HFOV_MATCH) {
      ++matched_count;
      fusion_association_matched_total_.fetch_add(1, std::memory_order_relaxed);
    } else {
      ++unmatched_count;
      fusion_association_unmatched_total_.fetch_add(1, std::memory_order_relaxed);
    }

    fused.fused_color_types.push_back(fused_color);
    fused.vision_confidences.push_back(matched_conf);
    fused.association_status.push_back(static_cast<uint8_t>(status));
    fused.association_scores.push_back(score);
    fused.association_reasons.push_back(statusToReason(status));
  }

  fused.matched_count = matched_count;
  fused.unmatched_count = unmatched_count;
  fusion_last_matched_count_ = static_cast<int>(matched_count);
  fusion_last_unmatched_count_ = static_cast<int>(unmatched_count);

  fused_detections_pub_.publish(fused);
  fusion_messages_published_.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace perception_ros
