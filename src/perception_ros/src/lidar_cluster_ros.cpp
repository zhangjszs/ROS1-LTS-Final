#include <perception_ros/lidar_cluster_ros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ros/serialization.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utility>

namespace perception_ros {

namespace {

constexpr std::uint8_t kConeOrangeSmall = 2;
constexpr std::uint8_t kConeOrangeBig = 3;
constexpr std::uint8_t kConeNone = 4;

std::uint8_t classifyConeTypeBySize(const ConeDetection &det,
                                    bool enable_size_typing,
                                    double big_height_threshold,
                                    double big_area_threshold)
{
  // LiDAR-only pipeline: geometry typing emits ORANGE_SMALL/ORANGE_BIG/NONE.
  // BLUE/YELLOW/RED are reserved for future camera-fused color outputs.
  if (!enable_size_typing)
  {
    return kConeNone;
  }

  const double height = std::max(0.0, static_cast<double>(det.max.z - det.min.z));
  const double width_x = std::max(0.0, static_cast<double>(det.max.x - det.min.x));
  const double width_y = std::max(0.0, static_cast<double>(det.max.y - det.min.y));
  const double footprint_area = width_x * width_y;

  if (height >= big_height_threshold || footprint_area >= big_area_threshold)
  {
    return kConeOrangeBig;
  }
  return kConeOrangeSmall;
}

bool LoadIntVector(const ros::NodeHandle &nh, const std::string &key, std::vector<int> &out)
{
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(key, value))
  {
    return false;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return false;
  }
  out.clear();
  out.reserve(static_cast<size_t>(value.size()));
  for (int i = 0; i < value.size(); ++i)
  {
    if (value[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      out.push_back(static_cast<int>(value[i]));
    }
    else if (value[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      out.push_back(static_cast<int>(static_cast<double>(value[i])));
    }
    else
    {
      return false;
    }
  }
  return true;
}

bool LoadDoubleVector(const ros::NodeHandle &nh, const std::string &key, std::vector<double> &out)
{
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(key, value))
  {
    return false;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return false;
  }
  out.clear();
  out.reserve(static_cast<size_t>(value.size()));
  for (int i = 0; i < value.size(); ++i)
  {
    if (value[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      out.push_back(static_cast<double>(value[i]));
    }
    else if (value[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      out.push_back(static_cast<double>(static_cast<int>(value[i])));
    }
    else
    {
      return false;
    }
  }
  return true;
}

}  // namespace

LidarClusterRos::LidarClusterRos(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh))
{
  loadParams();
  core_.Configure(config_);

  sub_point_cloud_ = nh_.subscribe(input_topic_, 2, &LidarClusterRos::pointCallback, this);

  passthrough_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(passthrough_topic_, 1);
  no_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(no_ground_topic_, 1);
  cones_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cones_topic_, 1);
  detections_pub_ = nh_.advertise<autodrive_msgs::HUAT_ConeDetections>(detections_topic_, 10);

  // Initialize distortion compensator V2 (支持time字段、预积分、外参)
  auto compensator_config = DistortionCompensatorV2Config::LoadFromRos(private_nh_);
  if (compensator_config.enable) {
    compensator_ = std::make_unique<DistortionCompensatorV2>(nh_, compensator_config);
  }

  ROS_INFO("lidar cluster finished initialization");
}

bool LidarClusterRos::IsLegacyPollMode() const
{
  return pipeline_mode_ == "legacy_poll";
}

double LidarClusterRos::LegacyPollHz() const
{
  return legacy_poll_hz_;
}

void LidarClusterRos::loadParams()
{
  if (!private_nh_.param<std::string>("topics/input", input_topic_, "points/raw"))
  {
    ROS_WARN_STREAM("Did not load topics/input. Standard value is: " << input_topic_);
  }
  if (!private_nh_.param<std::string>("topics/points/passthrough", passthrough_topic_, "points/passthrough"))
  {
    ROS_WARN_STREAM("Did not load topics/points/passthrough. Standard value is: " << passthrough_topic_);
  }
  if (!private_nh_.param<std::string>("topics/points/no_ground", no_ground_topic_, "points/no_ground"))
  {
    ROS_WARN_STREAM("Did not load topics/points/no_ground. Standard value is: " << no_ground_topic_);
  }
  if (!private_nh_.param<std::string>("topics/points/cones", cones_topic_, "points/cones"))
  {
    ROS_WARN_STREAM("Did not load topics/points/cones. Standard value is: " << cones_topic_);
  }
  if (!private_nh_.param<std::string>("topics/detections", detections_topic_, "detections"))
  {
    ROS_WARN_STREAM("Did not load topics/detections. Standard value is: " << detections_topic_);
  }

  // 性能优化选项
  if (!private_nh_.param<bool>("use_point_cloud_pool", use_point_cloud_pool_, false))
  {
    ROS_INFO_STREAM("Did not load use_point_cloud_pool. Standard value is: " << use_point_cloud_pool_);
  }
  if (use_point_cloud_pool_)
  {
    ROS_INFO("Point cloud pool enabled for memory optimization");
  }

  if (!private_nh_.param<std::string>("pipeline_mode", pipeline_mode_, "event_driven"))
  {
    ROS_WARN_STREAM("Did not load pipeline_mode. Standard value is: " << pipeline_mode_);
  }
  if (pipeline_mode_ != "event_driven" && pipeline_mode_ != "legacy_poll")
  {
    ROS_WARN_STREAM("Invalid pipeline_mode: " << pipeline_mode_ << ", fallback to event_driven");
    pipeline_mode_ = "event_driven";
  }
  if (!private_nh_.param<double>("legacy_poll_hz", legacy_poll_hz_, 10.0))
  {
    ROS_WARN_STREAM("Did not load legacy_poll_hz. Standard value is: " << legacy_poll_hz_);
  }
  if (legacy_poll_hz_ <= 0.0)
  {
    ROS_WARN_STREAM("Invalid legacy_poll_hz: " << legacy_poll_hz_ << ", fallback to 10.0");
    legacy_poll_hz_ = 10.0;
  }

  if (!private_nh_.param<double>("sensor_height", config_.sensor_height, 0.135))
  {
    ROS_WARN_STREAM("Did not load sensor_height. Standard value is: " << config_.sensor_height);
  }
  if (!private_nh_.param<int>("sensor_model", config_.sensor_model, 16))
  {
    ROS_WARN_STREAM("Did not load sensor_model. Standard value is: " << config_.sensor_model);
  }
  if (!private_nh_.param<int>("ransac/num_iter", config_.ransac.num_iter, 3))
  {
    ROS_WARN_STREAM("Did not load ransac/num_iter. Standard value is: " << config_.ransac.num_iter);
  }
  if (!private_nh_.param<int>("ransac/num_lpr", config_.ransac.num_lpr, 5))
  {
    ROS_WARN_STREAM("Did not load ransac/num_lpr. Standard value is: " << config_.ransac.num_lpr);
  }
  if (!private_nh_.param<double>("ransac/th_seeds", config_.ransac.th_seeds, 0.03))
  {
    ROS_WARN_STREAM("Did not load ransac/th_seeds. Standard value is: " << config_.ransac.th_seeds);
  }
  if (!private_nh_.param<double>("ransac/th_dist", config_.ransac.th_dist, 0.03))
  {
    ROS_WARN_STREAM("Did not load ransac/th_dist. Standard value is: " << config_.ransac.th_dist);
  }
  // RANSAC 优化参数
  private_nh_.param<bool>("ransac/enable_zone", config_.ransac.enable_zone, true);
  if (!LoadDoubleVector(private_nh_, "ransac/zone_boundaries", config_.ransac.zone_boundaries))
  {
    config_.ransac.zone_boundaries = {10.0, 20.0, 30.0};  // 默认值
  }
  private_nh_.param<bool>("ransac/adaptive_threshold", config_.ransac.adaptive_threshold, true);
  private_nh_.param<double>("ransac/th_dist_far_scale", config_.ransac.th_dist_far_scale, 2.0);
  private_nh_.param<double>("ransac/min_normal_z", config_.ransac.min_normal_z, 0.8);
  private_nh_.param<bool>("ransac/progressive_iteration", config_.ransac.progressive_iteration, true);
  if (!private_nh_.param<int>("road_type", config_.road_type, 2))
  {
    ROS_WARN_STREAM("Did not load road_type. Standard value is: " << config_.road_type);
  }
  if (!private_nh_.param<std::string>("ground_method", config_.ground_method, "ransac"))
  {
    ROS_WARN_STREAM("Did not load ground_method. Standard value is: " << config_.ground_method);
  }
  if (!private_nh_.param<bool>("force_fgs_fast_path", force_fgs_fast_path_, true))
  {
    ROS_WARN_STREAM("Did not load force_fgs_fast_path. Standard value is: " << force_fgs_fast_path_);
  }
  if (force_fgs_fast_path_ && config_.ground_method != "fgs")
  {
    ROS_WARN_STREAM("force_fgs_fast_path enabled: override ground_method from "
                    << config_.ground_method << " to fgs");
    config_.ground_method = "fgs";
  }
  if (!private_nh_.param<bool>("ground_watchdog/enable", ground_watchdog_enable_, true))
  {
    ROS_WARN_STREAM("Did not load ground_watchdog/enable. Standard value is: " << ground_watchdog_enable_);
  }
  if (!private_nh_.param<double>("ground_watchdog/warn_ms", ground_watchdog_warn_ms_, 8.0))
  {
    ROS_WARN_STREAM("Did not load ground_watchdog/warn_ms. Standard value is: " << ground_watchdog_warn_ms_);
  }
  if (!private_nh_.param<int>("ground_watchdog/warn_consecutive_frames", ground_watchdog_warn_frames_, 5))
  {
    ROS_WARN_STREAM("Did not load ground_watchdog/warn_consecutive_frames. Standard value is: "
                    << ground_watchdog_warn_frames_);
  }
  if (ground_watchdog_warn_frames_ < 1)
  {
    ROS_WARN_STREAM("Invalid ground_watchdog/warn_consecutive_frames: "
                    << ground_watchdog_warn_frames_ << ", fallback to 1");
    ground_watchdog_warn_frames_ = 1;
  }
  if (!private_nh_.param<std::string>("roi/mode", config_.roi.mode, "track"))
  {
    ROS_WARN_STREAM("Did not load roi/mode. Standard value is: " << config_.roi.mode);
  }
  if (!private_nh_.param<bool>("roi/use_point_clip", config_.roi.use_point_clip, false))
  {
    ROS_WARN_STREAM("Did not load roi/use_point_clip. Standard value is: " << config_.roi.use_point_clip);
  }
  if (!private_nh_.param<double>("roi/z_min", config_.roi.z_min, -1.0))
  {
    ROS_WARN_STREAM("Did not load roi/z_min. Standard value is: " << config_.roi.z_min);
  }
  if (!private_nh_.param<double>("roi/z_max", config_.roi.z_max, 0.7))
  {
    ROS_WARN_STREAM("Did not load roi/z_max. Standard value is: " << config_.roi.z_max);
  }
  if (!private_nh_.param<double>("roi/skidpad/x_min", config_.roi.skidpad.x_min, 0.0))
  {
    ROS_WARN_STREAM("Did not load roi/skidpad/x_min. Standard value is: " << config_.roi.skidpad.x_min);
  }
  if (!private_nh_.param<double>("roi/skidpad/x_max", config_.roi.skidpad.x_max, 10.0))
  {
    ROS_WARN_STREAM("Did not load roi/skidpad/x_max. Standard value is: " << config_.roi.skidpad.x_max);
  }
  if (!private_nh_.param<double>("roi/skidpad/y_min", config_.roi.skidpad.y_min, -3.0))
  {
    ROS_WARN_STREAM("Did not load roi/skidpad/y_min. Standard value is: " << config_.roi.skidpad.y_min);
  }
  if (!private_nh_.param<double>("roi/skidpad/y_max", config_.roi.skidpad.y_max, 3.0))
  {
    ROS_WARN_STREAM("Did not load roi/skidpad/y_max. Standard value is: " << config_.roi.skidpad.y_max);
  }
  if (!private_nh_.param<double>("roi/accel/x_min", config_.roi.accel.x_min, 0.0))
  {
    ROS_WARN_STREAM("Did not load roi/accel/x_min. Standard value is: " << config_.roi.accel.x_min);
  }
  if (!private_nh_.param<double>("roi/accel/x_max", config_.roi.accel.x_max, 100.0))
  {
    ROS_WARN_STREAM("Did not load roi/accel/x_max. Standard value is: " << config_.roi.accel.x_max);
  }
  if (!private_nh_.param<double>("roi/accel/y_min", config_.roi.accel.y_min, -3.0))
  {
    ROS_WARN_STREAM("Did not load roi/accel/y_min. Standard value is: " << config_.roi.accel.y_min);
  }
  if (!private_nh_.param<double>("roi/accel/y_max", config_.roi.accel.y_max, 3.0))
  {
    ROS_WARN_STREAM("Did not load roi/accel/y_max. Standard value is: " << config_.roi.accel.y_max);
  }
  if (!private_nh_.param<double>("roi/track/x_min", config_.roi.track.x_min, 0.0))
  {
    ROS_WARN_STREAM("Did not load roi/track/x_min. Standard value is: " << config_.roi.track.x_min);
  }
  if (!private_nh_.param<double>("roi/track/x_max", config_.roi.track.x_max, 20.0))
  {
    ROS_WARN_STREAM("Did not load roi/track/x_max. Standard value is: " << config_.roi.track.x_max);
  }
  if (!private_nh_.param<double>("roi/track/y_min", config_.roi.track.y_min, -3.0))
  {
    ROS_WARN_STREAM("Did not load roi/track/y_min. Standard value is: " << config_.roi.track.y_min);
  }
  if (!private_nh_.param<double>("roi/track/y_max", config_.roi.track.y_max, 3.0))
  {
    ROS_WARN_STREAM("Did not load roi/track/y_max. Standard value is: " << config_.roi.track.y_max);
  }
  if (!private_nh_.param<double>("roi/custom/x_min", config_.roi.custom.x_min, 0.0))
  {
    ROS_WARN_STREAM("Did not load roi/custom/x_min. Standard value is: " << config_.roi.custom.x_min);
  }
  if (!private_nh_.param<double>("roi/custom/x_max", config_.roi.custom.x_max, 20.0))
  {
    ROS_WARN_STREAM("Did not load roi/custom/x_max. Standard value is: " << config_.roi.custom.x_max);
  }
  if (!private_nh_.param<double>("roi/custom/y_min", config_.roi.custom.y_min, -3.0))
  {
    ROS_WARN_STREAM("Did not load roi/custom/y_min. Standard value is: " << config_.roi.custom.y_min);
  }
  if (!private_nh_.param<double>("roi/custom/y_max", config_.roi.custom.y_max, 3.0))
  {
    ROS_WARN_STREAM("Did not load roi/custom/y_max. Standard value is: " << config_.roi.custom.y_max);
  }
  if (!private_nh_.param<bool>("filters/sor/enable", config_.filters.sor.enable, false))
  {
    ROS_WARN_STREAM("Did not load filters/sor/enable. Standard value is: " << config_.filters.sor.enable);
  }
  if (!private_nh_.param<int>("filters/sor/mean_k", config_.filters.sor.mean_k, 50))
  {
    ROS_WARN_STREAM("Did not load filters/sor/mean_k. Standard value is: " << config_.filters.sor.mean_k);
  }
  if (!private_nh_.param<double>("filters/sor/stddev_mul", config_.filters.sor.stddev_mul, 1.0))
  {
    ROS_WARN_STREAM("Did not load filters/sor/stddev_mul. Standard value is: " << config_.filters.sor.stddev_mul);
  }
  if (!private_nh_.param<bool>("filters/voxel/enable", config_.filters.voxel.enable, true))
  {
    ROS_WARN_STREAM("Did not load filters/voxel/enable. Standard value is: " << config_.filters.voxel.enable);
  }
  if (!private_nh_.param<double>("filters/voxel/leaf_size", config_.filters.voxel.leaf_size, 0.05))
  {
    ROS_WARN_STREAM("Did not load filters/voxel/leaf_size. Standard value is: " << config_.filters.voxel.leaf_size);
  }
  if (!private_nh_.param<bool>("filters/adaptive_voxel/enable", config_.filters.adaptive_voxel.enable, false))
  {
    ROS_WARN_STREAM("Did not load filters/adaptive_voxel/enable. Standard value is: " << config_.filters.adaptive_voxel.enable);
  }
  if (!private_nh_.param<double>("filters/adaptive_voxel/leaf_size", config_.filters.adaptive_voxel.leaf_size, 0.1))
  {
    ROS_WARN_STREAM("Did not load filters/adaptive_voxel/leaf_size. Standard value is: " << config_.filters.adaptive_voxel.leaf_size);
  }
  if (!private_nh_.param<int>("filters/adaptive_voxel/density_thr", config_.filters.adaptive_voxel.density_thr, 50))
  {
    ROS_WARN_STREAM("Did not load filters/adaptive_voxel/density_thr. Standard value is: " << config_.filters.adaptive_voxel.density_thr);
  }
  // 距离自适应体素滤波参数
  if (!private_nh_.param<bool>("filters/distance_adaptive_voxel/enable", config_.filters.distance_adaptive_voxel.enable, false))
  {
    ROS_WARN_STREAM("Did not load filters/distance_adaptive_voxel/enable. Standard value is: " << config_.filters.distance_adaptive_voxel.enable);
  }
  if (!private_nh_.param<double>("filters/distance_adaptive_voxel/near_leaf", config_.filters.distance_adaptive_voxel.near_leaf, 0.08))
  {
    ROS_WARN_STREAM("Did not load filters/distance_adaptive_voxel/near_leaf. Standard value is: " << config_.filters.distance_adaptive_voxel.near_leaf);
  }
  if (!private_nh_.param<double>("filters/distance_adaptive_voxel/far_leaf", config_.filters.distance_adaptive_voxel.far_leaf, 0.03))
  {
    ROS_WARN_STREAM("Did not load filters/distance_adaptive_voxel/far_leaf. Standard value is: " << config_.filters.distance_adaptive_voxel.far_leaf);
  }
  if (!private_nh_.param<double>("filters/distance_adaptive_voxel/dist_threshold", config_.filters.distance_adaptive_voxel.dist_threshold, 10.0))
  {
    ROS_WARN_STREAM("Did not load filters/distance_adaptive_voxel/dist_threshold. Standard value is: " << config_.filters.distance_adaptive_voxel.dist_threshold);
  }
  // 强度滤波参数
  if (!private_nh_.param<bool>("filters/intensity/enable", config_.filters.intensity.enable, false))
  {
    ROS_WARN_STREAM("Did not load filters/intensity/enable. Standard value is: " << config_.filters.intensity.enable);
  }
  {
    double min_intensity_d = static_cast<double>(config_.filters.intensity.min_intensity);
    if (!private_nh_.param<double>("filters/intensity/min_intensity", min_intensity_d, 5.0))
    {
      ROS_WARN_STREAM("Did not load filters/intensity/min_intensity. Standard value is: " << min_intensity_d);
    }
    config_.filters.intensity.min_intensity = static_cast<float>(min_intensity_d);
  }
  // CropBox优化开关
  if (!private_nh_.param<bool>("filters/use_cropbox", config_.filters.use_cropbox, true))
  {
    ROS_WARN_STREAM("Did not load filters/use_cropbox. Standard value is: " << config_.filters.use_cropbox);
  }
  // 柱状障碍物滤波参数（基于局部z跨度去除树/墙壁）
  private_nh_.param<bool>("filters/obstacle_height/enable", config_.filters.obstacle_height.enable, true);
  private_nh_.param<double>("filters/obstacle_height/grid_size", config_.filters.obstacle_height.grid_size, 0.5);
  private_nh_.param<double>("filters/obstacle_height/max_z_span", config_.filters.obstacle_height.max_z_span, 0.5);
  private_nh_.param<int>("filters/obstacle_height/min_points_to_judge", config_.filters.obstacle_height.min_points_to_judge, 3);
  private_nh_.param<double>("filters/obstacle_height/min_distance", config_.filters.obstacle_height.min_distance, 10.0);

  // FGS (Fast Ground Segmentation) 参数
  private_nh_.param<int>("fgs/num_sectors", config_.fgs.num_sectors, 32);
  private_nh_.param<int>("fgs/num_bins", config_.fgs.num_bins, 80);
  private_nh_.param<double>("fgs/max_range", config_.fgs.max_range, 80.0);
  private_nh_.param<double>("fgs/min_range", config_.fgs.min_range, 0.1);
  private_nh_.param<double>("fgs/sensor_height", config_.fgs.sensor_height, 0.135);
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
  private_nh_.param<bool>("fgs/enable_temporal_smoothing", config_.fgs.enable_temporal_smoothing, true);
  private_nh_.param<double>("fgs/temporal_alpha", config_.fgs.temporal_alpha, 0.3);
  // 地面判定阈值对称性
  private_nh_.param<double>("fgs/ground_below_factor", config_.fgs.ground_below_factor, 1.5);
  // temporal alpha 自适应（S弯/快速变向）
  private_nh_.param<bool>("fgs/enable_adaptive_alpha", config_.fgs.enable_adaptive_alpha, true);
  private_nh_.param<double>("fgs/adaptive_alpha_max", config_.fgs.adaptive_alpha_max, 0.9);
  private_nh_.param<double>("fgs/adaptive_alpha_threshold", config_.fgs.adaptive_alpha_threshold, 0.05);

  if (!private_nh_.param<bool>("patchworkpp/enable_rnr", config_.patchworkpp.enable_rnr, true))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/enable_rnr. Standard value is: " << config_.patchworkpp.enable_rnr);
  }
  if (!private_nh_.param<bool>("patchworkpp/enable_rvpf", config_.patchworkpp.enable_rvpf, true))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/enable_rvpf. Standard value is: " << config_.patchworkpp.enable_rvpf);
  }
  if (!private_nh_.param<bool>("patchworkpp/enable_tgr", config_.patchworkpp.enable_tgr, true))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/enable_tgr. Standard value is: " << config_.patchworkpp.enable_tgr);
  }
  if (!private_nh_.param<int>("patchworkpp/num_iter", config_.patchworkpp.num_iter, 3))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/num_iter. Standard value is: " << config_.patchworkpp.num_iter);
  }
  if (!private_nh_.param<int>("patchworkpp/num_lpr", config_.patchworkpp.num_lpr, 20))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/num_lpr. Standard value is: " << config_.patchworkpp.num_lpr);
  }
  if (!private_nh_.param<int>("patchworkpp/num_min_pts", config_.patchworkpp.num_min_pts, 10))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/num_min_pts. Standard value is: " << config_.patchworkpp.num_min_pts);
  }
  if (!private_nh_.param<int>("patchworkpp/num_zones", config_.patchworkpp.num_zones, 4))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/num_zones. Standard value is: " << config_.patchworkpp.num_zones);
  }
  if (!private_nh_.param<int>("patchworkpp/num_rings_of_interest", config_.patchworkpp.num_rings_of_interest, 4))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/num_rings_of_interest. Standard value is: " << config_.patchworkpp.num_rings_of_interest);
  }
  if (!private_nh_.param<double>("patchworkpp/rnr_ver_angle_thr", config_.patchworkpp.rnr_ver_angle_thr, -15.0))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/rnr_ver_angle_thr. Standard value is: " << config_.patchworkpp.rnr_ver_angle_thr);
  }
  if (!private_nh_.param<double>("patchworkpp/rnr_intensity_thr", config_.patchworkpp.rnr_intensity_thr, 0.2))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/rnr_intensity_thr. Standard value is: " << config_.patchworkpp.rnr_intensity_thr);
  }
  if (!private_nh_.param<double>("patchworkpp/th_seeds", config_.patchworkpp.th_seeds, 0.125))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/th_seeds. Standard value is: " << config_.patchworkpp.th_seeds);
  }
  if (!private_nh_.param<double>("patchworkpp/th_dist", config_.patchworkpp.th_dist, 0.125))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/th_dist. Standard value is: " << config_.patchworkpp.th_dist);
  }
  if (!private_nh_.param<double>("patchworkpp/th_seeds_v", config_.patchworkpp.th_seeds_v, 0.25))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/th_seeds_v. Standard value is: " << config_.patchworkpp.th_seeds_v);
  }
  if (!private_nh_.param<double>("patchworkpp/th_dist_v", config_.patchworkpp.th_dist_v, 0.1))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/th_dist_v. Standard value is: " << config_.patchworkpp.th_dist_v);
  }
  if (!private_nh_.param<double>("patchworkpp/max_range", config_.patchworkpp.max_range, 80.0))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/max_range. Standard value is: " << config_.patchworkpp.max_range);
  }
  if (!private_nh_.param<double>("patchworkpp/min_range", config_.patchworkpp.min_range, 2.7))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/min_range. Standard value is: " << config_.patchworkpp.min_range);
  }
  if (!private_nh_.param<double>("patchworkpp/uprightness_thr", config_.patchworkpp.uprightness_thr, 0.707))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/uprightness_thr. Standard value is: " << config_.patchworkpp.uprightness_thr);
  }
  if (!private_nh_.param<double>("patchworkpp/adaptive_seed_selection_margin", config_.patchworkpp.adaptive_seed_selection_margin, -1.2))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/adaptive_seed_selection_margin. Standard value is: " << config_.patchworkpp.adaptive_seed_selection_margin);
  }
  if (!private_nh_.param<int>("patchworkpp/max_flatness_storage", config_.patchworkpp.max_flatness_storage, 1000))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/max_flatness_storage. Standard value is: " << config_.patchworkpp.max_flatness_storage);
  }
  if (!private_nh_.param<int>("patchworkpp/max_elevation_storage", config_.patchworkpp.max_elevation_storage, 1000))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/max_elevation_storage. Standard value is: " << config_.patchworkpp.max_elevation_storage);
  }
  if (!LoadIntVector(private_nh_, "patchworkpp/num_sectors_each_zone", config_.patchworkpp.num_sectors_each_zone))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/num_sectors_each_zone. Standard value is used.");
  }
  if (!LoadIntVector(private_nh_, "patchworkpp/num_rings_each_zone", config_.patchworkpp.num_rings_each_zone))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/num_rings_each_zone. Standard value is used.");
  }
  if (!LoadDoubleVector(private_nh_, "patchworkpp/elevation_thr", config_.patchworkpp.elevation_thr))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/elevation_thr. Standard value is used.");
  }
  if (!LoadDoubleVector(private_nh_, "patchworkpp/flatness_thr", config_.patchworkpp.flatness_thr))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/flatness_thr. Standard value is used.");
  }
  // Patchwork++ 优化参数
  if (!private_nh_.param<double>("patchworkpp/th_dist_far_scale", config_.patchworkpp.th_dist_far_scale, 1.5))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/th_dist_far_scale. Standard value is: " << config_.patchworkpp.th_dist_far_scale);
  }
  if (!private_nh_.param<double>("patchworkpp/min_normal_z", config_.patchworkpp.min_normal_z, 0.7))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/min_normal_z. Standard value is: " << config_.patchworkpp.min_normal_z);
  }
  if (!private_nh_.param<double>("patchworkpp/far_zone_min_pts_scale", config_.patchworkpp.far_zone_min_pts_scale, 2.0))
  {
    ROS_WARN_STREAM("Did not load patchworkpp/far_zone_min_pts_scale. Standard value is: " << config_.patchworkpp.far_zone_min_pts_scale);
  }
  if (!private_nh_.param<std::string>("str_range", config_.str_range, "15,30,45,60"))
  {
    ROS_WARN_STREAM("Did not load str_range. Standard value is: " << config_.str_range);
  }
  if (!private_nh_.param<std::string>("str_seg_distance", config_.str_seg_distance, "0.5,1.1,1.6,2.1,2.6"))
  {
    ROS_WARN_STREAM("Did not load str_seg_distance. Standard value is: " << config_.str_seg_distance);
  }

  // 聚类配置
  if (!LoadDoubleVector(private_nh_, "cluster/distance_segments", config_.cluster.distance_segments))
  {
    ROS_WARN_STREAM("Did not load cluster/distance_segments. Using defaults.");
  }
  if (!LoadDoubleVector(private_nh_, "cluster/cluster_tolerance", config_.cluster.cluster_tolerance))
  {
    ROS_WARN_STREAM("Did not load cluster/cluster_tolerance. Using defaults.");
  }
  if (!private_nh_.param<bool>("cluster/adaptive_size/enable", config_.cluster.adaptive_size.enable, true))
  {
    ROS_WARN_STREAM("Did not load cluster/adaptive_size/enable. Standard value is: " << config_.cluster.adaptive_size.enable);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/near_min_size", config_.cluster.adaptive_size.near_min_size, 3))
  {
    ROS_WARN_STREAM("Did not load cluster/adaptive_size/near_min_size. Standard value is: " << config_.cluster.adaptive_size.near_min_size);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/near_max_size", config_.cluster.adaptive_size.near_max_size, 100))
  {
    ROS_WARN_STREAM("Did not load cluster/adaptive_size/near_max_size. Standard value is: " << config_.cluster.adaptive_size.near_max_size);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/far_min_size", config_.cluster.adaptive_size.far_min_size, 1))
  {
    ROS_WARN_STREAM("Did not load cluster/adaptive_size/far_min_size. Standard value is: " << config_.cluster.adaptive_size.far_min_size);
  }
  if (!private_nh_.param<int>("cluster/adaptive_size/far_max_size", config_.cluster.adaptive_size.far_max_size, 30))
  {
    ROS_WARN_STREAM("Did not load cluster/adaptive_size/far_max_size. Standard value is: " << config_.cluster.adaptive_size.far_max_size);
  }
  if (!private_nh_.param<int>("cluster/min_cluster_size", config_.cluster.min_cluster_size, 1))
  {
    ROS_WARN_STREAM("Did not load cluster/min_cluster_size. Standard value is: " << config_.cluster.min_cluster_size);
  }
  if (!private_nh_.param<int>("cluster/max_cluster_size", config_.cluster.max_cluster_size, 50))
  {
    ROS_WARN_STREAM("Did not load cluster/max_cluster_size. Standard value is: " << config_.cluster.max_cluster_size);
  }
  // 聚类方法选择
  if (!private_nh_.param<std::string>("cluster/method", config_.cluster.method, "euclidean"))
  {
    ROS_WARN_STREAM("Did not load cluster/method. Standard value is: " << config_.cluster.method);
  }
  // DBSCAN 参数
  if (!private_nh_.param<double>("cluster/dbscan/eps", config_.cluster.dbscan.eps, 0.3))
  {
    ROS_WARN_STREAM("Did not load cluster/dbscan/eps. Standard value is: " << config_.cluster.dbscan.eps);
  }
  if (!private_nh_.param<int>("cluster/dbscan/min_pts", config_.cluster.dbscan.min_pts, 3))
  {
    ROS_WARN_STREAM("Did not load cluster/dbscan/min_pts. Standard value is: " << config_.cluster.dbscan.min_pts);
  }
  if (!private_nh_.param<bool>("cluster/dbscan/adaptive_eps", config_.cluster.dbscan.adaptive_eps, true))
  {
    ROS_WARN_STREAM("Did not load cluster/dbscan/adaptive_eps. Standard value is: " << config_.cluster.dbscan.adaptive_eps);
  }
  if (!private_nh_.param<double>("cluster/dbscan/eps_near", config_.cluster.dbscan.eps_near, 0.15))
  {
    ROS_WARN_STREAM("Did not load cluster/dbscan/eps_near. Standard value is: " << config_.cluster.dbscan.eps_near);
  }
  if (!private_nh_.param<double>("cluster/dbscan/eps_far", config_.cluster.dbscan.eps_far, 0.5))
  {
    ROS_WARN_STREAM("Did not load cluster/dbscan/eps_far. Standard value is: " << config_.cluster.dbscan.eps_far);
  }
  // VLP-16 参数
  if (!private_nh_.param<double>("cluster/vlp16/cluster_tolerance", config_.cluster.vlp16.cluster_tolerance, 0.3))
  {
    ROS_WARN_STREAM("Did not load cluster/vlp16/cluster_tolerance. Standard value is: " << config_.cluster.vlp16.cluster_tolerance);
  }
  if (!private_nh_.param<double>("cluster/vlp16/max_bbox_x", config_.cluster.vlp16.max_bbox_x, 0.4))
  {
    ROS_WARN_STREAM("Did not load cluster/vlp16/max_bbox_x. Standard value is: " << config_.cluster.vlp16.max_bbox_x);
  }
  if (!private_nh_.param<double>("cluster/vlp16/max_bbox_y", config_.cluster.vlp16.max_bbox_y, 0.4))
  {
    ROS_WARN_STREAM("Did not load cluster/vlp16/max_bbox_y. Standard value is: " << config_.cluster.vlp16.max_bbox_y);
  }
  if (!private_nh_.param<double>("cluster/vlp16/max_bbox_z", config_.cluster.vlp16.max_bbox_z, 0.5))
  {
    ROS_WARN_STREAM("Did not load cluster/vlp16/max_bbox_z. Standard value is: " << config_.cluster.vlp16.max_bbox_z);
  }
  // point_clip 参数
  if (!private_nh_.param<double>("cluster/point_clip/min_distance", config_.cluster.point_clip.min_distance, 1.0))
  {
    ROS_WARN_STREAM("Did not load cluster/point_clip/min_distance. Standard value is: " << config_.cluster.point_clip.min_distance);
  }
  if (!private_nh_.param<double>("cluster/point_clip/max_distance", config_.cluster.point_clip.max_distance, 15.0))
  {
    ROS_WARN_STREAM("Did not load cluster/point_clip/max_distance. Standard value is: " << config_.cluster.point_clip.max_distance);
  }
  // FEC (Fast Euclidean Clustering) 参数
  if (!private_nh_.param<bool>("cluster/fec/enable", config_.cluster.fec.enable, true))
  {
    ROS_WARN_STREAM("Did not load cluster/fec/enable. Standard value is: " << config_.cluster.fec.enable);
  }
  if (!private_nh_.param<double>("cluster/fec/quality", config_.cluster.fec.quality, 0.3))
  {
    ROS_WARN_STREAM("Did not load cluster/fec/quality. Standard value is: " << config_.cluster.fec.quality);
  }
  // 多帧累积参数
  if (!private_nh_.param<bool>("cluster/multi_frame/enable", config_.cluster.multi_frame.enable, true))
  {
    ROS_WARN_STREAM("Did not load cluster/multi_frame/enable. Standard value is: " << config_.cluster.multi_frame.enable);
  }
  if (!private_nh_.param<int>("cluster/multi_frame/num_frames", config_.cluster.multi_frame.num_frames, 2))
  {
    ROS_WARN_STREAM("Did not load cluster/multi_frame/num_frames. Standard value is: " << config_.cluster.multi_frame.num_frames);
  }
  if (!private_nh_.param<double>("cluster/multi_frame/max_distance", config_.cluster.multi_frame.max_distance, 10.0))
  {
    ROS_WARN_STREAM("Did not load cluster/multi_frame/max_distance. Standard value is: " << config_.cluster.multi_frame.max_distance);
  }

  if (!private_nh_.param<double>("min_height", config_.min_height, -1))
  {
    ROS_WARN("Did not load min_height.");
  }
  if (!private_nh_.param<double>("max_height", config_.max_height, -1))
  {
    ROS_WARN("Did not load max_height.");
  }
  if (!private_nh_.param<double>("min_area", config_.min_area, -1))
  {
    ROS_WARN("Did not load min_area.");
  }
  if (!private_nh_.param<double>("max_area", config_.max_area, -1))
  {
    ROS_WARN("Did not load max_area.");
  }

  if (!private_nh_.param<double>("max_box_altitude", config_.max_box_altitude, 0))
  {
    ROS_WARN("Did not load max_box_altitude.");
  }

  // ---- Confidence 参数加载（修复：之前YAML中的confidence/*参数从未被读取） ----
  private_nh_.param<double>("confidence/min_height", config_.confidence.min_height, 0.15);
  private_nh_.param<double>("confidence/max_height", config_.confidence.max_height, 0.5);
  private_nh_.param<double>("confidence/min_area", config_.confidence.min_area, 0.01);
  private_nh_.param<double>("confidence/max_area", config_.confidence.max_area, 0.15);
  private_nh_.param<double>("confidence/max_box_altitude", config_.confidence.max_box_altitude, 0.5);
  private_nh_.param<double>("confidence/min_aspect_ratio", config_.confidence.min_aspect_ratio, 1.5);
  private_nh_.param<double>("confidence/min_verticality", config_.confidence.min_verticality, 0.8);
  private_nh_.param<double>("confidence/max_linearity", config_.confidence.max_linearity, 0.85);
  private_nh_.param<double>("confidence/min_density_near", config_.confidence.min_density_near, 50.0);
  private_nh_.param<double>("confidence/min_density_far", config_.confidence.min_density_far, 10.0);
  private_nh_.param<double>("confidence/distance_threshold", config_.confidence.distance_threshold, 5.0);
  private_nh_.param<double>("confidence/min_intensity_mean", config_.confidence.min_intensity_mean, 30.0);
  private_nh_.param<double>("confidence/weight_size", config_.confidence.weight_size, 0.3);
  private_nh_.param<double>("confidence/weight_shape", config_.confidence.weight_shape, 0.25);
  private_nh_.param<double>("confidence/weight_density", config_.confidence.weight_density, 0.2);
  private_nh_.param<double>("confidence/weight_intensity", config_.confidence.weight_intensity, 0.15);
  private_nh_.param<double>("confidence/weight_position", config_.confidence.weight_position, 0.1);
  private_nh_.param<bool>("confidence/enable_model_fitting", config_.confidence.enable_model_fitting, true);
  private_nh_.param<double>("confidence/model_fit_bonus", config_.confidence.model_fit_bonus, 0.2);
  private_nh_.param<double>("confidence/model_fit_penalty", config_.confidence.model_fit_penalty, 0.15);
  // 距离自适应置信度门槛
  private_nh_.param<double>("confidence/min_confidence_near", config_.confidence.min_confidence_near, 0.0);
  private_nh_.param<double>("confidence/min_confidence_far", config_.confidence.min_confidence_far, 0.3);
  private_nh_.param<double>("confidence/confidence_ramp_start", config_.confidence.confidence_ramp_start, 10.0);
  private_nh_.param<double>("confidence/confidence_ramp_end", config_.confidence.confidence_ramp_end, 30.0);

  // Track semantic confidence (neighbor-context scoring)
  private_nh_.param<bool>("confidence/track_semantic/enable", config_.confidence.track_semantic.enable, false);
  private_nh_.param<double>("confidence/track_semantic/weight", config_.confidence.track_semantic.weight, 0.0);
  private_nh_.param<double>("confidence/track_semantic/expected_track_width", config_.confidence.track_semantic.expected_track_width, 3.0);
  private_nh_.param<double>("confidence/track_semantic/expected_cone_spacing", config_.confidence.track_semantic.expected_cone_spacing, 5.0);
  private_nh_.param<double>("confidence/track_semantic/spacing_tolerance", config_.confidence.track_semantic.spacing_tolerance, 2.0);
  private_nh_.param<double>("confidence/track_semantic/width_tolerance", config_.confidence.track_semantic.width_tolerance, 1.0);
  private_nh_.param<double>("confidence/track_semantic/isolation_radius", config_.confidence.track_semantic.isolation_radius, 8.0);
  // Hard rejection thresholds for neighborhood filtering
  private_nh_.param<int>("confidence/track_semantic/min_neighbors_hard", config_.confidence.track_semantic.min_neighbors_hard, 0);
  private_nh_.param<double>("confidence/track_semantic/max_isolation_distance", config_.confidence.track_semantic.max_isolation_distance, 12.0);

  // 锥桶类型参数（当前仅根据几何尺寸区分小橙桶/大橙桶）
  private_nh_.param<bool>("cone_size_typing/enable", enable_cone_size_typing_, true);
  private_nh_.param<double>("cone_size_typing/big_height_threshold", big_cone_height_threshold_, 0.45);
  private_nh_.param<double>("cone_size_typing/big_area_threshold", big_cone_area_threshold_, 0.09);

  // ---- Cone Tracker 参数加载 ----
  private_nh_.param<bool>("tracker/enable", config_.tracker.enable, false);
  private_nh_.param<double>("tracker/association_threshold", config_.tracker.association_threshold, 0.5);
  private_nh_.param<int>("tracker/confirm_frames", config_.tracker.confirm_frames, 3);
  private_nh_.param<int>("tracker/delete_frames", config_.tracker.delete_frames, 5);
  private_nh_.param<double>("tracker/process_noise", config_.tracker.process_noise, 0.1);
  private_nh_.param<double>("tracker/measurement_noise", config_.tracker.measurement_noise, 0.05);
  private_nh_.param<bool>("tracker/only_output_confirmed", config_.tracker.only_output_confirmed, true);
  private_nh_.param<double>("tracker/confirmed_confidence_boost", config_.tracker.confirmed_confidence_boost, 0.1);

  // ---- Topology Repair 参数加载 ----
  private_nh_.param<bool>("topology/enable", config_.topology.enable, false);
  private_nh_.param<double>("topology/max_same_side_spacing", config_.topology.max_same_side_spacing, 5.0);
  private_nh_.param<double>("topology/min_track_width", config_.topology.min_track_width, 2.5);
  private_nh_.param<double>("topology/max_track_width", config_.topology.max_track_width, 4.0);
  private_nh_.param<double>("topology/max_repair_range", config_.topology.max_repair_range, 15.0);
  private_nh_.param<double>("topology/outlier_lateral_threshold", config_.topology.outlier_lateral_threshold, 5.0);
  private_nh_.param<double>("topology/interpolated_confidence", config_.topology.interpolated_confidence, 0.2);

  // ---- Proximity Deduplication 参数加载 ----
  private_nh_.param<bool>("dedup/enable", config_.dedup.enable, true);
  private_nh_.param<double>("dedup/radius", config_.dedup.radius, 0.5);

  // ---- Stacked Cone Detection 参数加载 ----
  private_nh_.param<bool>("stacked_cone_detection/enabled", config_.dedup.stacked_enable, false);
  private_nh_.param<double>("stacked_cone_detection/vertical_layers/layer_height", config_.dedup.layer_height, 0.25);
  private_nh_.param<int>("stacked_cone_detection/vertical_layers/max_layers", config_.dedup.max_layers, 3);
  private_nh_.param<double>("stacked_cone_detection/identification/max_xy_distance", config_.dedup.stacked_xy_threshold, 0.4);
  private_nh_.param<double>("stacked_cone_detection/identification/height_variation_threshold", config_.dedup.z_height_threshold, 0.3);

  // ---- 距离自适应Y轴ROI参数 ----
  private_nh_.param<bool>("roi/adaptive_y/enable", config_.roi.adaptive_y.enable, false);
  private_nh_.param<double>("roi/adaptive_y/near_y_half", config_.roi.adaptive_y.near_y_half, 5.0);
  private_nh_.param<double>("roi/adaptive_y/far_y_half", config_.roi.adaptive_y.far_y_half, 2.0);
  private_nh_.param<double>("roi/adaptive_y/ramp_start_x", config_.roi.adaptive_y.ramp_start_x, 5.0);

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
  ROS_INFO("filters/sor: %s (mean_k=%d, stddev_mul=%f)",
           config_.filters.sor.enable ? "on" : "off",
           config_.filters.sor.mean_k,
           config_.filters.sor.stddev_mul);
  ROS_INFO("filters/voxel: %s (leaf=%f)",
           config_.filters.voxel.enable ? "on" : "off",
           config_.filters.voxel.leaf_size);
  ROS_INFO("filters/adaptive_voxel: %s (leaf=%f, density_thr=%d)",
           config_.filters.adaptive_voxel.enable ? "on" : "off",
           config_.filters.adaptive_voxel.leaf_size,
           config_.filters.adaptive_voxel.density_thr);
  ROS_INFO("filters/distance_adaptive_voxel: %s (near_leaf=%f, far_leaf=%f, dist_thr=%f)",
           config_.filters.distance_adaptive_voxel.enable ? "on" : "off",
           config_.filters.distance_adaptive_voxel.near_leaf,
           config_.filters.distance_adaptive_voxel.far_leaf,
           config_.filters.distance_adaptive_voxel.dist_threshold);
  ROS_INFO("filters/intensity: %s (min=%f)",
           config_.filters.intensity.enable ? "on" : "off",
           config_.filters.intensity.min_intensity);
  ROS_INFO("filters/use_cropbox: %s",
           config_.filters.use_cropbox ? "on" : "off");
  ROS_INFO("filters/obstacle_height: %s (grid=%.2f, max_z_span=%.2f, min_pts=%d, min_dist=%.1f)",
           config_.filters.obstacle_height.enable ? "on" : "off",
           config_.filters.obstacle_height.grid_size,
           config_.filters.obstacle_height.max_z_span,
           config_.filters.obstacle_height.min_points_to_judge,
           config_.filters.obstacle_height.min_distance);
  ROS_INFO("cluster/method: %s", config_.cluster.method.c_str());
  ROS_INFO("force_fgs_fast_path: %s", force_fgs_fast_path_ ? "on" : "off");
  ROS_INFO("ground_watchdog: %s (warn_ms=%.3f, consecutive=%d)",
           ground_watchdog_enable_ ? "on" : "off",
           ground_watchdog_warn_ms_,
           ground_watchdog_warn_frames_);
  ROS_INFO("cluster/adaptive_size: %s (near: %d-%d, far: %d-%d)",
           config_.cluster.adaptive_size.enable ? "on" : "off",
           config_.cluster.adaptive_size.near_min_size,
           config_.cluster.adaptive_size.near_max_size,
           config_.cluster.adaptive_size.far_min_size,
           config_.cluster.adaptive_size.far_max_size);
  ROS_INFO("cluster/fixed_size: min=%d, max=%d",
           config_.cluster.min_cluster_size,
           config_.cluster.max_cluster_size);
  ROS_INFO("cluster/fec: %s (quality=%.2f)",
           config_.cluster.fec.enable ? "on" : "off",
           config_.cluster.fec.quality);
  ROS_INFO("cluster/multi_frame: %s (num_frames=%d, max_distance=%.1f)",
           config_.cluster.multi_frame.enable ? "on" : "off",
           config_.cluster.multi_frame.num_frames,
           config_.cluster.multi_frame.max_distance);
  ROS_INFO("confidence: weights(size=%.2f, shape=%.2f, density=%.2f, intensity=%.2f, position=%.2f)",
           config_.confidence.weight_size, config_.confidence.weight_shape,
           config_.confidence.weight_density, config_.confidence.weight_intensity,
           config_.confidence.weight_position);
  ROS_INFO("confidence: ramp(near=%.2f, far=%.2f, start=%.1fm, end=%.1fm)",
           config_.confidence.min_confidence_near, config_.confidence.min_confidence_far,
           config_.confidence.confidence_ramp_start, config_.confidence.confidence_ramp_end);
  ROS_INFO("roi/adaptive_y: %s (near_y_half=%.1f, far_y_half=%.1f, ramp_start_x=%.1f)",
           config_.roi.adaptive_y.enable ? "on" : "off",
           config_.roi.adaptive_y.near_y_half,
           config_.roi.adaptive_y.far_y_half,
           config_.roi.adaptive_y.ramp_start_x);
  ROS_INFO("cone_size_typing: %s (big_height_thr=%.2f, big_area_thr=%.3f)",
           enable_cone_size_typing_ ? "on" : "off",
           big_cone_height_threshold_,
           big_cone_area_threshold_);
  if (config_.cluster.method == "dbscan")
  {
    ROS_INFO("cluster/dbscan: eps=%f, min_pts=%d, adaptive_eps=%s (near=%f, far=%f)",
             config_.cluster.dbscan.eps,
             config_.cluster.dbscan.min_pts,
             config_.cluster.dbscan.adaptive_eps ? "on" : "off",
             config_.cluster.dbscan.eps_near,
             config_.cluster.dbscan.eps_far);
  }

  if (!private_nh_.param<bool>("perf_stats_enable", perf_enabled_, true))
  {
    ROS_WARN_STREAM("Did not load perf_stats_enable. Standard value is: " << perf_enabled_);
  }
  int perf_window = static_cast<int>(perf_window_);
  if (!private_nh_.param<int>("perf_stats_window", perf_window, 300))
  {
    ROS_WARN_STREAM("Did not load perf_stats_window. Standard value is: " << perf_window);
  }
  perf_window_ = static_cast<size_t>(perf_window);
  int perf_log_every = static_cast<int>(perf_log_every_);
  if (!private_nh_.param<int>("perf_stats_log_every", perf_log_every, 30))
  {
    ROS_WARN_STREAM("Did not load perf_stats_log_every. Standard value is: " << perf_log_every);
  }
  perf_log_every_ = static_cast<size_t>(perf_log_every);
  perf_stats_.Configure("lidar_cluster", perf_enabled_, perf_window_, perf_log_every_);

  sensor_model_ = config_.sensor_model;
  ROS_INFO_STREAM("pipeline_mode: " << pipeline_mode_ << ", legacy_poll_hz: " << legacy_poll_hz_);
}

void LidarClusterRos::applyModePreset()
{
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
  private_nh_.param<bool>(prefix + "/adaptive_y/enable",
                          config_.roi.adaptive_y.enable,
                          config_.roi.adaptive_y.enable);
  if (config_.roi.adaptive_y.enable) {
    private_nh_.param<double>(prefix + "/adaptive_y/near_y_half",
                              config_.roi.adaptive_y.near_y_half,
                              config_.roi.adaptive_y.near_y_half);
    private_nh_.param<double>(prefix + "/adaptive_y/far_y_half",
                              config_.roi.adaptive_y.far_y_half,
                              config_.roi.adaptive_y.far_y_half);
    private_nh_.param<double>(prefix + "/adaptive_y/ramp_start_x",
                              config_.roi.adaptive_y.ramp_start_x,
                              config_.roi.adaptive_y.ramp_start_x);
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
}

void LidarClusterRos::pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
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

  std::lock_guard<std::mutex> lock(seq_mutex_);
  last_header_ = msg->header;
  last_seq_ = msg->header.seq;
  got_cloud_ = true;
  // 零拷贝：转移点云所有权给core，避免深拷贝
  core_.SetInputCloud(std::move(cloud), msg->header.seq);

  // 事件驱动路径：收到新帧立即处理
  if (!IsLegacyPollMode())
  {
    runOnceLocked();
  }
}

void LidarClusterRos::RunOnce()
{
  std::lock_guard<std::mutex> lock(seq_mutex_);
  runOnceLocked();
}

void LidarClusterRos::runOnceLocked()
{
  if (!got_cloud_)
  {
    ROS_WARN_THROTTLE(5.0, "Not received Point Cloud!");
    return;
  }

  // 新帧闸门 - 仅处理新帧
  if (last_seq_ == last_processed_seq_)
  {
    return;  // 已处理过该帧，跳过
  }

  // 超时检查 - 丢弃过旧的点云
  // 使用帧间隔判断：如果当前帧时间戳比上一帧还旧，说明数据有问题
  // 这样可以同时支持实时运行和 rosbag 回放
  if (!last_cloud_stamp_.isZero() && last_header_.stamp < last_cloud_stamp_)
  {
    // 时间戳回退，可能是 rosbag 重新开始，重置状态
    ROS_INFO("Timestamp reset detected, reinitializing...");
    last_cloud_stamp_ = last_header_.stamp;
  }
  
  // 计算帧间隔（与上一处理帧的时间差）
  double frame_interval = last_cloud_stamp_.isZero() ? 0.0 : (last_header_.stamp - last_cloud_stamp_).toSec();
  if (frame_interval > max_cloud_age_ && frame_interval < 10.0)  // 10秒内的异常间隔
  {
    ROS_WARN_THROTTLE(1.0, "Large frame interval (%.2fs), possible frame drop", frame_interval);
  }
  
  last_cloud_stamp_ = last_header_.stamp;

  if (!core_.Process(&output_))
  {
    return;
  }

  updateGroundWatchdogLocked(output_.t_ground_ms);
  last_processed_seq_ = last_seq_;
  publishOutput(output_);
}

void LidarClusterRos::updateGroundWatchdogLocked(double t_ground_ms)
{
  if (!ground_watchdog_enable_)
  {
    return;
  }

  if (t_ground_ms > ground_watchdog_warn_ms_)
  {
    ++ground_watchdog_overrun_count_;
  }
  else
  {
    ground_watchdog_overrun_count_ = 0;
    return;
  }

  if (ground_watchdog_overrun_count_ < ground_watchdog_warn_frames_)
  {
    return;
  }

  ROS_WARN_THROTTLE(1.0,
                    "Ground watchdog triggered: t_ground_ms=%.3f exceeds %.3f for %d consecutive frames",
                    t_ground_ms,
                    ground_watchdog_warn_ms_,
                    ground_watchdog_warn_frames_);
  ground_watchdog_overrun_count_ = 0;
}

void LidarClusterRos::publishOutput(const LidarClusterOutput &output)
{
  size_t bytes_pub = 0;

  // 复用消息缓冲区，避免每帧重新分配
  if (output.passthrough)
  {
    pcl::toROSMsg(*output.passthrough, pub_pc_msg_);
    pub_pc_msg_.header = last_header_;
    passthrough_pub_.publish(pub_pc_msg_);
    bytes_pub += pub_pc_msg_.data.size();
  }

  if (output.not_ground)
  {
    pcl::toROSMsg(*output.not_ground, pub_pc_msg_);
    pub_pc_msg_.header = last_header_;
    no_ground_pub_.publish(pub_pc_msg_);
    bytes_pub += pub_pc_msg_.data.size();
  }

  if (output.cones_cloud)
  {
    pcl::toROSMsg(*output.cones_cloud, pub_cones_msg_);
    pub_cones_msg_.header = last_header_;
    cones_pub_.publish(pub_cones_msg_);
    bytes_pub += pub_cones_msg_.data.size();
  }

  autodrive_msgs::HUAT_ConeDetections detections;
  detections.header = last_header_;
  if (output.cones_cloud)
  {
    detections.pc_whole = pub_cones_msg_;
  }

  for (const auto &det : output.cones)
  {
    // Defense-in-depth: skip detections with non-finite centroids
    if (!std::isfinite(det.centroid.x) || !std::isfinite(det.centroid.y) || !std::isfinite(det.centroid.z))
    {
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
    if (sensor_model_ != 16)
    {
      detections.confidence.push_back(static_cast<float>(det.confidence));
    }
    else
    {
      detections.confidence.push_back(0.0f);  // VLP-16 填充默认值
    }
    detections.obj_dist.push_back(static_cast<float>(det.distance));
    detections.color_types.push_back(
        classifyConeTypeBySize(det,
                               enable_cone_size_typing_,
                               big_cone_height_threshold_,
                               big_cone_area_threshold_));

    if (det.cluster)
    {
      sensor_msgs::PointCloud2 cluster_msg;
      pcl::toROSMsg(*det.cluster, cluster_msg);
      cluster_msg.header = last_header_;
      detections.pc.push_back(cluster_msg);
    }
  }

  detections_pub_.publish(detections);
  bytes_pub += ros::serialization::serializationLength(detections);

  PerfSample sample;
  sample.t_pass_ms = output.t_pass_ms;
  sample.t_ground_ms = output.t_ground_ms;
  sample.t_cluster_ms = output.t_cluster_ms;
  sample.t_total_ms = output.t_total_ms;
  sample.n_points = static_cast<double>(output.input_points);
  sample.n_clusters = static_cast<double>(output.total_clusters);
  sample.bytes_pub = static_cast<double>(bytes_pub);
  perf_stats_.Add(sample);
}

}  // namespace perception_ros
