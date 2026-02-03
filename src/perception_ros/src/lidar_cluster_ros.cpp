#include <perception_ros/lidar_cluster_ros.hpp>

#include <ros/serialization.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utility>

namespace perception_ros {

namespace {

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

  // Initialize distortion compensator (decoupled module)
  auto compensator_config = DistortionCompensator::LoadConfig(private_nh_);
  if (compensator_config.enable) {
    compensator_ = std::make_unique<DistortionCompensator>(nh_, compensator_config);
  }

  ROS_INFO("lidar cluster finished initialization");
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
  ROS_INFO("cluster/method: %s", config_.cluster.method.c_str());
  ROS_INFO("cluster/adaptive_size: %s (near: %d-%d, far: %d-%d)",
           config_.cluster.adaptive_size.enable ? "on" : "off",
           config_.cluster.adaptive_size.near_min_size,
           config_.cluster.adaptive_size.near_max_size,
           config_.cluster.adaptive_size.far_min_size,
           config_.cluster.adaptive_size.far_max_size);
  ROS_INFO("cluster/fixed_size: min=%d, max=%d",
           config_.cluster.min_cluster_size,
           config_.cluster.max_cluster_size);
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
}

void LidarClusterRos::pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *cloud);

  // Apply distortion compensation (decoupled module)
  if (compensator_) {
    compensator_->Compensate(cloud, msg->header.stamp.toSec());
  }

  last_header_ = msg->header;
  last_seq_ = msg->header.seq;
  got_cloud_ = true;
  core_.SetInputCloud(cloud, msg->header.seq);
}

void LidarClusterRos::RunOnce()
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
  static ros::Time last_stamp;
  if (!last_stamp.isZero() && last_header_.stamp < last_stamp)
  {
    // 时间戳回退，可能是 rosbag 重新开始，重置状态
    ROS_INFO("Timestamp reset detected, reinitializing...");
    last_stamp = last_header_.stamp;
  }
  
  // 计算帧间隔（与上一处理帧的时间差）
  double frame_interval = last_stamp.isZero() ? 0.0 : (last_header_.stamp - last_stamp).toSec();
  if (frame_interval > max_cloud_age_ && frame_interval < 10.0)  // 10秒内的异常间隔
  {
    ROS_WARN_THROTTLE(1.0, "Large frame interval (%.2fs), possible frame drop", frame_interval);
  }
  
  last_stamp = last_header_.stamp;

  if (!core_.Process(&output_))
  {
    return;
  }

  last_processed_seq_ = last_seq_;
  publishOutput(output_);
}

void LidarClusterRos::publishOutput(const LidarClusterOutput &output)
{
  size_t bytes_pub = 0;

  sensor_msgs::PointCloud2 pc_msg;
  if (output.passthrough)
  {
    pcl::toROSMsg(*output.passthrough, pc_msg);
    pc_msg.header = last_header_;
    passthrough_pub_.publish(pc_msg);
    bytes_pub += pc_msg.data.size();
  }

  if (output.not_ground)
  {
    pcl::toROSMsg(*output.not_ground, pc_msg);
    pc_msg.header = last_header_;
    no_ground_pub_.publish(pc_msg);
    bytes_pub += pc_msg.data.size();
  }

  sensor_msgs::PointCloud2 cones_msg;
  if (output.cones_cloud)
  {
    pcl::toROSMsg(*output.cones_cloud, cones_msg);
    cones_msg.header = last_header_;
    cones_pub_.publish(cones_msg);
    bytes_pub += cones_msg.data.size();
  }

  autodrive_msgs::HUAT_ConeDetections detections;
  detections.header = last_header_;
  if (output.cones_cloud)
  {
    detections.pc_whole = cones_msg;
  }

  for (const auto &det : output.cones)
  {
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
