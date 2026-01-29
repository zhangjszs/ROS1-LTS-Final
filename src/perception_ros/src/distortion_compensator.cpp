/**
 * @file distortion_compensator.cpp
 * @brief 点云畸变补偿器实现
 * 
 * 封装 IMU 订阅和畸变补偿逻辑，将 IMU 相关代码从 lidar_cluster_ros 解耦。
 */

#include <perception_ros/distortion_compensator.hpp>

namespace perception_ros {

DistortionCompensator::DistortionCompensator(ros::NodeHandle& nh, 
                                             const DistortionCompensatorConfig& config)
    : config_(config)
{
  if (!config_.enable) {
    ROS_WARN("[DistortionCompensator] Disabled by config");
    return;
  }
  
  // 初始化 IMU 订阅器
  imu_sub_ = std::make_unique<IMUSubscriber>(nh, config_.imu_topic, 
                                              static_cast<size_t>(config_.buffer_size));
  ROS_INFO("[DistortionCompensator] IMU subscriber on: %s", config_.imu_topic.c_str());
  
  // 初始化畸变补偿器
  adjuster_ = std::make_unique<lidar_distortion::DistortionAdjust>();
  ROS_INFO("[DistortionCompensator] Initialized (scan_period: %.3f s)", config_.scan_period);
}

bool DistortionCompensator::Compensate(pcl::PointCloud<PointType>::Ptr& cloud, 
                                        double cloud_time)
{
  if (!config_.enable || !imu_sub_ || !adjuster_) {
    return false;
  }
  
  // 解析 IMU 数据
  imu_sub_->ParseData(imu_buffer_);
  
  // 时间同步
  IMUData synced_imu;
  if (!imu_sub_->SyncData(imu_buffer_, synced_imu, cloud_time)) {
    // 调试信息
    if (!imu_buffer_.empty()) {
      double imu_front = imu_buffer_.front().time;
      double imu_back = imu_buffer_.back().time;
      ROS_WARN_THROTTLE(2.0, "[DistortionCompensator] Sync failed: cloud=%.3f, imu=[%.3f, %.3f], size=%lu",
                        cloud_time, imu_front, imu_back, imu_buffer_.size());
    } else {
      ROS_WARN_THROTTLE(5.0, "[DistortionCompensator] IMU buffer empty, cloud=%.3f", cloud_time);
    }
    return false;
  }
  
  // 打印 IMU 数据
  if (config_.log_imu_data) {
    ROS_INFO_THROTTLE(1.0, "[IMU] vn=%.2f ve=%.2f vd=%.2f wx=%.4f wy=%.4f wz=%.4f",
                      synced_imu.velocity.vn, synced_imu.velocity.ve, synced_imu.velocity.vd,
                      synced_imu.angular_velocity.wx, synced_imu.angular_velocity.wy, 
                      synced_imu.angular_velocity.wz);
  }
  
  // 设置运动信息
  adjuster_->SetMotionInfo(config_.scan_period, synced_imu);
  
  // 应用畸变补偿
  pcl::PointCloud<PointType>::Ptr cloud_compensated(new pcl::PointCloud<PointType>);
  adjuster_->AdjustCloud(cloud, cloud_compensated);
  
  // 打印补偿信息
  if (config_.log_compensation) {
    ROS_INFO_THROTTLE(2.0, "[Compensation] %lu -> %lu points",
                      cloud->points.size(), cloud_compensated->points.size());
  }
  
  // 替换原点云
  cloud = cloud_compensated;
  return true;
}

DistortionCompensatorConfig DistortionCompensator::LoadConfig(ros::NodeHandle& nh)
{
  DistortionCompensatorConfig config;
  
  nh.param<bool>("imu/enable", config.enable, false);
  nh.param<std::string>("imu/topic", config.imu_topic, "/pbox_pub/Ins");
  nh.param<int>("imu/buffer_size", config.buffer_size, 200);
  nh.param<float>("imu/distortion/scan_period", config.scan_period, 0.1f);
  nh.param<bool>("imu/distortion/enable", config.enable, config.enable);  // 兼容旧配置
  nh.param<bool>("imu/debug/log_imu_data", config.log_imu_data, false);
  nh.param<bool>("imu/debug/log_compensation", config.log_compensation, false);
  
  ROS_INFO("[DistortionCompensator] Config: enable=%s, topic=%s, period=%.3f",
           config.enable ? "true" : "false", config.imu_topic.c_str(), config.scan_period);
  
  return config;
}

}  // namespace perception_ros
