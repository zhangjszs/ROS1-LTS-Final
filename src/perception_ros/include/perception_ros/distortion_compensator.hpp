#pragma once

#include <memory>
#include <deque>
#include <string>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <perception_ros/imu_subscriber.hpp>
#include <perception_core/distortion_adjust.hpp>
#include <perception_core/imu_data.hpp>

namespace perception_ros {

typedef pcl::PointXYZI PointType;

/**
 * @brief 畸变补偿器配置
 */
struct DistortionCompensatorConfig {
  bool enable = false;                    // 是否启用畸变补偿
  std::string imu_topic = "/pbox_pub/Ins"; // IMU 话题
  int buffer_size = 200;                   // IMU 缓冲区大小
  float scan_period = 0.1f;                // 激光扫描周期 (秒)
  bool log_imu_data = false;               // 是否打印 IMU 数据
  bool log_compensation = false;           // 是否打印补偿信息
};

/**
 * @brief 点云畸变补偿器
 * 
 * 封装 IMU 订阅和畸变补偿逻辑，提供简洁的接口给点云处理节点使用。
 * 
 * 使用示例:
 * @code
 *   auto config = DistortionCompensator::LoadConfig(private_nh);
 *   if (config.enable) {
 *     compensator = std::make_unique<DistortionCompensator>(nh, config);
 *   }
 *   
 *   // 在点云回调中
 *   if (compensator) {
 *     compensator->Compensate(cloud, cloud_time);
 *   }
 * @endcode
 */
class DistortionCompensator {
 public:
  /**
   * @brief 构造函数
   * @param nh ROS 节点句柄
   * @param config 配置参数
   */
  DistortionCompensator(ros::NodeHandle& nh, const DistortionCompensatorConfig& config);
  
  /**
   * @brief 对点云进行畸变补偿
   * @param cloud 输入/输出点云 (in-place 修改)
   * @param cloud_time 点云时间戳 (秒)
   * @return true 补偿成功，false 补偿失败（IMU 同步失败等）
   */
  bool Compensate(pcl::PointCloud<PointType>::Ptr& cloud, double cloud_time);
  
  /**
   * @brief 从 ROS 参数服务器读取配置
   * @param nh 私有节点句柄 (通常是 private_nh)
   * @return 配置结构体
   */
  static DistortionCompensatorConfig LoadConfig(ros::NodeHandle& nh);
  
  /**
   * @brief 检查是否启用畸变补偿
   */
  bool IsEnabled() const { return config_.enable; }
  
  /**
   * @brief 获取配置
   */
  const DistortionCompensatorConfig& GetConfig() const { return config_; }

 private:
  DistortionCompensatorConfig config_;
  std::unique_ptr<IMUSubscriber> imu_sub_;
  std::unique_ptr<lidar_distortion::DistortionAdjust> adjuster_;
  std::deque<IMUData> imu_buffer_;
};

}  // namespace perception_ros
