/*
 * @Description: 点云畸变补偿
 * @Author: Jiaxi Dai
 * @Date: 2020-02-25 14:38:12
 * @Modified: 2025 - 同步2024huat加速度补偿功能
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <perception_core/imu_data.hpp>
typedef pcl::PointXYZI PointType;

namespace lidar_distortion {

/**
 * @brief 畸变补偿模式枚举
 */
enum class CompensationMode {
  VELOCITY_ONLY,    ///< 仅速度补偿
  VELOCITY_ACCEL    ///< 速度+加速度补偿
};

class DistortionAdjust {
  public:
    DistortionAdjust();
    
    /**
     * @brief 设置补偿模式
     * @param mode 补偿模式
     */
    void SetMode(CompensationMode mode) { mode_ = mode; }
    
    /**
     * @brief 获取当前补偿模式
     */
    CompensationMode GetMode() const { return mode_; }
    
    /**
     * @brief 设置运动信息（仅速度）
     * @param scan_period 激光扫描周期（秒）
     * @param velocity_data IMU数据，包含速度信息
     */
    void SetMotionInfo(float scan_period, const IMUData& velocity_data);
    
    /**
     * @brief 设置运动信息（速度+加速度）
     * [2024huat同步] 从旧项目移植的接口
     * @param scan_period 激光扫描周期（秒）
     * @param velocity_data IMU数据，包含速度信息
     * @param acceleration_data IMU数据，包含加速度信息
     */
    void SetMotionInfo(float scan_period, const IMUData& velocity_data, const IMUData& acceleration_data);
    
    /**
     * @brief 执行点云畸变补偿
     * @param input_cloud_ptr 输入点云
     * @param output_cloud_ptr 输出点云
     */
    void AdjustCloud(pcl::PointCloud<PointType>::Ptr& input_cloud_ptr, 
                     pcl::PointCloud<PointType>::Ptr& output_cloud_ptr);
                     
  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time, const Eigen::Vector3f& angular_rate);
    
  private:
    float scan_period_ = 0.1f;
    Eigen::Vector3f velocity_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f angular_rate_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f acceleration_ = Eigen::Vector3f::Zero();  ///< [2024huat同步] 加速度
    CompensationMode mode_ = CompensationMode::VELOCITY_ONLY;
    float head_ = 0.0f;
    float pitch_ = 0.0f;
};

} // namespace lidar_distortion
#endif
