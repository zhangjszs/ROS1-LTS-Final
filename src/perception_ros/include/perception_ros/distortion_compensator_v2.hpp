/**
 * @file distortion_compensator_v2.hpp
 * @brief ROS集成的畸变补偿器V2
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <perception_core/distortion_adjust_v2.hpp>
#include <autodrive_msgs/HUAT_InsP2.h>
#include <deque>
#include <mutex>

namespace perception_ros {

/**
 * @brief ROS集成的畸变补偿器V2配置
 */
struct DistortionCompensatorV2Config {
    bool enable = false;

    // IMU话题
    std::string imu_topic = "/pbox_pub/Ins";
    int imu_buffer_size = 200;

    // 扫描参数
    double scan_period = 0.1;

    // 补偿模式
    std::string mode = "velocity_accel";  // velocity_only, velocity_accel, full_6dof, preintegration

    // 点云时间源
    std::string point_time_source = "auto";  // auto, point_time, angle

    // 参考时刻
    std::string ref_time = "scan_end";  // scan_start, scan_middle, scan_end

    // IMU到LiDAR外参
    bool extrinsics_enable = false;
    double extrinsics_tx = 0.0;
    double extrinsics_ty = 0.0;
    double extrinsics_tz = 0.0;
    double extrinsics_roll = 0.0;
    double extrinsics_pitch = 0.0;
    double extrinsics_yaw = 0.0;

    // 调试
    bool debug_output = false;
    bool publish_debug_info = false;

    // 从ROS参数加载
    static DistortionCompensatorV2Config LoadFromRos(ros::NodeHandle& nh);
};

/**
 * @brief ROS集成的畸变补偿器V2
 */
class DistortionCompensatorV2 {
public:
    using PointType = pcl::PointXYZI;
    using CloudPtr = pcl::PointCloud<PointType>::Ptr;

    DistortionCompensatorV2(ros::NodeHandle& nh, const DistortionCompensatorV2Config& config);

    /**
     * @brief 补偿点云
     * @param cloud 输入/输出点云
     * @param timestamp 点云时间戳
     * @return 是否成功
     */
    bool Compensate(CloudPtr& cloud, double timestamp);

    /**
     * @brief 补偿点云（从ROS消息）
     * @param msg 输入点云消息
     * @param cloud 输出点云
     * @return 是否成功
     */
    bool CompensateFromMsg(const sensor_msgs::PointCloud2& msg, CloudPtr& cloud);

    /**
     * @brief 获取调试信息
     */
    const lidar_distortion::DistortionAdjustV2::DebugInfo& GetDebugInfo() const {
        return compensator_.GetDebugInfo();
    }

    /**
     * @brief 是否启用
     */
    bool IsEnabled() const { return config_.enable; }

    /**
     * @brief G10: 获取最新自车速度（LiDAR坐标系: x前 y左 z上）
     * @param[out] vx 前向速度 [m/s]
     * @param[out] vy 左向速度 [m/s]
     * @param[out] yaw_rate 航向角速度 [rad/s]，左转为正
     * @return 是否有有效IMU数据
     */
    bool GetLatestEgoVelocity(double &vx, double &vy, double &yaw_rate) const;

private:
    void imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr& msg);

    DistortionCompensatorV2Config config_;
    lidar_distortion::DistortionAdjustV2 compensator_;

    ros::Subscriber imu_sub_;
    ros::Publisher debug_pub_;

    std::mutex imu_mutex_;
    bool has_imu_data_ = false;
};

}  // namespace perception_ros
