/*
 * @Description: 点云畸变补偿
 * @Author: Jiaxi Dai
 * @Date: 2020-02-25 14:38:12
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <perception_core/imu_data.hpp>
typedef pcl::PointXYZI PointType;
// #include "perception_core/distortion_adjust.hpp"
// #include "lidar_localization/sensor_data/velocity_data.hpp"
// #include "lidar_localization/sensor_data/cloud_data.hpp"
namespace lidar_distortion {


class DistortionAdjust {
  public:
    DistortionAdjust();
    void SetMotionInfo(float scan_period, IMUData velocity_data);
    // bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);
    void AdjustCloud(pcl::PointCloud<PointType>::Ptr& input_cloud_ptr, pcl::PointCloud<PointType>::Ptr& output_cloud_ptr);
    // std::shared_ptr<DistortionAdjust> distadjust;
  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);
  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
    float head_;
    float pitch_;
};

} // namespace lidar_distortion
#endif
