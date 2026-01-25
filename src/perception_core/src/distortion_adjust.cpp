/*
 * @Description: 点云畸变补偿
 * @Author: Jiaxi Dai
 * @Date: 2021年11月29日14:52:37
 */
#include <perception_core/distortion_adjust.hpp>

using namespace lidar_distortion;
DistortionAdjust::DistortionAdjust()
{
}

void DistortionAdjust::SetMotionInfo(float scan_period, IMUData velocity_data)
{
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_acceleration.x, -velocity_data.linear_acceleration.y, -velocity_data.linear_acceleration.z;
    angular_rate_ << velocity_data.rpy.heading, -velocity_data.rpy.pitch, 0;
}

void DistortionAdjust::AdjustCloud(pcl::PointCloud<PointType>::Ptr &input_cloud_ptr, pcl::PointCloud<PointType>::Ptr &output_cloud_ptr)
{
    pcl::PointCloud<PointType>::Ptr origin_cloud_ptr = input_cloud_ptr;
    output_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);
    angular_rate_ =
        velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index)
    {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);

        if (orientation < 0.0)
            orientation += 2.0 * M_PI;

        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;
        // 0~1
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;
        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
        Eigen::Vector3f rotated_point = current_matrix * origin_point;
        Eigen::Vector3f adjusted_point = origin_point + velocity_ * real_time;
        PointType point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return;
}
Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time)
{
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(0), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy;
    return t_V.matrix();
}
