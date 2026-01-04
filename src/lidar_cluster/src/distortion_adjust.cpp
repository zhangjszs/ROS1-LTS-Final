/*
 * @Description: 点云畸变补偿
 * @Author: Jiaxi Dai
 * @Date: 2021年11月29日14:52:37
 */
#include <distortion_adjust.hpp>
// #include "glog/logging.h"

using namespace lidar_distortion;
DistortionAdjust::DistortionAdjust()
{
}

void DistortionAdjust::SetMotionInfo(float scan_period, IMUData velocity_data)
{
    scan_period_ = scan_period;
    // ROS_INFO("?????");
    // std::cout<<velocity_data.linear_acceleration.x<<std::endl;
    velocity_ << velocity_data.linear_acceleration.x, -velocity_data.linear_acceleration.y, -velocity_data.linear_acceleration.z;
    // ROS_INFO("?????");
    // angular_rate_ << velocity_data.angular_velocity.x * M_PI / 180.0, -velocity_data.angular_velocity.y* M_PI / 180.0, -velocity_data.angular_velocity.z* M_PI / 180.0;
    angular_rate_ << velocity_data.rpy.heading, -velocity_data.rpy.pitch, 0;
    // std::cout<< "ang:"<<angular_rate_<<std::endl;
    // std::cout<<"y:"<<velocity_data.angular_velocity.y<<std::endl;
    // head_ << velocity_data.rpy.heading, velocity_data.rpy.pitch;
    // head_ = velocity_data.rpy.heading;
    // pitch_ =velocity_data.rpy.pitch;

    // std::cout<<"head_"<<std::setprecision(20)<<head_<<std::endl;
    // std::cout<<"pitch_"<<std::setprecision(20)<<pitch_<<std::endl;
    // std::cout<<rpy_<<std::endl;
    // std::cout<<angular_rate_<<std::endl;
    // ROS_INFO("?????");
}

void DistortionAdjust::AdjustCloud(pcl::PointCloud<PointType>::Ptr &input_cloud_ptr, pcl::PointCloud<PointType>::Ptr &output_cloud_ptr)
{
    pcl::PointCloud<PointType>::Ptr origin_cloud_ptr = input_cloud_ptr;
    // origin_cloud_ptr = input_cloud_ptr;
    output_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    // ROS_INFO("?????");
    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);
    // std::cout<<"y:"<<origin_cloud_ptr->points[0].y<<" x: "<<origin_cloud_ptr->points[0].x<<std::endl;
    // ROS_INFO("?????");
    // std::cout<<"start_orientation"<<start_orientation<<std::endl;
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);
    // output_cloud_ptr = origin_cloud_ptr;
    // ROS_INFO("?????");
    // std::cout<<"velocity:"<<velocity_<<std::endl;
    // std::cout<<"angular_rate_:"<<angular_rate_<<std::endl;
    // exit(0);
    angular_rate_ =
        velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index)
    {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);

        // std::cout<<orientation<<std::endl;
        if (orientation < 0.0)
            orientation += 2.0 * M_PI;

        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;
        // 0~1
        //  std::cout<<orientation<<std::endl;
        //  exit(-1);
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;
        // float real_time = fabs(orientation) / orientation_space * scan_period_ ;
        // std::cout<<"real_time"<<real_time<<std::endl;
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

// Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {

//     Eigen::Vector3f angle = angular_rate_ * real_time;
//     // float Uz= angle(2)  / head_;
//     // float Uy = angle(1) / pitch_ ;
//     // std::cout<<"real time:"<<real_time<<std::endl;
//     // std::cout<<"angular "<<angle<<std::endl;
//     // std::cout<<"rpy "<< rpy_<<std::endl;
//     Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
//     // Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
//     // Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf t_V;
//     // t_V = t_Vz * t_Vy * t_Vx;
//     return t_Vz.matrix();
// }
Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time)
{
    // std::cout<<"head_"<<head_<<std::endl;
    // std::cout<<"pitch_"<<pitch_<<std::endl;
    Eigen::Vector3f angle = angular_rate_ * real_time;
    // float Uz = head_  * real_time * M_PI / 180.0;
    // float Uy = - pitch_ * real_time * M_PI / 180.0;
    // float Ux = 0;
    // std::cout<<"angle0"<<angle(0)<<std::endl;
    Eigen::AngleAxisf t_Vz(angle(0), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitX());
    // Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy;
    return t_V.matrix();
}
// Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {

//     Eigen::Vector3f angle = angular_rate_ * real_time;
//     // std::cout<<"head_"<<head_<<"pitch_"<<pitch_<<std::endl;
//     // float Uz = angle(2)  / head_;
//     // float Uy = angle(1) / pitch_ ;
//     // std::cout<<"Uz: "<<Uz<<", "<<"Uy:"<<Uy<<std::endl;
//     // std::cout<<"real time:"<<real_time<<std::endl;
//     // std::cout<<"angular "<<angle<<std::endl;
//     // std::cout<<"rpy "<< rpy_<<std::endl;
//     Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
//     // Eigen::AngleAxisf t_Vy(Uy, Eigen::Vector3f::UnitY());
//     // Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf t_V;
//     t_V = t_Vz ;
//     return t_V.matrix();
// }
// end space