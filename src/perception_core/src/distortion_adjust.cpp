/*
 * @Description: 点云畸变补偿
 * @Author: Jiaxi Dai
 * @Date: 2021年11月29日14:52:37
 * @Modified: 2026-01-29 - 修复P0问题：空云保护、强度复制、旋转生效、坐标系修正
 */
#include <perception_core/distortion_adjust.hpp>

using namespace lidar_distortion;

DistortionAdjust::DistortionAdjust()
{
}

void DistortionAdjust::SetMotionInfo(float scan_period, IMUData velocity_data)
{
    scan_period_ = scan_period;

    // 速度转换：NED导航系 -> 车体FRD系
    // 使用航向角进行旋转变换
    float heading = velocity_data.orientation.heading * M_PI / 180.0;
    float vn = velocity_data.velocity.vn;
    float ve = velocity_data.velocity.ve;
    float vd = velocity_data.velocity.vd;

    velocity_ << cos(heading) * vn + sin(heading) * ve,   // 前向
                -sin(heading) * vn + cos(heading) * ve,   // 右向
                 vd;                                       // 下向

    // 角速度已经是FRD车体系，直接使用
    angular_rate_ << velocity_data.angular_velocity.wx,
                     velocity_data.angular_velocity.wy,
                     velocity_data.angular_velocity.wz;
}

void DistortionAdjust::AdjustCloud(pcl::PointCloud<PointType>::Ptr &input_cloud_ptr, 
                                   pcl::PointCloud<PointType>::Ptr &output_cloud_ptr)
{
    // P0: 空云检查保护
    if (!input_cloud_ptr || input_cloud_ptr->points.empty()) {
        output_cloud_ptr = input_cloud_ptr;
        return;
    }
    
    pcl::PointCloud<PointType>::Ptr origin_cloud_ptr = input_cloud_ptr;
    output_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    output_cloud_ptr->points.reserve(origin_cloud_ptr->points.size());  // 预分配内存
    
    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    
    // P0: 空云检查已在上面完成，这里可以安全访问 points[0]
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);
    
    // P1修复：速度和角速度应使用inverse变换，与点云变换一致
    Eigen::Vector3f velocity_rotated = rotate_matrix.inverse() * velocity_;
    Eigen::Vector3f angular_rate_rotated = rotate_matrix.inverse() * angular_rate_;

    for (size_t point_index = 0; point_index < origin_cloud_ptr->points.size(); ++point_index)
    {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, 
                                  origin_cloud_ptr->points[point_index].x);

        if (orientation < 0.0)
            orientation += 2.0 * M_PI;

        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;
            
        // 计算该点相对于扫描起始时刻的时间偏移 (-scan_period/2 ~ +scan_period/2)
        float real_time = orientation / orientation_space * scan_period_ - scan_period_ / 2.0;
        
        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        // P0修复：补偿方向应为反向（从点采集时刻回到参考时刻）
        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time, angular_rate_rotated);
        Eigen::Vector3f rotated_point = current_matrix.inverse() * origin_point;

        // 应用平移补偿（减法：回退到参考时刻）
        Eigen::Vector3f adjusted_point = rotated_point - velocity_rotated * real_time;
        
        PointType point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        // P0: 复制 intensity 字段
        point.intensity = origin_cloud_ptr->points[point_index].intensity;
        output_cloud_ptr->points.push_back(point);
    }

    // 变换回原始坐标系
    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    
    // 设置点云元数据
    output_cloud_ptr->width = output_cloud_ptr->points.size();
    output_cloud_ptr->height = 1;
    output_cloud_ptr->is_dense = origin_cloud_ptr->is_dense;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time, const Eigen::Vector3f& angular_rate)
{
    // 计算角度增量
    Eigen::Vector3f angle = angular_rate * real_time;
    
    // 构建旋转矩阵：ZYX顺序 (yaw-pitch-roll)
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());  // yaw
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());  // pitch
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());  // roll
    
    Eigen::Matrix3f rotation_matrix = (t_Vz * t_Vy * t_Vx).matrix();
    return rotation_matrix;
}
