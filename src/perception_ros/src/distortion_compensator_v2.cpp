/**
 * @file distortion_compensator_v2.cpp
 * @brief ROS集成的畸变补偿器V2实现
 */

#include <perception_ros/distortion_compensator_v2.hpp>
#include <fsd_common/geometry_utils.hpp>
#include <std_msgs/String.h>
#include <sstream>

namespace perception_ros {

namespace {
// 检测点云是否有time字段
bool HasTimeField(const sensor_msgs::PointCloud2& msg) {
    for (const auto& field : msg.fields) {
        if (field.name == "time" || field.name == "t" || field.name == "timestamp") {
            return true;
        }
    }
    return false;
}
}  // namespace

DistortionCompensatorV2Config DistortionCompensatorV2Config::LoadFromRos(ros::NodeHandle& nh) {
    DistortionCompensatorV2Config config;

    nh.param<bool>("imu/enable", config.enable, false);
    nh.param<std::string>("imu/topic", config.imu_topic, "/pbox_pub/Ins");
    nh.param<int>("imu/buffer_size", config.imu_buffer_size, 200);

    nh.param<double>("imu/distortion/scan_period", config.scan_period, 0.1);
    nh.param<std::string>("imu/distortion/mode", config.mode, "velocity_accel");
    nh.param<std::string>("imu/distortion/point_time_source", config.point_time_source, "auto");
    nh.param<std::string>("imu/distortion/ref_time", config.ref_time, "scan_end");

    // 外参
    nh.param<bool>("imu/sensor/extrinsics/enable", config.extrinsics_enable, false);
    nh.param<double>("imu/sensor/extrinsics/tx", config.extrinsics_tx, 0.0);
    nh.param<double>("imu/sensor/extrinsics/ty", config.extrinsics_ty, 0.0);
    nh.param<double>("imu/sensor/extrinsics/tz", config.extrinsics_tz, 0.0);
    nh.param<double>("imu/sensor/extrinsics/roll", config.extrinsics_roll, 0.0);
    nh.param<double>("imu/sensor/extrinsics/pitch", config.extrinsics_pitch, 0.0);
    nh.param<double>("imu/sensor/extrinsics/yaw", config.extrinsics_yaw, 0.0);

    // 调试
    nh.param<bool>("imu/distortion/debug/enable", config.debug_output, false);
    nh.param<bool>("imu/distortion/debug/publish_debug_info", config.publish_debug_info, false);

    return config;
}

DistortionCompensatorV2::DistortionCompensatorV2(ros::NodeHandle& nh,
                                                   const DistortionCompensatorV2Config& config)
    : config_(config) {

    if (!config_.enable) {
        ROS_INFO("Distortion compensation V2 disabled");
        return;
    }

    // 配置补偿器
    lidar_distortion::DistortionConfigV2 comp_config;
    comp_config.scan_period = config_.scan_period;

    // 补偿模式
    if (config_.mode == "velocity_only") {
        comp_config.mode = lidar_distortion::DistortionConfigV2::Mode::VELOCITY_ONLY;
    } else if (config_.mode == "velocity_accel") {
        comp_config.mode = lidar_distortion::DistortionConfigV2::Mode::VELOCITY_ACCEL;
    } else if (config_.mode == "full_6dof") {
        comp_config.mode = lidar_distortion::DistortionConfigV2::Mode::FULL_6DOF;
    } else if (config_.mode == "preintegration") {
        comp_config.mode = lidar_distortion::DistortionConfigV2::Mode::IMU_PREINTEGRATION;
    }

    // 点云时间源
    if (config_.point_time_source == "auto") {
        comp_config.point_time_source = lidar_distortion::DistortionConfigV2::PointTimeSource::AUTO_DETECT;
    } else if (config_.point_time_source == "point_time") {
        comp_config.point_time_source = lidar_distortion::DistortionConfigV2::PointTimeSource::USE_POINT_TIME;
    } else if (config_.point_time_source == "angle") {
        comp_config.point_time_source = lidar_distortion::DistortionConfigV2::PointTimeSource::COMPUTE_FROM_ANGLE;
    }

    // 参考时刻
    if (config_.ref_time == "scan_start") {
        comp_config.ref_time = lidar_distortion::DistortionConfigV2::RefTime::SCAN_START;
    } else if (config_.ref_time == "scan_middle") {
        comp_config.ref_time = lidar_distortion::DistortionConfigV2::RefTime::SCAN_MIDDLE;
    } else if (config_.ref_time == "scan_end") {
        comp_config.ref_time = lidar_distortion::DistortionConfigV2::RefTime::SCAN_END;
    }

    // 外参
    comp_config.extrinsics.enable = config_.extrinsics_enable;
    if (config_.extrinsics_enable) {
        comp_config.extrinsics.translation << config_.extrinsics_tx,
                                               config_.extrinsics_ty,
                                               config_.extrinsics_tz;
        comp_config.extrinsics.setRotationFromEuler(config_.extrinsics_roll,
                                                     config_.extrinsics_pitch,
                                                     config_.extrinsics_yaw);
    }

    comp_config.debug_output = config_.debug_output;

    compensator_.SetConfig(comp_config);

    // 订阅IMU
    imu_sub_ = nh.subscribe(config_.imu_topic, config_.imu_buffer_size,
                            &DistortionCompensatorV2::imuCallback, this);

    // 调试发布
    if (config_.publish_debug_info) {
        debug_pub_ = nh.advertise<std_msgs::String>("distortion_debug", 10);
    }

    ROS_INFO("Distortion compensation V2 enabled:");
    ROS_INFO("  - Mode: %s", config_.mode.c_str());
    ROS_INFO("  - Point time source: %s", config_.point_time_source.c_str());
    ROS_INFO("  - Reference time: %s", config_.ref_time.c_str());
    ROS_INFO("  - Scan period: %.3f s", config_.scan_period);
    ROS_INFO("  - Extrinsics: %s", config_.extrinsics_enable ? "enabled" : "disabled");
}

void DistortionCompensatorV2::imuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);

    lidar_distortion::ImuState imu;
    imu.timestamp = msg->header.stamp.toSec();

    // 速度转换：NED导航系 -> 车体FRD系（完整ZYX旋转）
    double heading_rad = fsd_common::DegToRad(msg->Heading);
    double pitch_rad = fsd_common::DegToRad(msg->Pitch);
    double roll_rad = fsd_common::DegToRad(msg->Roll);

    Eigen::Matrix3d R_ned2body;
    double ch = std::cos(heading_rad), sh = std::sin(heading_rad);
    double cp = std::cos(pitch_rad),   sp = std::sin(pitch_rad);
    double cr = std::cos(roll_rad),    sr = std::sin(roll_rad);

    // ZYX欧拉角旋转矩阵 (Heading-Pitch-Roll)
    R_ned2body <<
        ch*cp,                  sh*cp,                 -sp,
        ch*sp*sr - sh*cr,       sh*sp*sr + ch*cr,       cp*sr,
        ch*sp*cr + sh*sr,       sh*sp*cr - ch*sr,       cp*cr;

    Eigen::Vector3d v_ned(msg->Vn, msg->Ve, msg->Vd);
    Eigen::Vector3d v_body = R_ned2body * v_ned;

    imu.velocity = v_body;

    // 角速度（已经是FRD车体系）
    imu.angular_velocity.x() = msg->gyro_x;
    imu.angular_velocity.y() = msg->gyro_y;
    imu.angular_velocity.z() = msg->gyro_z;

    // 加速度（已经是FRD车体系）
    imu.acceleration.x() = msg->acc_x;
    imu.acceleration.y() = msg->acc_y;
    imu.acceleration.z() = msg->acc_z;

    // 姿态角
    imu.roll = msg->Roll;
    imu.pitch = msg->Pitch;
    imu.yaw = msg->Heading;
    imu.updateOrientation();

    compensator_.AddImuData(imu);
    has_imu_data_ = true;
}

bool DistortionCompensatorV2::Compensate(CloudPtr& cloud, double timestamp) {
    if (!config_.enable) {
        return true;  // 未启用，直接返回成功
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);

    if (!has_imu_data_) {
        ROS_WARN_THROTTLE(1.0, "No IMU data received yet for distortion compensation");
        return false;
    }

    bool success = compensator_.Compensate(cloud, timestamp);

    // 发布调试信息
    if (config_.publish_debug_info && debug_pub_.getNumSubscribers() > 0) {
        const auto& debug = compensator_.GetDebugInfo();
        std::stringstream ss;
        ss << "Distortion V2 Debug:\n"
           << "  Points: " << debug.points_compensated << "\n"
           << "  Max disp: " << debug.max_displacement * 1000.0 << " mm\n"
           << "  Avg disp: " << debug.avg_displacement * 1000.0 << " mm\n"
           << "  Velocity: [" << debug.ref_velocity.x() << ", "
           << debug.ref_velocity.y() << ", " << debug.ref_velocity.z() << "] m/s\n"
           << "  Used point time: " << (debug.used_point_time ? "yes" : "no") << "\n"
           << "  Used preintegration: " << (debug.used_preintegration ? "yes" : "no");
        if (!debug.error_msg.empty()) {
            ss << "\n  Error: " << debug.error_msg;
        }

        std_msgs::String msg;
        msg.data = ss.str();
        debug_pub_.publish(msg);
    }

    if (!success && config_.debug_output) {
        ROS_WARN_THROTTLE(1.0, "Distortion compensation failed: %s",
                          compensator_.GetDebugInfo().error_msg.c_str());
    }

    return success;
}

bool DistortionCompensatorV2::CompensateFromMsg(const sensor_msgs::PointCloud2& msg,
                                                  CloudPtr& cloud) {
    if (!config_.enable) {
        pcl::fromROSMsg(msg, *cloud);
        return true;
    }

    // 根据配置决定是否使用点云time字段
    bool has_time = HasTimeField(msg);
    bool use_time_field = false;

    if (config_.point_time_source == "point_time") {
        // 用户明确要求使用time字段
        if (has_time) {
            use_time_field = true;
        } else {
            ROS_WARN_THROTTLE(5.0, "point_time_source='point_time' but cloud has no time field, falling back to angle");
        }
    } else if (config_.point_time_source == "angle") {
        // 用户明确要求使用角度计算
        use_time_field = false;
    } else {
        // auto: 有time字段就用
        use_time_field = has_time;
    }

    if (use_time_field) {
        // 使用带time字段的点云
        pcl::PointCloud<lidar_distortion::PointXYZIRT>::Ptr cloud_with_time(
            new pcl::PointCloud<lidar_distortion::PointXYZIRT>);
        pcl::fromROSMsg(msg, *cloud_with_time);

        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (!has_imu_data_) {
            // 没有IMU数据，直接转换
            cloud.reset(new pcl::PointCloud<PointType>);
            cloud->reserve(cloud_with_time->size());
            for (const auto& pt : cloud_with_time->points) {
                PointType p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z;
                p.intensity = pt.intensity;
                cloud->push_back(p);
            }
            cloud->header = cloud_with_time->header;
            cloud->width = cloud->size();
            cloud->height = 1;
            return true;
        }

        return compensator_.CompensateWithTime<lidar_distortion::PointXYZIRT>(
            cloud_with_time, cloud, msg.header.stamp.toSec());
    } else {
        // 标准PointXYZI点云，使用角度计算时间
        cloud.reset(new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(msg, *cloud);
        return Compensate(cloud, msg.header.stamp.toSec());
    }
}

}  // namespace perception_ros
