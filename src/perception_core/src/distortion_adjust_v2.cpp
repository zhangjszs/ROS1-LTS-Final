/**
 * @file distortion_adjust_v2.cpp
 * @brief 改进版点云畸变补偿实现
 */

#include <perception_core/distortion_adjust_v2.hpp>
#include <cmath>
#include <algorithm>

namespace lidar_distortion {

void DistortionAdjustV2::AddImuData(const ImuState& imu) {
    // 保持时间顺序
    if (!imu_buffer_.empty() && imu.timestamp < imu_buffer_.back().timestamp) {
        // 时间戳回退，清空缓冲区（可能是rosbag重放）
        imu_buffer_.clear();
        preint_cache_.clear();
    }

    imu_buffer_.push_back(imu);

    // 限制缓冲区大小
    while (imu_buffer_.size() > kMaxImuBufferSize) {
        imu_buffer_.pop_front();
    }
}

bool DistortionAdjustV2::Compensate(CloudPtr& cloud, double cloud_timestamp) {
    if (!cloud || cloud->empty()) {
        debug_info_.error_msg = "Empty input cloud";
        return false;
    }

    // 计算参考时刻
    double ref_timestamp;
    switch (config_.ref_time) {
        case DistortionConfigV2::RefTime::SCAN_START:
            ref_timestamp = cloud_timestamp - config_.scan_period;
            break;
        case DistortionConfigV2::RefTime::SCAN_MIDDLE:
            ref_timestamp = cloud_timestamp - config_.scan_period / 2.0;
            break;
        case DistortionConfigV2::RefTime::SCAN_END:
        default:
            ref_timestamp = cloud_timestamp;
            break;
    }

    // 获取参考时刻的IMU状态
    ImuState ref_state;
    if (config_.use_imu_interpolation) {
        if (!InterpolateImu(ref_timestamp, ref_state)) {
            if (imu_buffer_.empty()) {
                debug_info_.error_msg = "IMU buffer empty";
                return false;
            }
            // 使用最近的IMU数据
            ref_state = imu_buffer_.back();
        }
    } else {
        if (imu_buffer_.empty()) {
            debug_info_.error_msg = "IMU buffer empty";
            return false;
        }
        // 找最近的IMU数据
        double min_dt = std::numeric_limits<double>::max();
        for (const auto& imu : imu_buffer_) {
            double dt = std::abs(imu.timestamp - ref_timestamp);
            if (dt < min_dt) {
                min_dt = dt;
                ref_state = imu;
            }
        }
        if (min_dt > config_.max_time_diff) {
            debug_info_.error_msg = "IMU data too old";
            return false;
        }
    }

    // 更新调试信息
    debug_info_.ref_timestamp = ref_timestamp;
    debug_info_.ref_velocity = ref_state.velocity;
    debug_info_.ref_angular_velocity = ref_state.angular_velocity;
    debug_info_.points_compensated = 0;
    debug_info_.max_displacement = 0.0;
    debug_info_.avg_displacement = 0.0;
    debug_info_.used_point_time = false;
    debug_info_.used_preintegration = (config_.mode == DistortionConfigV2::Mode::IMU_PREINTEGRATION);
    debug_info_.error_msg.clear();

    double total_displacement = 0.0;

    // 补偿每个点
    for (auto& point : cloud->points) {
        // 计算该点相对于参考时刻的时间偏移
        double point_rel_time = ComputePointTimeFromAngle(point.x, point.y);

        // 计算相对于参考时刻的时间差
        double dt;
        switch (config_.ref_time) {
            case DistortionConfigV2::RefTime::SCAN_START:
                dt = point_rel_time;
                break;
            case DistortionConfigV2::RefTime::SCAN_END:
                dt = point_rel_time - config_.scan_period;
                break;
            case DistortionConfigV2::RefTime::SCAN_MIDDLE:
            default:
                dt = point_rel_time - config_.scan_period / 2.0;
                break;
        }

        // 保存原始位置用于计算位移
        float orig_x = point.x;
        float orig_y = point.y;
        float orig_z = point.z;

        // 根据模式补偿
        if (config_.mode == DistortionConfigV2::Mode::IMU_PREINTEGRATION) {
            double point_abs_time = cloud_timestamp - config_.scan_period + point_rel_time;
            CompensatePointPreintegration(point, point_abs_time, ref_timestamp, ref_state);
        } else {
            CompensatePointSimple(point, dt, ref_state);
        }

        // 计算位移量
        double disp = std::sqrt(
            std::pow(point.x - orig_x, 2) +
            std::pow(point.y - orig_y, 2) +
            std::pow(point.z - orig_z, 2));
        total_displacement += disp;
        if (disp > debug_info_.max_displacement) {
            debug_info_.max_displacement = disp;
        }

        debug_info_.points_compensated++;
    }

    if (debug_info_.points_compensated > 0) {
        debug_info_.avg_displacement = total_displacement / debug_info_.points_compensated;
    }

    return true;
}

bool DistortionAdjustV2::InterpolateImu(double timestamp, ImuState& out) const {
    if (imu_buffer_.size() < 2) {
        if (imu_buffer_.size() == 1) {
            double dt = std::abs(imu_buffer_.front().timestamp - timestamp);
            if (dt < config_.max_time_diff) {
                out = imu_buffer_.front();
                return true;
            }
        }
        return false;
    }

    // 找到包围timestamp的两个IMU数据
    const ImuState* before = nullptr;
    const ImuState* after = nullptr;

    for (size_t i = 0; i < imu_buffer_.size(); ++i) {
        if (imu_buffer_[i].timestamp >= timestamp) {
            if (i > 0) {
                before = &imu_buffer_[i - 1];
                after = &imu_buffer_[i];
            } else {
                // timestamp在第一个数据之前
                double dt = imu_buffer_[i].timestamp - timestamp;
                if (dt < config_.max_time_diff) {
                    out = imu_buffer_[i];
                    return true;
                }
                return false;
            }
            break;
        }
    }

    // timestamp在最后一个数据之后
    if (!after) {
        double dt = timestamp - imu_buffer_.back().timestamp;
        if (dt < config_.max_time_diff) {
            out = imu_buffer_.back();
            return true;
        }
        return false;
    }

    // 线性插值
    double dt = after->timestamp - before->timestamp;
    if (dt < 1e-9) {
        out = *before;
        return true;
    }

    double alpha = (timestamp - before->timestamp) / dt;

    out.timestamp = timestamp;
    out.velocity = (1.0 - alpha) * before->velocity + alpha * after->velocity;
    out.angular_velocity = (1.0 - alpha) * before->angular_velocity +
                           alpha * after->angular_velocity;
    out.acceleration = (1.0 - alpha) * before->acceleration +
                       alpha * after->acceleration;
    out.roll = (1.0 - alpha) * before->roll + alpha * after->roll;
    out.pitch = (1.0 - alpha) * before->pitch + alpha * after->pitch;

    // 航向角需要特殊处理（角度环绕）
    double yaw_diff = after->yaw - before->yaw;
    if (yaw_diff > 180.0) yaw_diff -= 360.0;
    if (yaw_diff < -180.0) yaw_diff += 360.0;
    out.yaw = before->yaw + alpha * yaw_diff;

    // 四元数球面线性插值 (SLERP)
    out.orientation = before->orientation.slerp(alpha, after->orientation);

    return true;
}

bool DistortionAdjustV2::Preintegrate(double t_start, double t_end,
                                       ImuPreintegration& result) const {
    if (imu_buffer_.empty()) {
        return false;
    }

    result.dt = t_end - t_start;
    result.delta_p.setZero();
    result.delta_v.setZero();
    result.delta_q.setIdentity();

    if (std::abs(result.dt) < 1e-9) {
        return true;
    }

    // 找到时间范围内的IMU数据
    std::vector<const ImuState*> imu_in_range;
    for (const auto& imu : imu_buffer_) {
        if (imu.timestamp >= t_start && imu.timestamp <= t_end) {
            imu_in_range.push_back(&imu);
        }
    }

    if (imu_in_range.empty()) {
        // 没有IMU数据在范围内，使用插值
        ImuState interp_state;
        double t_mid = (t_start + t_end) / 2.0;
        if (!InterpolateImu(t_mid, interp_state)) {
            return false;
        }

        // 简单积分
        double dt = t_end - t_start;
        Eigen::Vector3d acc = ApplyExtrinsics(interp_state.acceleration);
        Eigen::Vector3d gyro = ApplyExtrinsics(interp_state.angular_velocity);

        result.delta_v = acc * dt;
        result.delta_p = 0.5 * acc * dt * dt;

        // 旋转增量
        Eigen::Vector3d angle = gyro * dt;
        double angle_norm = angle.norm();
        if (angle_norm > 1e-9) {
            result.delta_q = Eigen::Quaterniond(
                Eigen::AngleAxisd(angle_norm, angle / angle_norm));
        }

        return true;
    }

    // 中点积分法
    double last_t = t_start;
    Eigen::Vector3d last_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();

    // 获取起始时刻的IMU状态
    ImuState start_state;
    if (InterpolateImu(t_start, start_state)) {
        last_acc = ApplyExtrinsics(start_state.acceleration);
        last_gyro = ApplyExtrinsics(start_state.angular_velocity);
    }

    for (const auto* imu : imu_in_range) {
        double curr_t = imu->timestamp;
        double dt = curr_t - last_t;

        if (dt < 1e-9) continue;

        Eigen::Vector3d curr_acc = ApplyExtrinsics(imu->acceleration);
        Eigen::Vector3d curr_gyro = ApplyExtrinsics(imu->angular_velocity);

        // 中点值
        Eigen::Vector3d mid_acc = 0.5 * (last_acc + curr_acc);
        Eigen::Vector3d mid_gyro = 0.5 * (last_gyro + curr_gyro);

        // 旋转增量
        Eigen::Vector3d angle = mid_gyro * dt;
        double angle_norm = angle.norm();
        Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
        if (angle_norm > 1e-9) {
            dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle_norm, angle / angle_norm));
        }

        // 更新预积分量
        // 位置增量需要考虑旋转
        result.delta_p += result.delta_v * dt + 0.5 * (result.delta_q * mid_acc) * dt * dt;
        result.delta_v += (result.delta_q * mid_acc) * dt;
        result.delta_q = result.delta_q * dq;
        result.delta_q.normalize();

        last_t = curr_t;
        last_acc = curr_acc;
        last_gyro = curr_gyro;
    }

    // 处理最后一段到t_end
    if (last_t < t_end) {
        ImuState end_state;
        if (InterpolateImu(t_end, end_state)) {
            double dt = t_end - last_t;
            Eigen::Vector3d curr_acc = ApplyExtrinsics(end_state.acceleration);
            Eigen::Vector3d curr_gyro = ApplyExtrinsics(end_state.angular_velocity);

            Eigen::Vector3d mid_acc = 0.5 * (last_acc + curr_acc);
            Eigen::Vector3d mid_gyro = 0.5 * (last_gyro + curr_gyro);

            Eigen::Vector3d angle = mid_gyro * dt;
            double angle_norm = angle.norm();
            Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
            if (angle_norm > 1e-9) {
                dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle_norm, angle / angle_norm));
            }

            result.delta_p += result.delta_v * dt + 0.5 * (result.delta_q * mid_acc) * dt * dt;
            result.delta_v += (result.delta_q * mid_acc) * dt;
            result.delta_q = result.delta_q * dq;
            result.delta_q.normalize();
        }
    }

    return true;
}

double DistortionAdjustV2::ComputePointTimeFromAngle(float x, float y) const {
    // 计算方位角
    double azimuth = std::atan2(static_cast<double>(y), static_cast<double>(x));

    // 将方位角转换到 [0, 2π] 范围
    if (azimuth < 0) {
        azimuth += 2.0 * M_PI;
    }

    // Velodyne扫描方向
    double time;
    if (config_.clockwise_scan) {
        // 顺时针扫描：方位角增大 -> 时间增大
        time = azimuth / (2.0 * M_PI) * config_.scan_period;
    } else {
        // 逆时针扫描：方位角减小 -> 时间增大
        time = (2.0 * M_PI - azimuth) / (2.0 * M_PI) * config_.scan_period;
    }

    return time;
}

void DistortionAdjustV2::CompensatePointSimple(PointType& point, double dt,
                                                const ImuState& ref_state) {
    Eigen::Vector3d p(point.x, point.y, point.z);

    // 应用外参变换到速度和加速度
    Eigen::Vector3d velocity = ApplyExtrinsics(ref_state.velocity);
    Eigen::Vector3d acceleration = ApplyExtrinsics(ref_state.acceleration);
    Eigen::Vector3d angular_velocity = ApplyExtrinsics(ref_state.angular_velocity);

    // 计算位移补偿
    Eigen::Vector3d displacement;

    switch (config_.mode) {
        case DistortionConfigV2::Mode::VELOCITY_ACCEL:
            // s = v*t + 0.5*a*t^2
            displacement = velocity * dt + 0.5 * acceleration * dt * dt;
            break;

        case DistortionConfigV2::Mode::FULL_6DOF: {
            // 完整6DOF补偿
            // 1. 旋转补偿
            Eigen::Vector3d angle = angular_velocity * dt;
            double angle_norm = angle.norm();
            Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
            if (angle_norm > 1e-9) {
                Eigen::AngleAxisd aa(angle_norm, angle / angle_norm);
                rotation = aa.toRotationMatrix();
            }

            // 旋转点到参考时刻
            p = rotation.transpose() * p;

            // 2. 平移补偿
            displacement = velocity * dt + 0.5 * acceleration * dt * dt;
            break;
        }

        case DistortionConfigV2::Mode::VELOCITY_ONLY:
        default:
            // s = v*t
            displacement = velocity * dt;
            break;
    }

    // 应用补偿（减去位移，将点移动到参考时刻）
    p -= displacement;

    // 写回点
    point.x = static_cast<float>(p.x());
    point.y = static_cast<float>(p.y());
    point.z = static_cast<float>(p.z());
}

void DistortionAdjustV2::CompensatePointPreintegration(PointType& point,
                                                        double point_time,
                                                        double ref_time,
                                                        const ImuState& ref_state) {
    Eigen::Vector3d p(point.x, point.y, point.z);

    // 预积分从point_time到ref_time
    ImuPreintegration preint;
    if (!Preintegrate(point_time, ref_time, preint)) {
        // 预积分失败，回退到简单模式
        double dt = point_time - ref_time;
        CompensatePointSimple(point, dt, ref_state);
        return;
    }

    // 应用预积分结果
    // 旋转补偿
    p = preint.delta_q.inverse() * p;

    // 平移补偿
    p -= preint.delta_p;

    // 写回点
    point.x = static_cast<float>(p.x());
    point.y = static_cast<float>(p.y());
    point.z = static_cast<float>(p.z());
}

Eigen::Vector3d DistortionAdjustV2::ApplyExtrinsics(const Eigen::Vector3d& v_imu) const {
    if (config_.extrinsics.enable) {
        return config_.extrinsics.transformVector(v_imu);
    }
    return v_imu;
}

}  // namespace lidar_distortion
