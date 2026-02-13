/**
 * @file distortion_adjust_v2.hpp
 * @brief 改进版点云畸变补偿
 *
 * 改进点：
 * 1. 支持Velodyne点云自带的time字段
 * 2. IMU预积分，更精确的运动估计
 * 3. IMU到LiDAR外参补偿
 * 4. 支持多种补偿模式
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>
#include <vector>
#include <string>

namespace lidar_distortion {

/**
 * @brief Velodyne点类型（带time和ring字段）
 */
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float time;  // 相对于扫描起始的时间偏移 [s]
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

/**
 * @brief IMU状态（用于插值和预积分）
 */
struct ImuState {
    double timestamp = 0.0;

    // 速度 (车体坐标系 FRD) [m/s]
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    // 角速度 (车体坐标系 FRD) [rad/s]
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

    // 加速度 (车体坐标系 FRD) [m/s²]
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    // 姿态角 [度]
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // 姿态四元数（从姿态角计算）
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    // 从姿态角更新四元数
    void updateOrientation() {
        double r = roll * M_PI / 180.0;
        double p = pitch * M_PI / 180.0;
        double y = yaw * M_PI / 180.0;
        orientation = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
    }
};

/**
 * @brief IMU预积分结果
 */
struct ImuPreintegration {
    double dt = 0.0;                                      // 积分时间
    Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();    // 位置增量
    Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();    // 速度增量
    Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();  // 旋转增量
};

/**
 * @brief IMU到LiDAR外参
 */
struct ImuLidarExtrinsics {
    bool enable = false;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();  // IMU在LiDAR坐标系下的位置 [m]
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();  // IMU到LiDAR的旋转

    // 从欧拉角设置旋转 (度)
    void setRotationFromEuler(double roll_deg, double pitch_deg, double yaw_deg) {
        double r = roll_deg * M_PI / 180.0;
        double p = pitch_deg * M_PI / 180.0;
        double y = yaw_deg * M_PI / 180.0;
        rotation = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
    }

    // 将IMU坐标系下的向量转换到LiDAR坐标系
    Eigen::Vector3d transformVector(const Eigen::Vector3d& v_imu) const {
        return rotation * v_imu;
    }

    // 将IMU坐标系下的点转换到LiDAR坐标系
    Eigen::Vector3d transformPoint(const Eigen::Vector3d& p_imu) const {
        return rotation * p_imu + translation;
    }
};

/**
 * @brief 畸变补偿配置
 */
struct DistortionConfigV2 {
    // 扫描周期 [s]
    double scan_period = 0.1;

    // 补偿模式
    enum class Mode {
        VELOCITY_ONLY,      // 仅速度补偿
        VELOCITY_ACCEL,     // 速度+加速度补偿
        FULL_6DOF,          // 完整6自由度补偿（含旋转）
        IMU_PREINTEGRATION  // IMU预积分补偿（最精确）
    };
    Mode mode = Mode::VELOCITY_ACCEL;

    // 点云时间字段
    enum class PointTimeSource {
        AUTO_DETECT,        // 自动检测
        USE_POINT_TIME,     // 使用点云time字段
        COMPUTE_FROM_ANGLE  // 从角度计算
    };
    PointTimeSource point_time_source = PointTimeSource::AUTO_DETECT;

    // 是否使用IMU插值
    bool use_imu_interpolation = true;

    // 最大允许的时间差 [s]
    double max_time_diff = 0.2;

    // 参考时刻选择
    enum class RefTime {
        SCAN_START,     // 扫描起始时刻
        SCAN_MIDDLE,    // 扫描中间时刻
        SCAN_END        // 扫描结束时刻
    };
    RefTime ref_time = RefTime::SCAN_END;  // 默认使用扫描结束时刻（与ROS时间戳一致）

    // IMU到LiDAR外参
    ImuLidarExtrinsics extrinsics;

    // Velodyne扫描方向
    bool clockwise_scan = true;  // true=顺时针（从上往下看）

    // 重力加速度 [m/s²]，FRD坐标系下向下为正
    double gravity = 9.7883105;

    // 调试选项
    bool debug_output = false;
};

/**
 * @brief 改进版畸变补偿器
 */
class DistortionAdjustV2 {
public:
    using PointType = pcl::PointXYZI;
    using CloudPtr = pcl::PointCloud<PointType>::Ptr;
    using ConstCloudPtr = pcl::PointCloud<PointType>::ConstPtr;

    DistortionAdjustV2() = default;

    /**
     * @brief 设置配置
     */
    void SetConfig(const DistortionConfigV2& config) { config_ = config; }

    /**
     * @brief 获取配置
     */
    const DistortionConfigV2& GetConfig() const { return config_; }

    /**
     * @brief 添加IMU数据到缓冲区
     */
    void AddImuData(const ImuState& imu);

    /**
     * @brief 清空IMU缓冲区
     */
    void ClearImuBuffer() { imu_buffer_.clear(); }

    /**
     * @brief 获取IMU缓冲区大小
     */
    size_t GetImuBufferSize() const { return imu_buffer_.size(); }

    /**
     * @brief 执行畸变补偿（标准PointXYZI点云）
     * @param cloud 输入/输出点云
     * @param cloud_timestamp 点云时间戳 [s]
     * @return 是否成功
     */
    bool Compensate(CloudPtr& cloud, double cloud_timestamp);

    /**
     * @brief 执行畸变补偿（带time字段的点云）
     * @param cloud 输入点云（带time字段）
     * @param output 输出点云（标准PointXYZI）
     * @param cloud_timestamp 点云时间戳 [s]
     * @return 是否成功
     */
    template<typename PointT>
    bool CompensateWithTime(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                            CloudPtr& output,
                            double cloud_timestamp);

    /**
     * @brief 调试信息
     */
    struct DebugInfo {
        double ref_timestamp = 0.0;
        Eigen::Vector3d ref_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d ref_angular_velocity = Eigen::Vector3d::Zero();
        size_t points_compensated = 0;
        double max_displacement = 0.0;
        double avg_displacement = 0.0;
        bool used_point_time = false;
        bool used_preintegration = false;
        std::string error_msg;
    };
    const DebugInfo& GetDebugInfo() const { return debug_info_; }

private:
    /**
     * @brief 在指定时刻插值IMU数据
     */
    bool InterpolateImu(double timestamp, ImuState& out) const;

    /**
     * @brief IMU预积分
     * @param t_start 起始时间
     * @param t_end 结束时间
     * @param result 预积分结果
     * @return 是否成功
     */
    bool Preintegrate(double t_start, double t_end, ImuPreintegration& result) const;

    /**
     * @brief 计算点的时间偏移（从角度）
     */
    double ComputePointTimeFromAngle(float x, float y) const;

    /**
     * @brief 补偿单个点（简单模式）
     */
    void CompensatePointSimple(PointType& point, double dt, const ImuState& ref_state);

    /**
     * @brief 补偿单个点（预积分模式）
     */
    void CompensatePointPreintegration(PointType& point, double point_time,
                                        double ref_time, const ImuState& ref_state);

    /**
     * @brief 应用外参变换（向量旋转，适用于角速度、加速度）
     */
    Eigen::Vector3d ApplyExtrinsics(const Eigen::Vector3d& v_imu) const;

    /**
     * @brief 应用外参变换到速度（含杠杆臂补偿）
     * v_lidar = R * v_imu + omega_lidar × t_imu_to_lidar
     */
    Eigen::Vector3d ApplyExtrinsicsVelocity(const Eigen::Vector3d& v_imu,
                                             const Eigen::Vector3d& omega_imu) const;

    DistortionConfigV2 config_;
    std::deque<ImuState> imu_buffer_;
    static constexpr size_t kMaxImuBufferSize = 1000;

    DebugInfo debug_info_;
};

// ============== 模板实现 ==============

template<typename PointT>
bool DistortionAdjustV2::CompensateWithTime(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    CloudPtr& output,
    double cloud_timestamp) {

    if (!cloud || cloud->empty()) {
        debug_info_.error_msg = "Empty input cloud";
        return false;
    }

    // pt.time 语义校验：检查首个点的time字段范围
    {
        float t0 = cloud->points[0].time;
        if (t0 > 1e6) {
            // time字段看起来是绝对时间戳，不是相对偏移
            debug_info_.error_msg = "pt.time appears to be absolute timestamp, expected relative offset [0, scan_period]";
            return false;
        }
        if (t0 < -config_.scan_period * 1.5) {
            // time字段为较大负值，可能是相对scan结束的偏移
            debug_info_.error_msg = "pt.time is negative, driver may use scan-end-relative convention";
            return false;
        }
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
    if (!InterpolateImu(ref_timestamp, ref_state)) {
        debug_info_.error_msg = "Failed to interpolate IMU at ref time";
        return false;
    }

    // 初始化输出点云
    output.reset(new pcl::PointCloud<PointType>);
    output->reserve(cloud->size());
    output->header = cloud->header;

    // 更新调试信息
    debug_info_.ref_timestamp = ref_timestamp;
    debug_info_.ref_velocity = ref_state.velocity;
    debug_info_.ref_angular_velocity = ref_state.angular_velocity;
    debug_info_.points_compensated = 0;
    debug_info_.max_displacement = 0.0;
    debug_info_.avg_displacement = 0.0;
    debug_info_.used_point_time = true;
    debug_info_.used_preintegration = (config_.mode == DistortionConfigV2::Mode::IMU_PREINTEGRATION);
    debug_info_.error_msg.clear();

    double total_displacement = 0.0;

    // 补偿每个点
    for (const auto& pt : cloud->points) {
        // 获取点的时间（相对于扫描起始）
        double point_rel_time = static_cast<double>(pt.time);

        // 计算点的绝对时间
        double point_abs_time = cloud_timestamp - config_.scan_period + point_rel_time;

        // 计算相对于参考时刻的时间差
        double dt = point_abs_time - ref_timestamp;

        // 创建输出点
        PointType out_pt;
        out_pt.x = pt.x;
        out_pt.y = pt.y;
        out_pt.z = pt.z;
        out_pt.intensity = pt.intensity;

        // 根据模式补偿
        if (config_.mode == DistortionConfigV2::Mode::IMU_PREINTEGRATION) {
            CompensatePointPreintegration(out_pt, point_abs_time, ref_timestamp, ref_state);
        } else {
            CompensatePointSimple(out_pt, dt, ref_state);
        }

        // 计算位移量（用于调试）
        double disp = std::sqrt(
            std::pow(out_pt.x - pt.x, 2) +
            std::pow(out_pt.y - pt.y, 2) +
            std::pow(out_pt.z - pt.z, 2));
        total_displacement += disp;
        if (disp > debug_info_.max_displacement) {
            debug_info_.max_displacement = disp;
        }

        output->push_back(out_pt);
        debug_info_.points_compensated++;
    }

    if (debug_info_.points_compensated > 0) {
        debug_info_.avg_displacement = total_displacement / debug_info_.points_compensated;
    }

    output->width = output->size();
    output->height = 1;
    output->is_dense = cloud->is_dense;

    return true;
}

}  // namespace lidar_distortion

// PCL点类型注册
POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_distortion::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
)
