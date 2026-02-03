#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include <utility>

namespace perception {

/**
 * @brief 锥桶跟踪器 - 利用时序信息提高检测稳定性
 *
 * 功能：
 * 1. 多帧一致性验证：连续N帧检测到才确认
 * 2. 卡尔曼滤波平滑位置
 * 3. 减少闪烁和误检
 */
class ConeTracker {
public:
    struct TrackedCone {
        int id = -1;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double confidence = 0.0;
        int hit_count = 0;      // 连续检测到的帧数
        int miss_count = 0;     // 连续未检测到的帧数
        bool confirmed = false; // 是否已确认
        double vx = 0.0;        // 速度估计（用于预测）
        double vy = 0.0;
    };

    struct Config {
        double association_threshold = 0.5;  // 关联距离阈值 [m]
        int confirm_frames = 3;              // 确认所需连续帧数
        int delete_frames = 5;               // 删除所需连续丢失帧数
        double process_noise = 0.1;          // 过程噪声
        double measurement_noise = 0.05;     // 测量噪声
        bool enable_velocity_prediction = true;
    };

    struct Detection {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double confidence = 0.0;
    };

    ConeTracker() = default;

    void setConfig(const Config& config) { config_ = config; }

    /**
     * @brief 更新跟踪器
     * @param detections 当前帧检测结果
     * @param dt 时间间隔 [s]
     */
    void update(const std::vector<Detection>& detections, double dt);

    /**
     * @brief 获取已确认的锥桶
     * @return 已确认的锥桶列表
     */
    std::vector<TrackedCone> getConfirmedCones() const;

    /**
     * @brief 获取所有跟踪的锥桶（包括未确认的）
     */
    std::vector<TrackedCone> getAllTracks() const { return tracks_; }

    /**
     * @brief 重置跟踪器
     */
    void reset() {
        tracks_.clear();
        next_id_ = 0;
    }

private:
    /**
     * @brief 预测步骤
     */
    void predict(double dt);

    /**
     * @brief 数据关联（匈牙利算法简化版）
     */
    std::vector<std::pair<int, int>> associate(const std::vector<Detection>& detections);

    /**
     * @brief 更新已关联的轨迹
     */
    void updateTrack(TrackedCone& track, const Detection& det);

    /**
     * @brief 创建新轨迹
     */
    void createTrack(const Detection& det);

    /**
     * @brief 删除丢失的轨迹
     */
    void pruneDeadTracks();

    Config config_;
    std::vector<TrackedCone> tracks_;
    int next_id_ = 0;
};

}  // namespace perception
