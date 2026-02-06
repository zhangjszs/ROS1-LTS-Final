#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>

namespace perception {

/**
 * @brief Fast Ground Segmentation (FGS) 快速地面分割算法
 *
 * 基于极坐标栅格的地面分割，复杂度O(n)，单帧处理<5ms
 *
 * 算法原理：
 * 1. 将点云投影到极坐标系 (r, theta)
 * 2. 按角度划分扇区，每个扇区按距离划分bin
 * 3. 每个bin内提取最低点作为地面候选
 * 4. 对每个扇区进行线性拟合得到地面模型
 * 5. 根据点到地面模型的距离分类
 *
 * CPU优化：
 * - 单遍扫描，无迭代
 * - 内存预分配，避免动态分配
 * - 缓存友好的数据布局
 */

using PointType = pcl::PointXYZI;

/**
 * @brief 线段模型 (Zermas 2017 增量线段生长)
 */
struct LineSegment {
    double a = 0.0;          // 斜率
    double b = 0.0;          // 截距
    int bin_start = 0;       // 起始bin (inclusive)
    int bin_end = 0;         // 结束bin (exclusive)
    int num_points = 0;      // 拟合用的bin数
    bool valid = false;
    double getHeight(double r) const { return a * r + b; }
};

struct FGSConfig {
    // 极坐标栅格参数
    int num_sectors = 32;           // 扇区数量 (角度划分)
    int num_bins = 80;              // 每个扇区的bin数量 (距离划分)
    double max_range = 80.0;        // 最大处理距离 (m)
    double min_range = 0.1;         // 最小处理距离 (m) - 降低以处理近距离点

    // 地面拟合参数
    double sensor_height = 0.135;   // 传感器高度 (m)
    double th_ground = 0.08;        // 地面距离阈值 (m)
    double th_ground_far = 0.15;    // 远距离地面阈值 (m)
    double far_distance = 20.0;     // 远距离分界点 (m)

    // 近距离特殊处理
    double near_distance = 2.0;     // 近距离分界点 (m)
    double th_ground_near = 0.12;   // 近距离地面阈值 (m) - 放宽近距离阈值

    // 线性拟合参数
    double max_slope = 0.3;         // 最大地面坡度 (tan值)
    double min_points_per_bin = 1;  // 每个bin最少点数

    // 法向量约束
    double min_normal_z = 0.85;     // 法向量Z分量最小值

    // 高度约束
    double max_height_diff = 0.3;   // 相邻bin最大高度差 (m)

    // 模型插值
    bool use_neighbor_model = true; // 无效扇区使用邻近扇区模型

    // 增量线段生长参数 (Zermas 2017)
    int max_segments_per_sector = 4;     // 每扇区最大线段数
    double segment_merge_dist = 0.15;    // 线段生长偏差阈值 (m)，过小会导致帧间线段数不稳定

    // 地面点精细化 (Zermas 2017)
    bool enable_refinement = false;      // 二次拟合开关（默认关，FSD赛道可不开）

    // 代表点选择 (Himmelsbach 2010)
    bool use_lowest_n_mean = true;       // 使用 lowest-N 均值代替 min_z
    int lowest_n = 3;                    // N 值

    // 扇区间平滑 (Himmelsbach 2010)
    bool enable_sector_smoothing = true; // 相邻扇区模型平滑

    // 帧间时序平滑 (抑制闪烁)
    bool enable_temporal_smoothing = true;  // 帧间EMA平滑开关
    double temporal_alpha = 0.3;            // EMA系数，越小越平滑（0=完全用历史，1=完全用当前帧）

    // 地面判定阈值对称性
    double ground_below_factor = 1.5;       // 下方阈值 = threshold * factor（1.0=完全对称，2.0=旧行为）
    // 说明：车辆 roll 时一侧地面系统性偏高、另一侧偏低
    // 过大的不对称会导致偏高侧地面被误判为障碍物
    // 1.5 是折中值：仍允许地面点略低于模型（噪声），但不会过度放宽

    // temporal alpha 自适应（S弯/快速变向场景）
    bool enable_adaptive_alpha = true;      // 自适应 alpha 开关
    double adaptive_alpha_max = 0.9;        // 模型剧变时的最大 alpha（几乎完全信任当前帧）
    double adaptive_alpha_threshold = 0.05; // 触发自适应的模型变化阈值（a/b 的加权差异）
};

/**
 * @brief 极坐标bin结构
 */
struct PolarBin {
    double min_z = std::numeric_limits<double>::max();  // bin内最低点Z值
    double sum_z = 0.0;                                  // Z值累加
    int count = 0;                                       // 点数
    bool valid = false;                                  // 是否有效

    // Lowest-N buffer (Himmelsbach 2010, 固定大小避免动态分配)
    static constexpr int kLowestN = 3;
    std::array<double, kLowestN> lowest_z;
    int lowest_count = 0;

    void reset() {
        min_z = std::numeric_limits<double>::max();
        sum_z = 0.0;
        count = 0;
        valid = false;
        lowest_count = 0;
    }

    void addPoint(double z) {
        if (z < min_z) min_z = z;
        sum_z += z;
        count++;
        valid = true;

        // 维护 lowest-N 数组（插入排序，O(N) where N=3）
        if (lowest_count < kLowestN) {
            lowest_z[lowest_count] = z;
            lowest_count++;
            // 保持排序
            for (int i = lowest_count - 1; i > 0 && lowest_z[i] < lowest_z[i - 1]; --i) {
                std::swap(lowest_z[i], lowest_z[i - 1]);
            }
        } else if (z < lowest_z[kLowestN - 1]) {
            lowest_z[kLowestN - 1] = z;
            for (int i = kLowestN - 2; i >= 0 && lowest_z[i + 1] < lowest_z[i]; --i) {
                std::swap(lowest_z[i], lowest_z[i + 1]);
            }
        }
    }

    /// 返回 lowest-N 均值作为代表点 (Himmelsbach 2010)
    double getPrototypeZ() const {
        if (lowest_count == 0) return 0.0;
        double sum = 0.0;
        for (int i = 0; i < lowest_count; ++i) {
            sum += lowest_z[i];
        }
        return sum / lowest_count;
    }

    double getMeanZ() const {
        return count > 0 ? sum_z / count : 0.0;
    }
};

/**
 * @brief 扇区地面模型 (线性模型: z = a * r + b)
 */
struct SectorGroundModel {
    double a = 0.0;     // 斜率 (兼容：全局单线模型)
    double b = 0.0;     // 截距 (传感器高度处)
    bool valid = false;
    std::vector<LineSegment> segments;  // 多线段模型 (Zermas 2017)

    double getHeight(double r) const {
        return a * r + b;
    }

    /// 根据bin_idx查找对应线段的高度 (多线段查询)
    /// max_extrapolate_bins: 最大外推距离（bin数），超过则回退到全局模型
    double getSegmentHeight(double r, int bin_idx, int max_extrapolate_bins = 5) const {
        // 查找 bin_idx 所在的线段（精确命中）
        for (const auto& seg : segments) {
            if (seg.valid && bin_idx >= seg.bin_start && bin_idx < seg.bin_end) {
                return seg.getHeight(r);
            }
        }
        // 回退到最近线段，但限制外推距离
        // 远处地面坡度可能和近处完全不同，外推过远会产生大误差
        int min_dist_bins = std::numeric_limits<int>::max();
        const LineSegment* nearest = nullptr;
        for (const auto& seg : segments) {
            if (!seg.valid) continue;
            int d = 0;
            if (bin_idx < seg.bin_start) d = seg.bin_start - bin_idx;
            else if (bin_idx >= seg.bin_end) d = bin_idx - seg.bin_end + 1;
            if (d < min_dist_bins) {
                min_dist_bins = d;
                nearest = &seg;
            }
        }
        // 只在外推距离合理时使用最近线段，否则回退到全局模型
        if (nearest && min_dist_bins <= max_extrapolate_bins) {
            return nearest->getHeight(r);
        }
        // 全局模型：经过帧间 EMA 平滑，更稳定
        return a * r + b;
    }
};

class FastGroundSegmentation {
public:
    FastGroundSegmentation();
    ~FastGroundSegmentation() = default;

    /**
     * @brief 配置算法参数
     */
    void configure(const FGSConfig& config);

    /**
     * @brief 执行地面分割
     * @param input 输入点云
     * @param ground 输出地面点云
     * @param non_ground 输出非地面点云
     */
    void segment(const pcl::PointCloud<PointType>::Ptr& input,
                 pcl::PointCloud<PointType>::Ptr& ground,
                 pcl::PointCloud<PointType>::Ptr& non_ground);

    /**
     * @brief 仅输出非地面点（优化版本，减少内存分配）
     */
    void segmentNonGround(const pcl::PointCloud<PointType>::Ptr& input,
                          pcl::PointCloud<PointType>::Ptr& non_ground);

private:
    /**
     * @brief 初始化极坐标栅格
     */
    void initPolarGrid();

    /**
     * @brief 重置极坐标栅格
     */
    void resetPolarGrid();

    /**
     * @brief 将点云填充到极坐标栅格
     */
    void fillPolarGrid(const pcl::PointCloud<PointType>::Ptr& input);

    /**
     * @brief 拟合每个扇区的地面模型
     */
    void fitGroundModels();

    /**
     * @brief 对单个扇区进行线性拟合
     */
    void fitSectorModel(int sector_idx);

    /**
     * @brief 使用邻近扇区模型填充无效扇区
     */
    void interpolateInvalidModels();

    /**
     * @brief 获取有效的邻近扇区模型
     */
    const SectorGroundModel* getNeighborModel(int sector_idx) const;

    /**
     * @brief 判断点是否为地面点
     */
    bool isGroundPoint(const PointType& pt, int sector_idx, int bin_idx, double distance) const;

    /**
     * @brief 计算点到极坐标索引
     */
    inline void getGridIndex(const PointType& pt, int& sector_idx, int& bin_idx, double& distance) const;

    /**
     * @brief 获取自适应地面阈值
     */
    inline double getAdaptiveThreshold(double distance) const;

    /**
     * @brief 扇区间模型平滑 (Himmelsbach 2010)
     */
    void smoothSectorModels();

    /**
     * @brief 精细化二次拟合 (Zermas 2017, 可选)
     */
    void refinementPass(const pcl::PointCloud<PointType>::Ptr& input);

    /**
     * @brief 帧间时序平滑 (EMA, 抑制闪烁)
     */
    void temporalSmooth();

private:
    FGSConfig config_;

    // 极坐标栅格 [sector][bin]
    std::vector<std::vector<PolarBin>> polar_grid_;

    // 每个扇区的地面模型
    std::vector<SectorGroundModel> sector_models_;

    // 帧间时序平滑：上一帧模型缓存
    std::vector<SectorGroundModel> prev_sector_models_;
    bool has_prev_models_ = false;

    // 预计算的常量
    double sector_angle_;       // 每个扇区的角度 (rad)
    double bin_size_;           // 每个bin的距离大小 (m)
    double inv_sector_angle_;   // 1 / sector_angle_
    double inv_bin_size_;       // 1 / bin_size_

    // 内存预分配
    bool initialized_ = false;
};

// ============== 内联函数实现 ==============

inline void FastGroundSegmentation::getGridIndex(const PointType& pt,
                                                  int& sector_idx,
                                                  int& bin_idx,
                                                  double& distance) const {
    // 计算极坐标
    distance = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    double angle = std::atan2(pt.y, pt.x) + M_PI;  // [0, 2*PI]

    // 计算索引
    sector_idx = static_cast<int>(angle * inv_sector_angle_);
    if (sector_idx >= config_.num_sectors) sector_idx = config_.num_sectors - 1;
    if (sector_idx < 0) sector_idx = 0;

    bin_idx = static_cast<int>((distance - config_.min_range) * inv_bin_size_);
    if (bin_idx >= config_.num_bins) bin_idx = config_.num_bins - 1;
    if (bin_idx < 0) bin_idx = 0;
}

inline double FastGroundSegmentation::getAdaptiveThreshold(double distance) const {
    // 近距离：使用放宽的阈值（近处点云密集，但可能有噪声）
    if (distance < config_.near_distance) {
        // 线性插值：从 th_ground_near 过渡到 th_ground
        double ratio = distance / config_.near_distance;
        return config_.th_ground_near + ratio * (config_.th_ground - config_.th_ground_near);
    }
    // 中距离：使用标准阈值
    if (distance < config_.far_distance) {
        return config_.th_ground;
    }
    // 远距离：线性插值到更宽松的阈值
    double ratio = (distance - config_.far_distance) / (config_.max_range - config_.far_distance);
    ratio = std::min(1.0, std::max(0.0, ratio));
    return config_.th_ground + ratio * (config_.th_ground_far - config_.th_ground);
}

}  // namespace perception
