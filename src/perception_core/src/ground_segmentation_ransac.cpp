#include <perception_core/lidar_cluster_core.hpp>

#include <algorithm>
#include <cmath>

// 地面分割处理（优化 RANSAC v2）
// 优化点：
// 1. 减少内存拷贝 - 使用索引而非复制点云
// 2. 单次SVD快速模式 - 简单场景跳过迭代
// 3. 向量化距离计算 - Eigen批量运算
// 4. 预分配内存 - 减少动态分配
// 5. 早期终止 - 收敛检测

namespace {
// 内联比较函数，避免函数调用开销
inline bool point_cmp_z(const PointType &a, const PointType &b) {
    return a.z < b.z;
}

// 快速计算水平距离的平方（避免sqrt）
inline float dist_sq_xy(const PointType& pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

// 快速计算水平距离
inline float dist_xy(const PointType& pt) {
    return std::sqrt(dist_sq_xy(pt));
}
}  // namespace

// 获取点所属的距离分区索引（优化：使用二分查找）
int lidar_cluster::getZoneIndex_(double distance) const
{
    if (zone_boundaries_.empty()) return 0;

    // 对于少量分区，线性查找更快
    if (zone_boundaries_.size() <= 4) {
        for (size_t i = 0; i < zone_boundaries_.size(); ++i) {
            if (distance < zone_boundaries_[i]) {
                return static_cast<int>(i);
            }
        }
        return static_cast<int>(zone_boundaries_.size());
    }

    // 多分区使用二分查找
    auto it = std::lower_bound(zone_boundaries_.begin(), zone_boundaries_.end(), distance);
    return static_cast<int>(it - zone_boundaries_.begin());
}

// 获取自适应距离阈值（优化：预计算常量）
double lidar_cluster::getAdaptiveThreshold_(double distance) const
{
    if (!adaptive_threshold_) return th_dist_;

    // 线性插值：近处使用 th_dist_，远处放大
    // 优化：避免除法，使用预计算的倒数
    const double inv_max_range = 1.0 / max_range_;
    double scale = 1.0 + (distance * inv_max_range) * (th_dist_far_scale_ - 1.0);
    return th_dist_ * std::min(scale, th_dist_far_scale_);
}

// 优化版地面分割
void lidar_cluster::ground_segmentation_ransac_(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                                pcl::PointCloud<PointType>::Ptr &g_not_ground_pc)
{
    g_not_ground_pc->points.clear();

    // 空云检查
    if (!in_pc || in_pc->points.empty()) {
        g_not_ground_pc->width = 0;
        g_not_ground_pc->height = 1;
        g_not_ground_pc->is_dense = true;
        return;
    }

    const size_t n_points = in_pc->points.size();

    // 预分配输出空间
    g_not_ground_pc->points.reserve(n_points / 2);

    // ========== 优化1：使用索引数组代替复制点云 ==========
    std::vector<size_t> sorted_indices(n_points);
    for (size_t i = 0; i < n_points; ++i) {
        sorted_indices[i] = i;
    }

    // 按Z轴排序索引
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&in_pc](size_t a, size_t b) {
                  return in_pc->points[a].z < in_pc->points[b].z;
              });

    // 找到第一个有效点（z >= -sensor_height）
    size_t valid_start = 0;
    const float neg_sensor_height = -sensor_height_;
    for (size_t i = 0; i < n_points; ++i) {
        if (in_pc->points[sorted_indices[i]].z >= neg_sensor_height) {
            valid_start = i;
            break;
        }
    }

    if (valid_start >= n_points) {
        g_not_ground_pc->width = 0;
        g_not_ground_pc->height = 1;
        g_not_ground_pc->is_dense = true;
        return;
    }

    // ========== 分区 RANSAC ==========
    if (enable_zone_ && !zone_boundaries_.empty()) {
        const size_t num_zones = zone_boundaries_.size() + 1;

        // 预分配分区索引数组
        std::vector<std::vector<size_t>> zone_indices(num_zones);
        for (auto& zi : zone_indices) {
            zi.reserve(n_points / num_zones);
        }

        // 单次遍历分配到各分区
        for (size_t i = valid_start; i < n_points; ++i) {
            const size_t idx = sorted_indices[i];
            const PointType& pt = in_pc->points[idx];
            float dist = dist_xy(pt);
            int zone_idx = getZoneIndex_(dist);
            zone_indices[zone_idx].push_back(idx);
        }

        // 每个分区独立处理
        for (size_t z = 0; z < num_zones; z++) {
            const auto& indices = zone_indices[z];
            if (indices.empty()) continue;

            // 计算分区中心距离用于自适应阈值
            double zone_center_dist = (z == 0) ? zone_boundaries_[0] / 2.0 :
                                      (z == num_zones - 1) ? zone_boundaries_.back() + 5.0 :
                                      (zone_boundaries_[z-1] + zone_boundaries_[z]) / 2.0;
            float th_dist_zone = static_cast<float>(getAdaptiveThreshold_(zone_center_dist));

            // ========== 优化2：快速种子点提取 ==========
            // 找到该分区内Z值最小的点（已排序）
            double sum_z = 0;
            int seed_count = 0;
            std::vector<size_t> seed_indices;
            seed_indices.reserve(std::min(static_cast<size_t>(num_lpr_ * 10), indices.size()));

            // 计算最低点平均高度
            for (size_t i = 0; i < indices.size() && seed_count < num_lpr_; ++i) {
                sum_z += in_pc->points[indices[i]].z;
                seed_count++;
            }

            if (seed_count == 0) {
                // 无种子点，全部视为非地面
                for (size_t idx : indices) {
                    g_not_ground_pc->points.push_back(in_pc->points[idx]);
                }
                continue;
            }

            float lpr_height = static_cast<float>(sum_z / seed_count);
            float seed_threshold = lpr_height + static_cast<float>(th_seeds_);

            // 收集种子点
            for (size_t idx : indices) {
                if (in_pc->points[idx].z < seed_threshold) {
                    seed_indices.push_back(idx);
                }
            }

            if (seed_indices.empty()) {
                for (size_t idx : indices) {
                    g_not_ground_pc->points.push_back(in_pc->points[idx]);
                }
                continue;
            }

            // ========== 优化3：单次/少次迭代模式 ==========
            Eigen::Vector3f normal;
            float d;

            // 使用种子点计算初始平面
            {
                Eigen::Vector3f mean = Eigen::Vector3f::Zero();
                for (size_t idx : seed_indices) {
                    const PointType& pt = in_pc->points[idx];
                    mean += Eigen::Vector3f(pt.x, pt.y, pt.z);
                }
                mean /= static_cast<float>(seed_indices.size());

                // 计算协方差矩阵
                Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
                for (size_t idx : seed_indices) {
                    const PointType& pt = in_pc->points[idx];
                    Eigen::Vector3f diff(pt.x - mean.x(), pt.y - mean.y(), pt.z - mean.z());
                    cov += diff * diff.transpose();
                }
                cov /= static_cast<float>(seed_indices.size());

                // SVD求解法向量
                Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU);
                normal = svd.matrixU().col(2);

                // 法向量约束
                if (std::abs(normal(2)) < min_normal_z_) {
                    normal = Eigen::Vector3f(0, 0, 1);
                }

                d = -normal.dot(mean);
            }

            // ========== 优化4：向量化分类 ==========
            // 对于小分区或简单场景，单次拟合即可
            int actual_iters = (indices.size() < 100 || num_iter_ <= 1) ? 1 : std::min(num_iter_, 3);

            std::vector<size_t> ground_indices;
            ground_indices.reserve(indices.size());

            for (int iter = 0; iter < actual_iters; iter++) {
                ground_indices.clear();

                // 批量计算点到平面距离
                for (size_t idx : indices) {
                    const PointType& pt = in_pc->points[idx];
                    float dist_to_plane = std::abs(normal.x() * pt.x +
                                                   normal.y() * pt.y +
                                                   normal.z() * pt.z + d);

                    if (dist_to_plane < th_dist_zone) {
                        ground_indices.push_back(idx);
                    }
                }

                // 早期终止：如果地面点数量稳定
                if (iter > 0 && ground_indices.size() == seed_indices.size()) {
                    break;
                }

                // 更新平面（如果还有迭代）
                if (iter < actual_iters - 1 && !ground_indices.empty()) {
                    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
                    for (size_t idx : ground_indices) {
                        const PointType& pt = in_pc->points[idx];
                        mean += Eigen::Vector3f(pt.x, pt.y, pt.z);
                    }
                    mean /= static_cast<float>(ground_indices.size());

                    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
                    for (size_t idx : ground_indices) {
                        const PointType& pt = in_pc->points[idx];
                        Eigen::Vector3f diff(pt.x - mean.x(), pt.y - mean.y(), pt.z - mean.z());
                        cov += diff * diff.transpose();
                    }
                    cov /= static_cast<float>(ground_indices.size());

                    Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU);
                    normal = svd.matrixU().col(2);

                    if (std::abs(normal(2)) < min_normal_z_) {
                        normal = Eigen::Vector3f(0, 0, 1);
                    }

                    d = -normal.dot(mean);
                    seed_indices = ground_indices;
                }
            }

            // 输出非地面点
            // 使用布尔标记数组代替排序+二分查找 (O(n) vs O(n log n))
            std::vector<bool> is_ground(n_points, false);
            for (size_t idx : ground_indices) {
                is_ground[idx] = true;
            }
            for (size_t idx : indices) {
                if (!is_ground[idx]) {
                    g_not_ground_pc->points.push_back(in_pc->points[idx]);
                }
            }
        }
    } else {
        // ========== 全局 RANSAC（优化版）==========
        // 提取种子点
        double sum_z = 0;
        int seed_count = 0;
        std::vector<size_t> seed_indices;
        seed_indices.reserve(n_points / 4);

        for (size_t i = valid_start; i < n_points && seed_count < num_lpr_; ++i) {
            sum_z += in_pc->points[sorted_indices[i]].z;
            seed_count++;
        }

        if (seed_count == 0) {
            // 全部视为非地面
            for (size_t i = valid_start; i < n_points; ++i) {
                g_not_ground_pc->points.push_back(in_pc->points[sorted_indices[i]]);
            }
            g_not_ground_pc->width = g_not_ground_pc->points.size();
            g_not_ground_pc->height = 1;
            g_not_ground_pc->is_dense = in_pc->is_dense;
            return;
        }

        float lpr_height = static_cast<float>(sum_z / seed_count);
        float seed_threshold = lpr_height + static_cast<float>(th_seeds_);

        for (size_t i = valid_start; i < n_points; ++i) {
            size_t idx = sorted_indices[i];
            if (in_pc->points[idx].z < seed_threshold) {
                seed_indices.push_back(idx);
            }
        }

        if (seed_indices.empty()) {
            for (size_t i = valid_start; i < n_points; ++i) {
                g_not_ground_pc->points.push_back(in_pc->points[sorted_indices[i]]);
            }
            g_not_ground_pc->width = g_not_ground_pc->points.size();
            g_not_ground_pc->height = 1;
            g_not_ground_pc->is_dense = in_pc->is_dense;
            return;
        }

        // 迭代拟合
        Eigen::Vector3f normal;
        float d;
        std::vector<size_t> ground_indices = seed_indices;

        for (int iter = 0; iter < num_iter_; iter++) {
            if (ground_indices.empty()) break;

            // 计算平面
            Eigen::Vector3f mean = Eigen::Vector3f::Zero();
            for (size_t idx : ground_indices) {
                const PointType& pt = in_pc->points[idx];
                mean += Eigen::Vector3f(pt.x, pt.y, pt.z);
            }
            mean /= static_cast<float>(ground_indices.size());

            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (size_t idx : ground_indices) {
                const PointType& pt = in_pc->points[idx];
                Eigen::Vector3f diff(pt.x - mean.x(), pt.y - mean.y(), pt.z - mean.z());
                cov += diff * diff.transpose();
            }
            cov /= static_cast<float>(ground_indices.size());

            Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU);
            normal = svd.matrixU().col(2);

            if (std::abs(normal(2)) < min_normal_z_) {
                normal = Eigen::Vector3f(0, 0, 1);
            }

            d = -normal.dot(mean);

            // 分类
            size_t prev_ground_count = ground_indices.size();
            ground_indices.clear();

            for (size_t i = valid_start; i < n_points; ++i) {
                size_t idx = sorted_indices[i];
                const PointType& pt = in_pc->points[idx];

                // 自适应阈值
                float dist = dist_xy(pt);
                float th = static_cast<float>(getAdaptiveThreshold_(dist));

                float dist_to_plane = std::abs(normal.x() * pt.x +
                                               normal.y() * pt.y +
                                               normal.z() * pt.z + d);

                if (dist_to_plane < th) {
                    ground_indices.push_back(idx);
                }
            }

            // 早期终止
            if (ground_indices.size() == prev_ground_count) {
                break;
            }
        }

        // 输出非地面点
        // 使用布尔标记数组代替排序+二分查找 (O(n) vs O(n log n))
        std::vector<bool> is_ground(n_points, false);
        for (size_t idx : ground_indices) {
            is_ground[idx] = true;
        }
        for (size_t i = valid_start; i < n_points; ++i) {
            size_t idx = sorted_indices[i];
            if (!is_ground[idx]) {
                g_not_ground_pc->points.push_back(in_pc->points[idx]);
            }
        }
    }

    // 设置输出点云属性
    g_not_ground_pc->width = g_not_ground_pc->points.size();
    g_not_ground_pc->height = 1;
    g_not_ground_pc->is_dense = in_pc->is_dense;

    // 更新g_ground_pc（如果需要）
    if (g_ground_pc) {
        g_ground_pc->width = g_ground_pc->points.size();
        g_ground_pc->height = 1;
        g_ground_pc->is_dense = in_pc->is_dense;
    }
}

// extract_initial_seeds_ 提取初始种子点（保留兼容性）
void lidar_cluster::extract_initial_seeds_(const pcl::PointCloud<PointType> &p_sorted)
{
    double sum = 0;
    int cnt = 0;
    for (size_t i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;
    g_seeds_pc->clear();
    for (const auto& pt : p_sorted.points) {
        if (pt.z < lpr_height + th_seeds_) {
            g_seeds_pc->points.push_back(pt);
        }
    }
}

// estimate_plane_ 估计平面参数（保留兼容性）
void lidar_cluster::estimate_plane_(void)
{
    if (!g_ground_pc || g_ground_pc->points.empty()) {
        return;
    }

    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    normal_ = (svd.matrixU().col(2));

    // 法向量约束
    if (std::abs(normal_(2)) < min_normal_z_) {
        normal_ << 0, 0, 1;
    }

    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    th_dist_d_ = th_dist_ - d_;
}
