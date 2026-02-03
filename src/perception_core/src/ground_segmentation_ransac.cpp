#include <perception_core/lidar_cluster_core.hpp>

#include <algorithm>
#include <cmath>

// 地面分割处理（优化 RANSAC）

bool point_cmp(const PointType &a, const PointType &b)
{
    return a.z < b.z;
}

// 获取点所属的距离分区索引
int lidar_cluster::getZoneIndex_(double distance) const
{
    if (zone_boundaries_.empty()) return 0;
    for (size_t i = 0; i < zone_boundaries_.size(); ++i) {
        if (distance < zone_boundaries_[i]) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(zone_boundaries_.size());
}

// 获取自适应距离阈值
double lidar_cluster::getAdaptiveThreshold_(double distance) const
{
    if (!adaptive_threshold_) return th_dist_;
    
    // 线性插值：近处使用 th_dist_，远处放大
    double scale = 1.0 + (distance / max_range_) * (th_dist_far_scale_ - 1.0);
    return th_dist_ * std::min(scale, th_dist_far_scale_);
}

// ground_segmentation_ransac_ 地面分割（优化版）
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
    
    // 1. 按Z轴排序
    pcl::PointCloud<PointType> laserCloudIn, laserCloudIn_org;
    laserCloudIn = laserCloudIn_org = *in_pc;
    std::sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);
    
    // 2. 移除镜面反射点 (z < -sensor_height)
    pcl::PointCloud<PointType>::iterator it = laserCloudIn.points.begin();
    for (size_t i = 0; i < laserCloudIn.points.size(); i++) {
        if (laserCloudIn.points[i].z < -1 * sensor_height_) {
            it++;
        } else {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
    
    if (laserCloudIn.points.empty()) {
        g_not_ground_pc->clear();
        g_not_ground_pc->width = 0;
        g_not_ground_pc->height = 1;
        g_not_ground_pc->is_dense = true;
        return;
    }
    
    // ========== 分区 RANSAC ==========
    if (enable_zone_ && !zone_boundaries_.empty()) {
        // 按距离分区
        size_t num_zones = zone_boundaries_.size() + 1;
        std::vector<pcl::PointCloud<PointType>> zone_clouds(num_zones);
        std::vector<pcl::PointCloud<PointType>> zone_clouds_org(num_zones);
        
        for (const auto& pt : laserCloudIn.points) {
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            int zone_idx = getZoneIndex_(dist);
            zone_clouds[zone_idx].points.push_back(pt);
        }
        for (const auto& pt : laserCloudIn_org.points) {
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            int zone_idx = getZoneIndex_(dist);
            zone_clouds_org[zone_idx].points.push_back(pt);
        }
        
        // 每个分区独立处理
        for (size_t z = 0; z < num_zones; z++) {
            if (zone_clouds[z].points.empty()) continue;
            
            // 计算分区中心距离用于自适应阈值
            double zone_center_dist = (z == 0) ? zone_boundaries_[0] / 2.0 :
                                      (z == num_zones - 1) ? zone_boundaries_.back() + 5.0 :
                                      (zone_boundaries_[z-1] + zone_boundaries_[z]) / 2.0;
            double th_dist_zone = getAdaptiveThreshold_(zone_center_dist);
            
            // 提取种子点
            pcl::PointCloud<PointType> zone_seeds;
            double sum = 0;
            int cnt = 0;
            for (size_t i = 0; i < zone_clouds[z].points.size() && cnt < num_lpr_; i++) {
                sum += zone_clouds[z].points[i].z;
                cnt++;
            }
            double lpr_height = cnt != 0 ? sum / cnt : 0;
            for (const auto& pt : zone_clouds[z].points) {
                if (pt.z < lpr_height + th_seeds_) {
                    zone_seeds.points.push_back(pt);
                }
            }
            
            if (zone_seeds.points.empty()) {
                // 无种子点，视为全非地面（而非地面！）
                for (const auto& pt : zone_clouds_org[z].points) {
                    g_not_ground_pc->points.push_back(pt);
                }
                continue;
            }
            
            // 迭代拟合
            pcl::PointCloud<PointType> zone_ground = zone_seeds;
            pcl::PointCloud<PointType> zone_not_ground;
            
            for (int iter = 0; iter < num_iter_; iter++) {
                if (zone_ground.points.empty()) break;
                
                // SVD 计算平面
                Eigen::Matrix3f cov;
                Eigen::Vector4f pc_mean;
                pcl::computeMeanAndCovarianceMatrix(zone_ground, cov, pc_mean);
                Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
                Eigen::Vector3f normal = svd.matrixU().col(2);
                
                // ===== 法向量约束 =====
                if (std::abs(normal(2)) < min_normal_z_) {
                    // 法向量过于水平，使用默认垂直地面
                    normal << 0, 0, 1;
                }
                
                Eigen::Vector3f seeds_mean = pc_mean.head<3>();
                float d = -(normal.transpose() * seeds_mean)(0, 0);
                
                zone_ground.points.clear();
                zone_not_ground.points.clear();
                
                // 分类
                for (const auto& pt : zone_clouds_org[z].points) {
                    Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
                    float dist_to_plane = std::abs(normal.dot(pt_vec) + d);
                    
                    if (dist_to_plane < th_dist_zone) {
                        zone_ground.points.push_back(pt);
                    } else {
                        zone_not_ground.points.push_back(pt);
                    }
                }
                
                // ===== 渐进式迭代 =====
                if (progressive_iteration_ && iter > 0) {
                    // 只保留边界点继续迭代
                    pcl::PointCloud<PointType> boundary_points;
                    for (const auto& pt : zone_ground.points) {
                        Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
                        float dist_to_plane = std::abs(normal.dot(pt_vec) + d);
                        // 边界点：距离在 [0.5*th_dist, th_dist] 之间
                        if (dist_to_plane > 0.5 * th_dist_zone) {
                            boundary_points.points.push_back(pt);
                        }
                    }
                    // 如果边界点太少，提前终止
                    if (boundary_points.points.size() < 10) break;
                }
            }
            
            // 合并非地面点
            for (const auto& pt : zone_not_ground.points) {
                g_not_ground_pc->points.push_back(pt);
            }
        }
    } else {
        // ========== 原始全局 RANSAC ==========
        extract_initial_seeds_(laserCloudIn);
        
        if (g_seeds_pc->points.empty()) {
            g_not_ground_pc->clear();
            g_not_ground_pc->width = 0;
            g_not_ground_pc->height = 1;
            g_not_ground_pc->is_dense = true;
            return;
        }
        
        g_ground_pc = g_seeds_pc;
        
        for (int i = 0; i < num_iter_; i++) {
            if (g_ground_pc->points.empty()) break;
            
            estimate_plane_();
            g_ground_pc->clear();
            g_not_ground_pc->clear();
            
            MatrixXf points(laserCloudIn_org.points.size(), 3);
            int j = 0;
            for (const auto& p : laserCloudIn_org.points) {
                points.row(j++) << p.x, p.y, p.z;
            }
            
            VectorXf result = points * normal_;
            for (int r = 0; r < result.rows(); r++) {
                // 自适应阈值
                double dist = std::sqrt(laserCloudIn_org[r].x * laserCloudIn_org[r].x + 
                                       laserCloudIn_org[r].y * laserCloudIn_org[r].y);
                double th = getAdaptiveThreshold_(dist);
                
                if (std::abs(result[r] + d_) < th) {
                    g_ground_pc->points.push_back(laserCloudIn_org[r]);
                } else {
                    g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
                }
            }
        }
    }
    
    g_ground_pc->width = g_ground_pc->points.size();
    g_ground_pc->height = 1;
    g_ground_pc->is_dense = in_pc->is_dense;
    g_not_ground_pc->width = g_not_ground_pc->points.size();
    g_not_ground_pc->height = 1;
    g_not_ground_pc->is_dense = in_pc->is_dense;
}

// extract_initial_seeds_ 提取初始种子点
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

// estimate_plane_ 估计平面参数
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
    
    // ===== 法向量约束 =====
    if (std::abs(normal_(2)) < min_normal_z_) {
        normal_ << 0, 0, 1;
    }
    
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    th_dist_d_ = th_dist_ - d_;
}
