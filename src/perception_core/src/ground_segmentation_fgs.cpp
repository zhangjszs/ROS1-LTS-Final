#include <perception_core/lidar_cluster_core.hpp>
#include <perception_core/fast_ground_segmentation.hpp>

#include <iostream>

void lidar_cluster::ground_segmentation_fgs_(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                              pcl::PointCloud<PointType>::Ptr &g_not_ground_pc)
{
    g_not_ground_pc->points.clear();

    if (!in_pc || in_pc->points.empty()) {
        g_not_ground_pc->width = 0;
        g_not_ground_pc->height = 1;
        g_not_ground_pc->is_dense = true;
        return;
    }

    // 懒初始化FGS实例
    if (!fgs_) {
        perception::FGSConfig fgs_config;
        fgs_config.num_sectors = config_.fgs.num_sectors;
        fgs_config.num_bins = config_.fgs.num_bins;
        fgs_config.max_range = config_.fgs.max_range;
        fgs_config.min_range = config_.fgs.min_range;
        fgs_config.sensor_height = config_.fgs.sensor_height;
        fgs_config.th_ground = config_.fgs.th_ground;
        fgs_config.th_ground_far = config_.fgs.th_ground_far;
        fgs_config.far_distance = config_.fgs.far_distance;
        // 近距离参数（解决正前方地面分割问题）
        fgs_config.near_distance = config_.fgs.near_distance;
        fgs_config.th_ground_near = config_.fgs.th_ground_near;
        fgs_config.max_slope = config_.fgs.max_slope;
        fgs_config.min_normal_z = config_.fgs.min_normal_z;
        fgs_config.max_height_diff = config_.fgs.max_height_diff;
        // 邻近扇区模型插值（解决稀疏区域问题）
        fgs_config.use_neighbor_model = config_.fgs.use_neighbor_model;

        // 增量线段生长参数 (Zermas 2017)
        fgs_config.max_segments_per_sector = config_.fgs.max_segments_per_sector;
        fgs_config.segment_merge_dist = config_.fgs.segment_merge_dist;

        // 地面点精细化 (Zermas 2017)
        fgs_config.enable_refinement = config_.fgs.enable_refinement;

        // 代表点选择 (Himmelsbach 2010)
        fgs_config.use_lowest_n_mean = config_.fgs.use_lowest_n_mean;
        fgs_config.lowest_n = config_.fgs.lowest_n;

        // 扇区间平滑 (Himmelsbach 2010)
        fgs_config.enable_sector_smoothing = config_.fgs.enable_sector_smoothing;

        // 帧间时序平滑 (抑制闪烁)
        fgs_config.enable_temporal_smoothing = config_.fgs.enable_temporal_smoothing;
        fgs_config.temporal_alpha = config_.fgs.temporal_alpha;

        // 地面判定阈值对称性
        fgs_config.ground_below_factor = config_.fgs.ground_below_factor;

        // temporal alpha 自适应（S弯/快速变向）
        fgs_config.enable_adaptive_alpha = config_.fgs.enable_adaptive_alpha;
        fgs_config.adaptive_alpha_max = config_.fgs.adaptive_alpha_max;
        fgs_config.adaptive_alpha_threshold = config_.fgs.adaptive_alpha_threshold;

        fgs_ = std::make_unique<perception::FastGroundSegmentation>();
        fgs_->configure(fgs_config);
    }

    // 执行FGS地面分割（仅输出非地面点，优化版本）
    fgs_->segmentNonGround(in_pc, g_not_ground_pc);
}
