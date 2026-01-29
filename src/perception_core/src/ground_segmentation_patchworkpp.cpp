#include <perception_core/lidar_cluster_core.hpp>

#include <Eigen/Dense>

#include <iostream>

void lidar_cluster::ground_segmentation_dispatch_(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                                  pcl::PointCloud<PointType>::Ptr &g_not_ground_pc)
{
    if (ground_method_ == "patchworkpp") {
        ground_segmentation_patchworkpp_(in_pc, g_not_ground_pc);
        return;
    }

    if (ground_method_ != "ransac") {
        std::cerr << "Unknown ground_method: " << ground_method_
                  << ", valid: ransac|patchworkpp, fallback to ransac"
                  << std::endl;
    }
    ground_segmentation_ransac_(in_pc, g_not_ground_pc);
}

void lidar_cluster::ground_segmentation_patchworkpp_(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                                     pcl::PointCloud<PointType>::Ptr &g_not_ground_pc)
{
    g_not_ground_pc->points.clear();
    if (!in_pc || in_pc->points.empty()) {
        return;
    }

    if (!patchwork_) {
        patchwork_ = std::make_unique<patchwork::PatchWorkpp>(patchwork_params_);
    }

    Eigen::MatrixXf cloud(static_cast<int>(in_pc->points.size()), 4);
    for (size_t i = 0; i < in_pc->points.size(); i++) {
        const auto &p = in_pc->points[i];
        cloud(static_cast<int>(i), 0) = p.x;
        cloud(static_cast<int>(i), 1) = p.y;
        cloud(static_cast<int>(i), 2) = p.z;
        cloud(static_cast<int>(i), 3) = p.intensity;
    }

    patchwork_->estimateGround(cloud);
    const Eigen::VectorXi nonground_idx = patchwork_->getNongroundIndices();

    g_not_ground_pc->points.reserve(static_cast<size_t>(nonground_idx.size()));
    for (int i = 0; i < nonground_idx.size(); i++) {
        const int idx = nonground_idx(i);
        if (idx < 0 || idx >= static_cast<int>(in_pc->points.size())) {
            continue;
        }
        g_not_ground_pc->points.push_back(in_pc->points[static_cast<size_t>(idx)]);
    }

    g_not_ground_pc->width = g_not_ground_pc->points.size();
    g_not_ground_pc->height = 1;
    g_not_ground_pc->is_dense = in_pc->is_dense;
}
