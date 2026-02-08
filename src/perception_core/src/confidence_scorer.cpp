#include "perception_core/confidence_scorer.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace perception {

void ConfidenceScorer::setConfig(const Config& config) {
    config_ = config;

    // Configure model fitter
    ConeModelFitter::Config fitter_config;
    fitter_config.enable = config.enable_model_fitting;
    model_fitter_.setConfig(fitter_config);
}

double ConfidenceScorer::computeConfidence(const ClusterFeatures& features) {
    if (features.point_count < 1) {
        return 0.0;
    }

    double size_score = scoreSizeConstraints(features);
    double shape_score = scoreShapeConstraints(features);
    double density_score = scoreDensityConstraints(features);
    double intensity_score = scoreIntensityConstraints(features);
    double position_score = scorePositionConstraints(features);

    double confidence = config_.weight_size * size_score
                      + config_.weight_shape * shape_score
                      + config_.weight_density * density_score
                      + config_.weight_intensity * intensity_score
                      + config_.weight_position * position_score;

    return std::max(0.0, std::min(1.0, confidence));
}

double ConfidenceScorer::computeConfidenceWithFitting(
    const ClusterFeatures& features,
    const pcl::PointCloud<PointType>::Ptr& cluster) {

    // Get base confidence from features
    double confidence = computeConfidence(features);

    // Apply model fitting if enabled
    if (config_.enable_model_fitting && cluster &&
        features.point_count >= 8) {

        ConeModelFitter::FitResult fit = model_fitter_.fitConeModel(cluster);

        if (fit.is_valid) {
            // Good fit: boost confidence
            double fit_quality = 1.0 - std::min(1.0, fit.fit_error);
            confidence += config_.model_fit_bonus * fit_quality;
        } else {
            // Failed to fit cone model: penalize
            confidence -= config_.model_fit_penalty;
        }
    }

    return std::max(0.0, std::min(1.0, confidence));
}

double ConfidenceScorer::scoreSizeConstraints(const ClusterFeatures& f) {
    // 改进：使用软评分而非硬阈值
    // 高度评分：高斯分布，中心在理想高度
    double ideal_height = (config_.min_height + config_.max_height) / 2.0;
    double height_sigma = (config_.max_height - config_.min_height) / 4.0;
    double height_score = std::exp(-std::pow(f.height - ideal_height, 2) / (2 * height_sigma * height_sigma));

    // 面积评分：高斯分布
    double ideal_area = (config_.min_area + config_.max_area) / 2.0;
    double area_sigma = (config_.max_area - config_.min_area) / 4.0;
    double area_score = std::exp(-std::pow(f.area - ideal_area, 2) / (2 * area_sigma * area_sigma));

    // 超出范围惩罚
    if (f.height < config_.min_height * 0.5 || f.height > config_.max_height * 2.0) {
        height_score *= 0.1;
    }
    if (f.area < config_.min_area * 0.5 || f.area > config_.max_area * 2.0) {
        area_score *= 0.1;
    }

    return (height_score + area_score) / 2.0;
}

double ConfidenceScorer::scoreShapeConstraints(const ClusterFeatures& f) {
    // 改进：使用软评分
    // 纵横比评分：sigmoid函数，超过阈值后快速上升
    double aspect_score = 1.0 / (1.0 + std::exp(-5.0 * (f.aspect_ratio - config_.min_aspect_ratio)));

    // 垂直度评分：线性映射到[0,1]
    // min_verticality <= 0.5 时，避免分母接近0导致数值不稳定。
    double verticality_score = 0.0;
    if (config_.min_verticality <= 0.5001) {
        verticality_score = std::max(0.0, std::min(1.0, f.verticality_score));
    } else {
        const double denom = config_.min_verticality - 0.5;
        verticality_score = std::max(0.0, std::min(1.0,
            (f.verticality_score - 0.5) / denom));
    }

    return (aspect_score + verticality_score) / 2.0;
}

double ConfidenceScorer::scoreDensityConstraints(const ClusterFeatures& f) {
    // 改进：距离自适应密度评分
    // 密度要求随距离线性降低
    double distance_factor = std::max(0.0, std::min(1.0,
        (f.distance_to_sensor - config_.distance_threshold) /
        (20.0 - config_.distance_threshold)));

    double min_density = config_.min_density_near * (1.0 - distance_factor) +
                         config_.min_density_far * distance_factor;

    // 软评分：密度越高分数越高，但有上限
    double density_ratio = f.point_density / min_density;
    double score = 1.0 - std::exp(-density_ratio);

    return std::max(0.0, std::min(1.0, score));
}

double ConfidenceScorer::scoreIntensityConstraints(const ClusterFeatures& f) {
    double score = 1.0;

    if (f.intensity_mean < config_.min_intensity_mean) {
        score -= 0.15;
    }

    return std::max(0.0, score);
}

double ConfidenceScorer::scorePositionConstraints(const ClusterFeatures& f) {
    double score = 1.0;

    if (std::fabs(f.ground_height) > config_.max_box_altitude) {
        score -= 0.2;
    }

    return std::max(0.0, score);
}

double ConfidenceScorer::scoreTrackSemanticConstraints(
    const pcl::PointXYZ& centroid,
    const std::vector<pcl::PointXYZ>& all_centroids,
    int self_index) {

    const auto& ts = config_.track_semantic;
    if (!ts.enable || all_centroids.size() < 2) {
        return 0.5;  // neutral score when disabled
    }

    double cx = centroid.x;
    double cy = centroid.y;

    // Find nearest neighbor and nearest opposite-side neighbor
    double min_dist = std::numeric_limits<double>::max();
    double min_opposite_dist = std::numeric_limits<double>::max();
    int neighbor_count = 0;

    for (size_t i = 0; i < all_centroids.size(); ++i) {
        if (static_cast<int>(i) == self_index) continue;

        double dx = all_centroids[i].x - cx;
        double dy = all_centroids[i].y - cy;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > ts.isolation_radius) continue;

        neighbor_count++;
        if (dist < min_dist) {
            min_dist = dist;
        }

        // Check if on opposite side (different sign of y in base_link frame)
        bool opposite_side = (all_centroids[i].y * cy < 0.0);
        if (opposite_side && dist < min_opposite_dist) {
            min_opposite_dist = dist;
        }
    }

    // Sub-score 1: Spacing score (nearest same-side neighbor within expected spacing)
    double spacing_score = 0.0;
    if (min_dist < std::numeric_limits<double>::max()) {
        double spacing_err = std::abs(min_dist - ts.expected_cone_spacing);
        spacing_score = std::max(0.0, 1.0 - spacing_err / ts.spacing_tolerance);
    }

    // Sub-score 2: Width score (nearest opposite-side neighbor at expected track width)
    double width_score = 0.0;
    if (min_opposite_dist < std::numeric_limits<double>::max()) {
        double width_err = std::abs(min_opposite_dist - ts.expected_track_width);
        width_score = std::max(0.0, 1.0 - width_err / ts.width_tolerance);
    }

    // Sub-score 3: Isolation penalty (no neighbors → low score)
    double isolation_score = 0.0;
    if (neighbor_count >= 3) {
        isolation_score = 1.0;
    } else if (neighbor_count >= 1) {
        isolation_score = 0.5;
    }

    return (spacing_score + width_score + isolation_score) / 3.0;
}

double ConfidenceScorer::computeConfidenceWithContext(
    const ClusterFeatures& features,
    const pcl::PointCloud<PointType>::Ptr& cluster,
    const std::vector<pcl::PointXYZ>& all_centroids,
    int self_index) {

    // Get base confidence with model fitting
    double confidence = computeConfidenceWithFitting(features, cluster);

    // Add track semantic dimension if enabled
    if (config_.track_semantic.enable && config_.track_semantic.weight > 0.0 &&
        self_index >= 0 && self_index < static_cast<int>(all_centroids.size())) {

        double semantic_score = scoreTrackSemanticConstraints(
            all_centroids[self_index], all_centroids, self_index);

        // Blend: reduce other weights proportionally to make room for semantic weight
        double other_weight = 1.0 - config_.track_semantic.weight;
        confidence = other_weight * confidence + config_.track_semantic.weight * semantic_score;
    }

    return std::max(0.0, std::min(1.0, confidence));
}

}  // namespace perception
