#include "perception_core/confidence_scorer.hpp"
#include <cmath>
#include <algorithm>

namespace perception {

void ConfidenceScorer::setConfig(const Config& config) {
    config_ = config;

    // Configure model fitter
    ConeModelFitter::Config fitter_config;
    fitter_config.enable = config.enable_model_fitting;
    model_fitter_.setConfig(fitter_config);
}

double ConfidenceScorer::computeConfidence(const ClusterFeatures& features) {
    // 移除严格的形状检查，只要有点就接受
    if (features.point_count < 2) {
        return 0.0;
    }

    // 极度宽松的评分，基本上所有簇都能通过
    double size_score = scoreSizeConstraints(features);
    double shape_score = 1.0;  // 不再检查形状
    double density_score = 1.0;  // 不再检查密度
    double intensity_score = 1.0;  // 不再检查强度
    double position_score = 1.0;  // 不再检查位置

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

    if (confidence < 0) {
        return confidence;  // Failed basic shape check
    }

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
    // 几乎不做任何限制，只要不是极端异常就接受
    return 1.0;
}

double ConfidenceScorer::scoreShapeConstraints(const ClusterFeatures& f) {
    double score = 1.0;

    // 非常宽松的形状约束
    if (f.aspect_ratio < config_.min_aspect_ratio) {
        score -= 0.05;  // 轻微惩罚
    }

    if (f.verticality_score < config_.min_verticality) {
        score -= 0.05;  // 轻微惩罚
    }

    return score;
}

double ConfidenceScorer::scoreDensityConstraints(const ClusterFeatures& f) {
    double score = 1.0;

    // 非常宽松的密度约束
    if (f.point_count < 2) {
        score -= 0.1;  // 只有点数极少时才惩罚
    }

    return score;
}

double ConfidenceScorer::scoreIntensityConstraints(const ClusterFeatures& f) {
    double score = 1.0;
    // 不对强度进行惩罚，只作为可选加分项
    return score;
}

double ConfidenceScorer::scorePositionConstraints(const ClusterFeatures& f) {
    double score = 1.0;

    // 非常宽松的位置约束
    if (std::fabs(f.ground_height) > config_.max_box_altitude) {
        score -= 0.1;  // 轻微惩罚
    }

    return score;
}

}  // namespace perception
