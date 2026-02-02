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
    if (features.point_count < 2) {
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
    double score = 1.0;

    if (f.height < config_.min_height || f.height > config_.max_height) {
        score -= 0.3;
    }

    if (f.area < config_.min_area || f.area > config_.max_area) {
        score -= 0.3;
    }

    return std::max(0.0, score);
}

double ConfidenceScorer::scoreShapeConstraints(const ClusterFeatures& f) {
    double score = 1.0;

    if (f.aspect_ratio < config_.min_aspect_ratio) {
        score -= 0.2;
    }

    if (f.verticality_score < config_.min_verticality) {
        score -= 0.2;
    }

    return std::max(0.0, score);
}

double ConfidenceScorer::scoreDensityConstraints(const ClusterFeatures& f) {
    double score = 1.0;

    double min_density = f.distance_to_sensor < config_.distance_threshold ?
                        config_.min_density_near : config_.min_density_far;

    if (f.point_density < min_density) {
        score -= 0.3;
    }

    return std::max(0.0, score);
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

}  // namespace perception
