#include "perception_core/confidence_scorer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perception {

void ConfidenceScorer::setConfig(const Config& config) {
  config_ = config;

  // Configure model fitter
  ConeModelFitter::Config fitter_config;
  fitter_config.enable = config.enable_model_fitting;
  fitter_config.ransac_iterations = 100;
  fitter_config.min_points_for_fitting = 5;
  fitter_config.use_pca_axis_check = true;
  fitter_config.use_least_squares_refinement = true;
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

  double confidence = config_.weight_size * size_score + config_.weight_shape * shape_score +
                      config_.weight_density * density_score +
                      config_.weight_intensity * intensity_score +
                      config_.weight_position * position_score;

  return std::max(0.0, std::min(1.0, confidence));
}

double ConfidenceScorer::computeConfidenceWithFitting(
    const ClusterFeatures& features, const pcl::PointCloud<PointType>::Ptr& cluster) {
  // Get base confidence from features
  double confidence = computeConfidence(features);

  // Apply model fitting if enabled
  // Lowered threshold from 8 to 5: SOTA 3D cone fitting (PCA + LS) works with sparse clusters
  if (config_.enable_model_fitting && cluster && features.point_count >= 5) {
    fitting_calls_total_++;

    // Skip model fitting for far sparse clusters or very low base confidence
    bool skip_far_sparse = (features.distance_to_sensor > 30.0 && features.point_count < 8);
    bool skip_low_confidence = (confidence < 0.25);
    if (skip_far_sparse || skip_low_confidence) {
      fitting_skipped_count_++;
      return std::max(0.0, std::min(1.0, confidence));
    }

    ConeModelFitter::FitResult fit = model_fitter_.fitConeModel(cluster);

    if (fit.is_valid) {
      fitting_success_count_++;
      // Good fit: boost confidence based on 3D geometric error and cone angle consistency
      double fit_quality = 1.0 - std::min(1.0, fit.fit_error);

      // Bonus for cone angle close to expected (~17.6 deg for FS standard cone)
      double angle_quality = 1.0;
      if (fit.cone_angle > 1e-6) {
        double expected = 0.307;  // ~17.6 degrees
        double angle_diff = std::fabs(fit.cone_angle - expected);
        angle_quality = std::max(0.0, 1.0 - angle_diff / 0.209);  // tolerance ~12 deg
      }

      double combined_quality = 0.7 * fit_quality + 0.3 * angle_quality;
      confidence += config_.model_fit_bonus * combined_quality;
    } else {
      fitting_fail_count_++;
      // Failed to fit cone model: penalize (but less aggressively for sparse clusters)
      double penalty = config_.model_fit_penalty;
      if (features.point_count < 8) {
        penalty *= 0.3;  // Reduce penalty for sparse clusters where fitting is harder
      }
      confidence -= penalty;
    }
  }

  return std::max(0.0, std::min(1.0, confidence));
}

double ConfidenceScorer::scoreSizeConstraints(const ClusterFeatures& f) {
  // 改进：使用软评分而非硬阈值
  // 高度评分：高斯分布，中心在理想高度
  double ideal_height = (config_.min_height + config_.max_height) / 2.0;
  double height_sigma = (config_.max_height - config_.min_height) / 4.0;
  double height_score =
      std::exp(-std::pow(f.height - ideal_height, 2) / (2 * height_sigma * height_sigma));

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
    verticality_score = std::max(0.0, std::min(1.0, (f.verticality_score - 0.5) / denom));
  }

  // 线性度惩罚：linearity > max_linearity → 强线性结构（墙边、栏杆），大幅降分
  double linearity_penalty = 1.0;
  if (config_.max_linearity > 0.0 && f.linearity > config_.max_linearity) {
    // 超过阈值越多，惩罚越重（线性衰减到0.1）
    double excess = (f.linearity - config_.max_linearity) / (1.0 - config_.max_linearity);
    linearity_penalty = std::max(0.1, 1.0 - 0.9 * excess);
  }

  return (aspect_score + verticality_score) / 2.0 * linearity_penalty;
}

double ConfidenceScorer::scoreDensityConstraints(const ClusterFeatures& f) {
  // 改进：距离自适应密度评分
  // 密度要求随距离线性降低
  double distance_factor =
      std::max(0.0, std::min(1.0, (f.distance_to_sensor - config_.distance_threshold) /
                                      (20.0 - config_.distance_threshold)));

  double min_density = config_.min_density_near * (1.0 - distance_factor) +
                       config_.min_density_far * distance_factor;

  // 软评分：密度越高分数越高，但有上限
  double density_ratio = f.point_density / min_density;
  double score = 1.0 - std::exp(-density_ratio);

  return std::max(0.0, std::min(1.0, score));
}

double ConfidenceScorer::scoreIntensityConstraints(const ClusterFeatures& f) {
  // 比例评分：强度越高分数越高，而非简单的二元阈值
  double ratio = f.intensity_mean / config_.min_intensity_mean;
  double raw_score = std::min(1.0, ratio);
  // 低于阈值的额外惩罚
  if (ratio < 1.0) {
    raw_score *= 0.85;
  }

  // Range-compensated intensity bonus (I * R^2)
  // SOTA: distance-invariant retroreflector detection (Heinzler et al. 2019)
  // Helps detect distant cones that would otherwise fall below raw threshold
  double comp_threshold = config_.min_intensity_mean * 100.0;  // Reference at ~10m
  double comp_ratio = f.intensity_compensated_mean / comp_threshold;
  double comp_score = std::min(1.0, comp_ratio);
  if (comp_ratio < 1.0) {
    comp_score *= 0.85;
  }

  // Blend: raw intensity is primary, compensated provides distance-invariant consistency
  // Weight toward compensated at longer distances where raw intensity attenuates
  double comp_weight = 0.3;
  if (f.distance_to_sensor > 15.0) {
    comp_weight = 0.5;  // At far range, compensated is more reliable
  }
  double score = (1.0 - comp_weight) * raw_score + comp_weight * comp_score;

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
    const pcl::PointXYZ& centroid, const std::vector<pcl::PointXYZ>& all_centroids,
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
    if (static_cast<int>(i) == self_index)
      continue;

    double dx = all_centroids[i].x - cx;
    double dy = all_centroids[i].y - cy;
    double dist_sq = dx * dx + dy * dy;
    double iso_radius_sq = ts.isolation_radius * ts.isolation_radius;

    if (dist_sq > iso_radius_sq)
      continue;

    double dist = std::sqrt(dist_sq);
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

  // Hard rejection: completely isolated detections get zero confidence
  // This is checked via the return value of -1.0 (sentinel)
  if (neighbor_count <= ts.min_neighbors_hard) {
    return -1.0;  // sentinel: caller should set confidence=0
  }
  if (min_dist > ts.max_isolation_distance) {
    return -1.0;  // sentinel: extremely isolated, hard reject
  }

  return (spacing_score + width_score + isolation_score) / 3.0;
}

double ConfidenceScorer::computeConfidenceWithContext(
    const ClusterFeatures& features, const pcl::PointCloud<PointType>::Ptr& cluster,
    const std::vector<pcl::PointXYZ>& all_centroids, int self_index) {
  // Get base confidence with model fitting
  double confidence = computeConfidenceWithFitting(features, cluster);

  // Add track semantic dimension if enabled
  if (config_.track_semantic.enable && config_.track_semantic.weight > 0.0 && self_index >= 0 &&
      self_index < static_cast<int>(all_centroids.size())) {
    double semantic_score =
        scoreTrackSemanticConstraints(all_centroids[self_index], all_centroids, self_index);

    // Hard rejection sentinel: semantic scoring determined this is isolated noise
    if (semantic_score < 0.0) {
      return 0.0;
    }

    // Blend: reduce other weights proportionally to make room for semantic weight
    double other_weight = 1.0 - config_.track_semantic.weight;
    confidence = other_weight * confidence + config_.track_semantic.weight * semantic_score;
  }

  return std::max(0.0, std::min(1.0, confidence));
}

ConfidenceScorer::ComponentScores ConfidenceScorer::computeComponentScores(
    const ClusterFeatures& features, const pcl::PointCloud<PointType>::Ptr& cluster,
    const std::vector<pcl::PointXYZ>& all_centroids, int self_index) {
  ComponentScores scores;
  if (features.point_count < 1) {
    return scores;
  }

  scores.size_score = scoreSizeConstraints(features);
  scores.shape_score = scoreShapeConstraints(features);
  scores.density_score = scoreDensityConstraints(features);
  scores.intensity_score = scoreIntensityConstraints(features);
  scores.position_score = scorePositionConstraints(features);

  if (config_.track_semantic.enable && self_index >= 0 &&
      self_index < static_cast<int>(all_centroids.size())) {
    double s = scoreTrackSemanticConstraints(all_centroids[self_index], all_centroids, self_index);
    if (s < 0.0) {
      scores.semantic_hard_rejected = true;
      scores.semantic_score = 0.0;
    } else {
      scores.semantic_score = s;
    }
  }

  return scores;
}

}  // namespace perception
