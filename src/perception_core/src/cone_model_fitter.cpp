#include "perception_core/cone_model_fitter.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>

namespace perception {

ConeModelFitter::FitResult ConeModelFitter::fitConeModel(
    const pcl::PointCloud<PointType>::Ptr& cluster) {
  FitResult result;

  if (!cluster || cluster->points.size() < config_.min_points_for_fitting) {
    return result;
  }

  // Get bounding box
  PointType min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);

  double height = std::fabs(max_pt.z - min_pt.z);
  result.height = height;

  if (height < 0.08) {
    return result;  // Too flat to be a cone
  }

  // === PCA Axis Check ===
  // Verify the cluster has a vertical principal axis (standing cone)
  if (config_.use_pca_axis_check && cluster->points.size() >= 3) {
    try {
      pcl::PCA<PointType> pca;
      pca.setInputCloud(cluster);
      Eigen::Vector3f axis = pca.getEigenVectors().col(0);  // Principal axis
      Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
      double verticality = std::fabs(static_cast<double>(axis.dot(z_axis)));

      if (verticality < config_.min_verticality) {
        return result;  // Axis too tilted, not a standing cone
      }
    } catch (...) {
      // PCA may fail for degenerate point distributions
      // Continue without axis check
    }
  }

  // Project points to ground plane (XY)
  std::vector<Eigen::Vector2f> points_2d;
  points_2d.reserve(cluster->points.size());

  for (const auto& pt : cluster->points) {
    points_2d.emplace_back(pt.x, pt.y);
  }

  // === Fit circle using RANSAC + optional LS refinement ===
  Eigen::Vector2f center;
  double radius, error;
  std::vector<bool> inlier_mask;

  if (!fitCircleRANSAC(points_2d, center, radius, inlier_mask)) {
    return result;
  }

  // Refine with least squares on inliers
  if (config_.use_least_squares_refinement) {
    if (!refineCircleLeastSquares(points_2d, inlier_mask, center, radius, error)) {
      // Fall back to RANSAC result if LS fails
      double sum_error = 0.0;
      int inlier_count = 0;
      for (size_t i = 0; i < points_2d.size(); ++i) {
        if (inlier_mask[i]) {
          double dist = std::fabs((points_2d[i] - center).norm() - radius);
          sum_error += dist;
          ++inlier_count;
        }
      }
      error = (inlier_count > 0) ? (sum_error / inlier_count) / radius : 1.0;
    }
  } else {
    double sum_error = 0.0;
    int inlier_count = 0;
    for (size_t i = 0; i < points_2d.size(); ++i) {
      if (inlier_mask[i]) {
        double dist = std::fabs((points_2d[i] - center).norm() - radius);
        sum_error += dist;
        ++inlier_count;
      }
    }
    error = (inlier_count > 0) ? (sum_error / inlier_count) / radius : 1.0;
  }

  // Check if radius is within expected range
  if (radius < config_.expected_base_radius_min || radius > config_.expected_base_radius_max) {
    return result;
  }

  // === Cone Angle Validation ===
  // For a right circular cone: tan(angle) = radius / height
  double cone_angle = std::atan(radius / height);
  result.cone_angle = cone_angle;

  double angle_diff = std::fabs(cone_angle - config_.expected_cone_half_angle);
  if (angle_diff > config_.cone_angle_tolerance) {
    return result;  // Cone angle doesn't match expected geometry
  }

  // === 3D Geometric Error (point-to-cone-surface distance) ===
  double cone_error = computeConeGeometricError(cluster, center, radius, min_pt.z, max_pt.z);

  // === Height Convergence Check ===
  if (!checkHeightConvergence(cluster, min_pt.z, max_pt.z)) {
    return result;
  }

  result.is_valid = true;
  result.fit_error = cone_error;
  result.base_radius = radius;
  result.base_center = center;

  return result;
}

bool ConeModelFitter::fitCircleRANSAC(const std::vector<Eigen::Vector2f>& points_2d,
                                      Eigen::Vector2f& center, double& radius,
                                      std::vector<bool>& inlier_mask) {
  if (points_2d.size() < 3) {
    return false;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, static_cast<int>(points_2d.size()) - 1);

  int best_inliers = 0;
  Eigen::Vector2f best_center = Eigen::Vector2f::Zero();
  double best_radius = 0.0;
  std::vector<bool> best_inlier_mask;

  for (int iter = 0; iter < config_.ransac_iterations; ++iter) {
    // Sample 3 random points
    int idx1 = dis(gen);
    int idx2 = dis(gen);
    int idx3 = dis(gen);

    if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) {
      continue;
    }

    const Eigen::Vector2f& p1 = points_2d[idx1];
    const Eigen::Vector2f& p2 = points_2d[idx2];
    const Eigen::Vector2f& p3 = points_2d[idx3];

    // Compute circle from 3 points using perpendicular bisector intersection
    double ax = p1.x() - p2.x();
    double ay = p1.y() - p2.y();

    double d = 2.0 * (ax * (p2.y() - p3.y()) - ay * (p2.x() - p3.x()));

    if (std::fabs(d) < 1e-6) {
      continue;  // Points are collinear
    }

    double a_sq = p1.x() * p1.x() + p1.y() * p1.y();
    double b_sq = p2.x() * p2.x() + p2.y() * p2.y();
    double c_sq = p3.x() * p3.x() + p3.y() * p3.y();

    double cx = ((a_sq - b_sq) * (p1.y() - p3.y()) - (a_sq - c_sq) * (p1.y() - p2.y())) / d;
    double cy = ((a_sq - b_sq) * (p3.x() - p1.x()) - (a_sq - c_sq) * (p2.x() - p1.x())) / d;

    Eigen::Vector2f c(cx, cy);
    double r = (p1 - c).norm();

    if (r < config_.expected_base_radius_min || r > config_.expected_base_radius_max) {
      continue;  // Skip implausible radii early
    }

    // Count inliers
    int inliers = 0;
    std::vector<bool> mask(points_2d.size(), false);
    for (size_t i = 0; i < points_2d.size(); ++i) {
      double dist = std::fabs((points_2d[i] - c).norm() - r);
      if (dist < config_.ransac_threshold) {
        ++inliers;
        mask[i] = true;
      }
    }

    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_center = c;
      best_radius = r;
      best_inlier_mask = std::move(mask);
    }
  }

  if (best_inliers < 3) {
    return false;
  }

  center = best_center;
  radius = best_radius;
  inlier_mask = std::move(best_inlier_mask);

  return true;
}

bool ConeModelFitter::refineCircleLeastSquares(const std::vector<Eigen::Vector2f>& points_2d,
                                               const std::vector<bool>& inlier_mask,
                                               Eigen::Vector2f& center, double& radius,
                                               double& error) {
  // Collect inlier points
  std::vector<Eigen::Vector2f> inliers;
  inliers.reserve(points_2d.size());
  for (size_t i = 0; i < points_2d.size(); ++i) {
    if (inlier_mask[i]) {
      inliers.push_back(points_2d[i]);
    }
  }

  if (inliers.size() < 3) {
    return false;
  }

  // Algebraic least squares circle fit (Kasa method)
  // Circle equation: (x - a)^2 + (y - b)^2 = r^2
  // Linearized: 2a*x + 2b*y + (r^2 - a^2 - b^2) = x^2 + y^2
  // Solve: [x_i, y_i, 1] * [2a, 2b, d]^T = x_i^2 + y_i^2  where d = r^2 - a^2 - b^2

  double sum_x = 0.0, sum_y = 0.0;
  double sum_x2 = 0.0, sum_y2 = 0.0;
  double sum_xy = 0.0;
  double sum_x3 = 0.0, sum_y3 = 0.0;
  double sum_x2y = 0.0, sum_xy2 = 0.0;

  for (const auto& pt : inliers) {
    double x = pt.x();
    double y = pt.y();
    double x2 = x * x;
    double y2 = y * y;

    sum_x += x;
    sum_y += y;
    sum_x2 += x2;
    sum_y2 += y2;
    sum_xy += x * y;
    sum_x3 += x2 * x;
    sum_y3 += y2 * y;
    sum_x2y += x2 * y;
    sum_xy2 += x * y2;
  }

  size_t n = inliers.size();

  // Build normal equations matrix
  double Cxx = sum_x2 - sum_x * sum_x / n;
  double Cxy = sum_xy - sum_x * sum_y / n;
  double Cyy = sum_y2 - sum_y * sum_y / n;
  double Cxzz = sum_x3 + sum_xy2 - (sum_x2 + sum_y2) * sum_x / n;
  double Cyzz = sum_x2y + sum_y3 - (sum_x2 + sum_y2) * sum_y / n;

  double denom = 2.0 * (Cxx * Cyy - Cxy * Cxy);
  if (std::fabs(denom) < 1e-12) {
    return false;  // Singular system
  }

  double a = (Cxzz * Cyy - Cyzz * Cxy) / denom;
  double b = (Cxx * Cyzz - Cxy * Cxzz) / denom;

  center = Eigen::Vector2f(a, b);

  // Compute radius from inliers
  double sum_r = 0.0;
  for (const auto& pt : inliers) {
    sum_r += (pt - center).norm();
  }
  radius = sum_r / inliers.size();

  if (radius < 1e-6) {
    return false;
  }

  // Compute normalized geometric error
  double sum_error = 0.0;
  for (const auto& pt : inliers) {
    double dist = std::fabs((pt - center).norm() - radius);
    sum_error += dist;
  }
  error = (sum_error / inliers.size()) / radius;

  return true;
}

double ConeModelFitter::computeConeGeometricError(const pcl::PointCloud<PointType>::Ptr& cluster,
                                                  const Eigen::Vector2f& center, double radius,
                                                  double min_z, double max_z) {
  if (!cluster || cluster->points.empty() || radius < 1e-6) {
    return 1.0;
  }

  double height = max_z - min_z;
  if (height < 1e-6) {
    return 1.0;
  }

  double sum_error = 0.0;
  int valid_points = 0;

  for (const auto& pt : cluster->points) {
    // Horizontal distance from cone axis
    double dx = pt.x - center.x();
    double dy = pt.y - center.y();
    double d_horiz = std::sqrt(dx * dx + dy * dy);

    // Normalized height within cone [0, 1]
    double z_norm = (pt.z - min_z) / height;

    // Clamp to valid cone range
    if (z_norm < -0.1 || z_norm > 1.1) {
      continue;  // Skip points far outside expected cone height
    }
    z_norm = std::max(0.0, std::min(1.0, z_norm));

    // Expected radius at this height for a right circular cone
    // r(z) = R * (1 - z/H)  where z is measured from base
    double r_expected = radius * (1.0 - z_norm);

    // Absolute error
    double err = std::fabs(d_horiz - r_expected);
    sum_error += err;
    ++valid_points;
  }

  if (valid_points == 0) {
    return 1.0;
  }

  // Normalized mean error (relative to radius)
  double mean_error = sum_error / valid_points;
  return mean_error / radius;
}

bool ConeModelFitter::checkHeightConvergence(const pcl::PointCloud<PointType>::Ptr& cluster,
                                             double min_z, double max_z) {
  double height = max_z - min_z;
  if (height < 0.1) {
    return false;
  }

  double threshold_z = min_z + height * config_.height_convergence_ratio;

  int bottom_count = 0;
  int top_count = 0;

  for (const auto& pt : cluster->points) {
    if (pt.z < threshold_z) {
      ++bottom_count;
    } else {
      ++top_count;
    }
  }

  // Top should have fewer points than bottom (cone converges)
  return top_count < bottom_count;
}

}  // namespace perception
