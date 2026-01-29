#include "perception_core/cone_model_fitter.hpp"
#include <pcl/common/common.h>
#include <cmath>
#include <algorithm>
#include <random>

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

    // Project points to ground plane (XY)
    std::vector<Eigen::Vector2f> points_2d;
    points_2d.reserve(cluster->points.size());

    for (const auto& pt : cluster->points) {
        points_2d.emplace_back(pt.x, pt.y);
    }

    // Fit circle using RANSAC
    Eigen::Vector2f center;
    double radius, error;

    if (!fitCircleRANSAC(points_2d, center, radius, error)) {
        return result;
    }

    // Check if radius is within expected range
    if (radius < config_.expected_base_radius_min ||
        radius > config_.expected_base_radius_max) {
        return result;
    }

    // Check height convergence (cone shape)
    if (!checkHeightConvergence(cluster, min_pt.z, max_pt.z)) {
        return result;
    }

    result.is_valid = true;
    result.fit_error = error;
    result.base_radius = radius;
    result.base_center = center;

    return result;
}

bool ConeModelFitter::fitCircleRANSAC(
    const std::vector<Eigen::Vector2f>& points_2d,
    Eigen::Vector2f& center, double& radius, double& error) {

    if (points_2d.size() < 3) {
        return false;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points_2d.size() - 1);

    int best_inliers = 0;
    Eigen::Vector2f best_center;
    double best_radius = 0.0;

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

        // Compute circle from 3 points
        double ax = p1.x() - p2.x();
        double ay = p1.y() - p2.y();
        double bx = p1.x() - p3.x();
        double by = p1.y() - p3.y();

        double d = 2.0 * (ax * (p2.y() - p3.y()) - ay * (p2.x() - p3.x()));

        if (std::fabs(d) < 1e-6) {
            continue;  // Points are collinear
        }

        double a_sq = p1.x() * p1.x() + p1.y() * p1.y();
        double b_sq = p2.x() * p2.x() + p2.y() * p2.y();
        double c_sq = p3.x() * p3.x() + p3.y() * p3.y();

        double cx = ((a_sq - b_sq) * (p1.y() - p3.y()) -
                     (a_sq - c_sq) * (p1.y() - p2.y())) / d;
        double cy = ((a_sq - b_sq) * (p3.x() - p1.x()) -
                     (a_sq - c_sq) * (p2.x() - p1.x())) / d;

        Eigen::Vector2f c(cx, cy);
        double r = (p1 - c).norm();

        // Count inliers
        int inliers = 0;
        for (const auto& pt : points_2d) {
            double dist = std::fabs((pt - c).norm() - r);
            if (dist < config_.ransac_threshold) {
                ++inliers;
            }
        }

        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_center = c;
            best_radius = r;
        }
    }

    if (best_inliers < 3) {
        return false;
    }

    // Compute fit error
    double sum_error = 0.0;
    for (const auto& pt : points_2d) {
        double dist = std::fabs((pt - best_center).norm() - best_radius);
        sum_error += dist;
    }
    error = sum_error / points_2d.size() / best_radius;  // Normalized error

    center = best_center;
    radius = best_radius;

    return true;
}

bool ConeModelFitter::checkHeightConvergence(
    const pcl::PointCloud<PointType>::Ptr& cluster,
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
