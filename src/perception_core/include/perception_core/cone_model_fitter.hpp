#pragma once

#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perception {

using PointType = pcl::PointXYZI;

class ConeModelFitter {
 public:
  struct Config {
    bool enable = true;
    int ransac_iterations = 100;             // Increased for sparse point clouds
    double ransac_threshold = 0.03;          // 3cm
    double expected_base_radius_min = 0.05;  // 5cm
    double expected_base_radius_max = 0.25;  // 25cm
    double height_convergence_ratio = 0.6;   // top 40% should have fewer points
    size_t min_points_for_fitting = 5;       // Lower threshold for distant cones

    // 3D cone geometry constraints (Formula Student standard cone)
    double expected_cone_half_angle = 0.307;  // ~17.6 degrees [rad]
    double cone_angle_tolerance = 0.209;      // ~12 degrees tolerance
    bool use_pca_axis_check = true;
    double min_verticality = 0.3;  // Minimum |dot(axis, z)| for standing cone
    bool use_least_squares_refinement = true;
  };

  struct FitResult {
    bool is_valid = false;
    double fit_error = 1.0;  // Normalized 3D geometric error
    double base_radius = 0.0;
    double height = 0.0;
    double cone_angle = 0.0;  // Apex half-angle [rad]
    Eigen::Vector2f base_center = Eigen::Vector2f::Zero();
  };

  ConeModelFitter() = default;
  explicit ConeModelFitter(const Config& config) : config_(config) {}

  void setConfig(const Config& config) { config_ = config; }
  FitResult fitConeModel(const pcl::PointCloud<PointType>::Ptr& cluster);

 private:
  bool fitCircleRANSAC(const std::vector<Eigen::Vector2f>& points_2d, Eigen::Vector2f& center,
                       double& radius, std::vector<bool>& inlier_mask);

  bool refineCircleLeastSquares(const std::vector<Eigen::Vector2f>& points_2d,
                                const std::vector<bool>& inlier_mask, Eigen::Vector2f& center,
                                double& radius, double& error);

  double computeConeGeometricError(const pcl::PointCloud<PointType>::Ptr& cluster,
                                   const Eigen::Vector2f& center, double radius, double min_z,
                                   double max_z);

  bool checkHeightConvergence(const pcl::PointCloud<PointType>::Ptr& cluster, double min_z,
                              double max_z);

  Config config_;
};

}  // namespace perception
