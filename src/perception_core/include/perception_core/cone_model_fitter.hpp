#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace perception {

using PointType = pcl::PointXYZI;

class ConeModelFitter {
public:
    struct Config {
        bool enable = true;
        int ransac_iterations = 50;
        double ransac_threshold = 0.03;  // 3cm
        double expected_base_radius_min = 0.05;  // 5cm
        double expected_base_radius_max = 0.15;  // 15cm
        double height_convergence_ratio = 0.6;  // top 40% should have fewer points
        int min_points_for_fitting = 8;
    };

    struct FitResult {
        bool is_valid = false;
        double fit_error = 1.0;
        double base_radius = 0.0;
        double height = 0.0;
        Eigen::Vector2f base_center = Eigen::Vector2f::Zero();
    };

    ConeModelFitter() = default;
    explicit ConeModelFitter(const Config& config) : config_(config) {}

    void setConfig(const Config& config) { config_ = config; }
    FitResult fitConeModel(const pcl::PointCloud<PointType>::Ptr& cluster);

private:
    bool fitCircleRANSAC(const std::vector<Eigen::Vector2f>& points_2d,
                         Eigen::Vector2f& center, double& radius, double& error);

    bool checkHeightConvergence(const pcl::PointCloud<PointType>::Ptr& cluster,
                                 double min_z, double max_z);

    Config config_;
};

}  // namespace perception
