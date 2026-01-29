#pragma once

#include <Eigen/Dense>
#include <pcl/point_types.h>

namespace perception {

struct ClusterFeatures {
    // Geometric features
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
    double area = 0.0;
    double volume = 0.0;
    double aspect_ratio = 0.0;  // height/(length+width)

    // Point cloud features
    int point_count = 0;
    double point_density = 0.0;  // points/volume
    double distance_to_sensor = 0.0;

    // Intensity features
    double intensity_mean = 0.0;
    double intensity_std = 0.0;
    double intensity_max = 0.0;

    // Shape features (PCA-based)
    double shape_elongation = 0.0;  // ratio of eigenvalues
    double shape_planarity = 0.0;
    double verticality_score = 0.0;

    // Position features
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    double ground_height = 0.0;  // min.z

    // Bounding box
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
};

}  // namespace perception
