#include "perception_core/cluster_feature_extractor.hpp"
#include <cmath>
#include <algorithm>

namespace perception {

ClusterFeatures ClusterFeatureExtractor::extract(const pcl::PointCloud<PointType>::Ptr& cluster) {
    ClusterFeatures features;

    if (!cluster || cluster->points.empty()) {
        return features;
    }

    computeGeometricFeatures(cluster, features);
    computeIntensityFeatures(cluster, features);
    computeShapeFeatures(cluster, features);

    return features;
}

void ClusterFeatureExtractor::computeGeometricFeatures(
    const pcl::PointCloud<PointType>::Ptr& cluster,
    ClusterFeatures& features) {

    PointType min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    features.min_point = pcl::PointXYZ(min_pt.x, min_pt.y, min_pt.z);
    features.max_point = pcl::PointXYZ(max_pt.x, max_pt.y, max_pt.z);

    features.length = std::fabs(max_pt.x - min_pt.x);
    features.width = std::fabs(max_pt.y - min_pt.y);
    features.height = std::fabs(max_pt.z - min_pt.z);
    features.area = features.length * features.width;
    features.volume = features.length * features.width * features.height;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    features.centroid = Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
    features.distance_to_sensor = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);

    if (features.volume > 1e-6) {
        features.point_density = static_cast<double>(cluster->points.size()) / features.volume;
        // 稀疏远处聚类密度封顶：<=3点在极小体积中会产生虚高密度
        if (cluster->points.size() <= 3 && features.distance_to_sensor > 20.0) {
            features.point_density = std::min(features.point_density, 200.0);
        }
    }

    double sum_lw = features.length + features.width;
    if (sum_lw > 1e-6) {
        features.aspect_ratio = features.height / sum_lw;
    }

    features.ground_height = min_pt.z;
    features.point_count = cluster->points.size();
}

void ClusterFeatureExtractor::computeIntensityFeatures(
    const pcl::PointCloud<PointType>::Ptr& cluster,
    ClusterFeatures& features) {

    if (cluster->points.empty()) return;

    double sum = 0.0;
    double max_intensity = 0.0;

    for (const auto& pt : cluster->points) {
        sum += pt.intensity;
        max_intensity = std::max(max_intensity, static_cast<double>(pt.intensity));
    }

    features.intensity_mean = sum / cluster->points.size();
    features.intensity_max = max_intensity;

    double variance = 0.0;
    for (const auto& pt : cluster->points) {
        double diff = pt.intensity - features.intensity_mean;
        variance += diff * diff;
    }
    features.intensity_std = std::sqrt(variance / cluster->points.size());
}

void ClusterFeatureExtractor::computeShapeFeatures(
    const pcl::PointCloud<PointType>::Ptr& cluster,
    ClusterFeatures& features) {

    if (cluster->points.size() < 3) return;

    pcl::PCA<PointType> pca;
    pca.setInputCloud(cluster);

    Eigen::Vector3f eigenvalues = pca.getEigenValues();

    if (eigenvalues[0] > 1e-6) {
        features.shape_elongation = eigenvalues[2] / eigenvalues[0];
        features.shape_planarity = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0];
        features.linearity = (eigenvalues[0] - eigenvalues[1]) / eigenvalues[0];
    }

    Eigen::Vector3f eigenvector = pca.getEigenVectors().col(0);
    Eigen::Vector3f z_axis(0, 0, 1);
    features.verticality_score = std::fabs(eigenvector.dot(z_axis));
}

}  // namespace perception
