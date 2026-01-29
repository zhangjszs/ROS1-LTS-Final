#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include "perception_core/cluster_features.hpp"

namespace perception {

using PointType = pcl::PointXYZI;

class ClusterFeatureExtractor {
public:
    ClusterFeatureExtractor() = default;

    ClusterFeatures extract(const pcl::PointCloud<PointType>::Ptr& cluster);

private:
    void computeGeometricFeatures(const pcl::PointCloud<PointType>::Ptr& cluster,
                                   ClusterFeatures& features);
    void computeIntensityFeatures(const pcl::PointCloud<PointType>::Ptr& cluster,
                                   ClusterFeatures& features);
    void computeShapeFeatures(const pcl::PointCloud<PointType>::Ptr& cluster,
                               ClusterFeatures& features);
};

}  // namespace perception
