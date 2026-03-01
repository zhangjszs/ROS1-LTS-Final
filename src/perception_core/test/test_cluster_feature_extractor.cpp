/**
 * @file test_cluster_feature_extractor.cpp
 * @brief ClusterFeatureExtractor 单元测试
 */

#include <gtest/gtest.h>

#include <perception_core/cluster_feature_extractor.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perception {
namespace {

using PointType = pcl::PointXYZI;

pcl::PointCloud<PointType>::Ptr CreateConeCluster(double x, double y, double z,
                                                   double height, double radius,
                                                   int num_points) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  cloud->reserve(num_points);

  // Create a cone-like cluster (cylinder with decreasing radius)
  for (int i = 0; i < num_points; ++i) {
    double h = static_cast<double>(i) / num_points * height;
    double r = radius * (1.0 - h / height * 0.5);  // Taper slightly
    double angle = static_cast<double>(i) * 2.0 * M_PI * 3.0 / num_points;

    PointType pt;
    pt.x = x + r * std::cos(angle);
    pt.y = y + r * std::sin(angle);
    pt.z = z + h;
    pt.intensity = 100.0 + 50.0 * std::sin(angle);
    cloud->push_back(pt);
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

pcl::PointCloud<PointType>::Ptr CreateWallCluster(double x, double y,
                                                   double width, double height,
                                                   int num_points) {
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  cloud->reserve(num_points);

  // Create a flat wall-like cluster
  for (int i = 0; i < num_points; ++i) {
    double wx = static_cast<double>(i % 10) / 10.0 * width;
    double wy = static_cast<double>(i) / num_points * 0.1;  // Thin in y
    double wz = static_cast<double>(i % 20) / 20.0 * height;

    PointType pt;
    pt.x = x + wx;
    pt.y = y + wy;
    pt.z = wz;
    pt.intensity = 80.0;
    cloud->push_back(pt);
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace

TEST(ClusterFeatureExtractorTest, EmptyCloudReturnsZeroFeatures) {
  ClusterFeatureExtractor extractor;
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();

  auto features = extractor.extract(cloud);

  EXPECT_EQ(features.point_count, 0);
  EXPECT_DOUBLE_EQ(features.height, 0.0);
  EXPECT_DOUBLE_EQ(features.area, 0.0);
  EXPECT_DOUBLE_EQ(features.volume, 0.0);
}

TEST(ClusterFeatureExtractorTest, SinglePointHasZeroDimensions) {
  ClusterFeatureExtractor extractor;
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();

  PointType pt;
  pt.x = 1.0;
  pt.y = 2.0;
  pt.z = 0.5;
  pt.intensity = 100.0;
  cloud->push_back(pt);

  auto features = extractor.extract(cloud);

  EXPECT_EQ(features.point_count, 1);
  EXPECT_DOUBLE_EQ(features.centroid.x(), 1.0);
  EXPECT_DOUBLE_EQ(features.centroid.y(), 2.0);
  EXPECT_DOUBLE_EQ(features.centroid.z(), 0.5);
}

TEST(ClusterFeatureExtractorTest, ConeClusterHasReasonableFeatures) {
  ClusterFeatureExtractor extractor;
  auto cloud = CreateConeCluster(5.0, 1.0, 0.0, 0.3, 0.15, 50);

  auto features = extractor.extract(cloud);

  EXPECT_EQ(features.point_count, 50);
  EXPECT_GT(features.height, 0.0);
  EXPECT_GT(features.width, 0.0);
  EXPECT_GT(features.length, 0.0);
  EXPECT_GT(features.area, 0.0);
  EXPECT_GT(features.volume, 0.0);
  EXPECT_GT(features.point_density, 0.0);

  // Distance to sensor should be approximately sqrt(5^2 + 1^2) = ~5.1m
  double expected_dist = std::sqrt(5.0 * 5.0 + 1.0 * 1.0);
  EXPECT_NEAR(features.distance_to_sensor, expected_dist, 0.5);
}

TEST(ClusterFeatureExtractorTest, WallClusterHasHighLinearity) {
  ClusterFeatureExtractor extractor;
  auto cloud = CreateWallCluster(3.0, 0.0, 2.0, 1.0, 100);

  auto features = extractor.extract(cloud);

  EXPECT_EQ(features.point_count, 100);
  EXPECT_GT(features.linearity, 0.5);  // Wall should have high linearity
}

TEST(ClusterFeatureExtractorTest, CentroidIsCorrectlyComputed) {
  ClusterFeatureExtractor extractor;
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();

  // Create symmetric cluster around (2, 3, 0.5)
  for (int i = 0; i < 8; ++i) {
    PointType pt;
    pt.x = 2.0 + (i & 1 ? 0.1 : -0.1);
    pt.y = 3.0 + (i & 2 ? 0.1 : -0.1);
    pt.z = 0.5 + (i & 4 ? 0.1 : -0.1);
    pt.intensity = 100.0;
    cloud->push_back(pt);
  }

  auto features = extractor.extract(cloud);

  EXPECT_NEAR(features.centroid.x(), 2.0, 0.01);
  EXPECT_NEAR(features.centroid.y(), 3.0, 0.01);
  EXPECT_NEAR(features.centroid.z(), 0.5, 0.01);
}

TEST(ClusterFeatureExtractorTest, IntensityFeaturesAreComputed) {
  ClusterFeatureExtractor extractor;
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();

  // Create points with varying intensity
  for (int i = 0; i < 10; ++i) {
    PointType pt;
    pt.x = i * 0.1;
    pt.y = 0.0;
    pt.z = 0.0;
    pt.intensity = static_cast<float>(50 + i * 10);  // 50, 60, 70, ..., 140
    cloud->push_back(pt);
  }

  auto features = extractor.extract(cloud);

  EXPECT_GT(features.intensity_mean, 0.0);
  EXPECT_GT(features.intensity_max, 0.0);
  EXPECT_GE(features.intensity_std, 0.0);

  // Mean should be around 95 (average of 50..140)
  EXPECT_NEAR(features.intensity_mean, 95.0, 5.0);
  // Max should be 140
  EXPECT_NEAR(features.intensity_max, 140.0, 1.0);
}

TEST(ClusterFeatureExtractorTest, GroundHeightIsMinZ) {
  ClusterFeatureExtractor extractor;
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();

  for (int i = 0; i < 20; ++i) {
    PointType pt;
    pt.x = i * 0.1;
    pt.y = 0.0;
    pt.z = 0.1 + i * 0.01;  // z ranges from 0.1 to 0.29
    pt.intensity = 100.0;
    cloud->push_back(pt);
  }

  auto features = extractor.extract(cloud);

  EXPECT_NEAR(features.ground_height, 0.1, 0.01);
  EXPECT_NEAR(features.min_point.z, 0.1, 0.01);
}

TEST(ClusterFeatureExtractorTest, BoundingBoxIsCorrect) {
  ClusterFeatureExtractor extractor;
  auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();

  // Create a box-shaped cluster
  PointType pt;
  pt.intensity = 100.0;

  pt.x = 1.0; pt.y = 2.0; pt.z = 0.0; cloud->push_back(pt);
  pt.x = 2.0; pt.y = 2.0; pt.z = 0.0; cloud->push_back(pt);
  pt.x = 1.0; pt.y = 3.0; pt.z = 0.0; cloud->push_back(pt);
  pt.x = 2.0; pt.y = 3.0; pt.z = 0.0; cloud->push_back(pt);
  pt.x = 1.0; pt.y = 2.0; pt.z = 0.5; cloud->push_back(pt);
  pt.x = 2.0; pt.y = 3.0; pt.z = 0.5; cloud->push_back(pt);

  auto features = extractor.extract(cloud);

  EXPECT_NEAR(features.min_point.x, 1.0, 0.01);
  EXPECT_NEAR(features.max_point.x, 2.0, 0.01);
  EXPECT_NEAR(features.min_point.y, 2.0, 0.01);
  EXPECT_NEAR(features.max_point.y, 3.0, 0.01);
  EXPECT_NEAR(features.min_point.z, 0.0, 0.01);
  EXPECT_NEAR(features.max_point.z, 0.5, 0.01);

  EXPECT_NEAR(features.length, 1.0, 0.01);  // x extent
  EXPECT_NEAR(features.width, 1.0, 0.01);   // y extent
  EXPECT_NEAR(features.height, 0.5, 0.01);  // z extent
}

TEST(ClusterFeatureExtractorTest, AspectRatioIsReasonable) {
  ClusterFeatureExtractor extractor;

  // Tall thin cone-like cluster (has extent in all dimensions)
  auto tall_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
  for (int i = 0; i < 20; ++i) {
    PointType pt;
    // Small x,y extent but non-zero
    double angle = static_cast<double>(i) * 2.0 * M_PI / 20.0;
    pt.x = 0.05 * std::cos(angle);  // Small diameter ~0.1m
    pt.y = 0.05 * std::sin(angle);
    pt.z = i * 0.02;  // Height ~0.4m
    pt.intensity = 100.0;
    tall_cloud->push_back(pt);
  }

  auto tall_features = extractor.extract(tall_cloud);

  // For a cone: height ~0.4m, length+width ~0.2m, aspect_ratio ~2.0
  EXPECT_GT(tall_features.aspect_ratio, 0.5);  // Tall cluster has high aspect ratio
  EXPECT_GT(tall_features.height, tall_features.length);
  EXPECT_GT(tall_features.height, tall_features.width);
}

}  // namespace perception

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
