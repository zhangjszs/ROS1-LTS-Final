/**
 * @file test_confidence_scorer.cpp
 * @brief ConfidenceScorer 单元测试
 */

#include <gtest/gtest.h>

#include <perception_core/confidence_scorer.hpp>

namespace {

perception::ClusterFeatures MakeNominalFeatures() {
  perception::ClusterFeatures f;
  f.point_count = 40;
  f.height = 0.32;
  f.area = 0.06;
  f.aspect_ratio = 2.2;
  f.verticality_score = 0.92;
  f.linearity = 0.2;
  f.point_density = 80.0;
  f.distance_to_sensor = 8.0;
  f.intensity_mean = 90.0;
  f.ground_height = 0.02;
  return f;
}

perception::ClusterFeatures MakePoorFeatures() {
  perception::ClusterFeatures f;
  f.point_count = 40;
  f.height = 1.2;
  f.area = 0.5;
  f.aspect_ratio = 0.5;
  f.verticality_score = 0.3;
  f.linearity = 0.98;
  f.point_density = 1.0;
  f.distance_to_sensor = 2.0;
  f.intensity_mean = 3.0;
  f.ground_height = 1.0;
  return f;
}

}  // namespace

TEST(ConfidenceScorerTest, ZeroPointCountReturnsZero) {
  perception::ConfidenceScorer scorer;
  perception::ClusterFeatures f = MakeNominalFeatures();
  f.point_count = 0;

  EXPECT_DOUBLE_EQ(scorer.computeConfidence(f), 0.0);
}

TEST(ConfidenceScorerTest, ConfidenceIsBoundedWithinUnitInterval) {
  perception::ConfidenceScorer scorer;
  const perception::ClusterFeatures good = MakeNominalFeatures();
  const perception::ClusterFeatures bad = MakePoorFeatures();

  const double good_conf = scorer.computeConfidence(good);
  const double bad_conf = scorer.computeConfidence(bad);
  EXPECT_GE(good_conf, 0.0);
  EXPECT_LE(good_conf, 1.0);
  EXPECT_GE(bad_conf, 0.0);
  EXPECT_LE(bad_conf, 1.0);
}

TEST(ConfidenceScorerTest, BetterFeaturesYieldHigherConfidence) {
  perception::ConfidenceScorer scorer;
  const perception::ClusterFeatures good = MakeNominalFeatures();
  const perception::ClusterFeatures bad = MakePoorFeatures();

  EXPECT_GT(scorer.computeConfidence(good), scorer.computeConfidence(bad));
}

TEST(ConfidenceScorerTest, ContextHardRejectsIsolatedDetection) {
  perception::ConfidenceScorer::Config cfg;
  cfg.enable_model_fitting = false;
  cfg.track_semantic.enable = true;
  cfg.track_semantic.weight = 0.2;
  cfg.track_semantic.min_neighbors_hard = 0;
  cfg.track_semantic.isolation_radius = 6.0;
  cfg.track_semantic.max_isolation_distance = 10.0;
  perception::ConfidenceScorer scorer(cfg);

  const perception::ClusterFeatures feature = MakeNominalFeatures();
  std::vector<pcl::PointXYZ> centroids;
  centroids.emplace_back(2.0f, 1.0f, 0.0f);     // self
  centroids.emplace_back(30.0f, 30.0f, 0.0f);   // outside isolation radius

  const double conf = scorer.computeConfidenceWithContext(
      feature, pcl::PointCloud<perception::PointType>::Ptr(), centroids, 0);
  EXPECT_DOUBLE_EQ(conf, 0.0);
}

TEST(ConfidenceScorerTest, ContextAcceptsStructuredNeighborhood) {
  perception::ConfidenceScorer::Config cfg;
  cfg.enable_model_fitting = false;
  cfg.track_semantic.enable = true;
  cfg.track_semantic.weight = 0.3;
  cfg.track_semantic.min_neighbors_hard = 0;
  cfg.track_semantic.isolation_radius = 10.0;
  cfg.track_semantic.max_isolation_distance = 12.0;
  cfg.track_semantic.expected_cone_spacing = 5.0;
  cfg.track_semantic.expected_track_width = 3.0;
  cfg.track_semantic.spacing_tolerance = 2.0;
  cfg.track_semantic.width_tolerance = 1.0;
  perception::ConfidenceScorer scorer(cfg);

  const perception::ClusterFeatures feature = MakeNominalFeatures();
  std::vector<pcl::PointXYZ> centroids;
  centroids.emplace_back(2.0f, 1.0f, 0.0f);    // self
  centroids.emplace_back(7.0f, 1.0f, 0.0f);    // same side, spacing ~5m
  centroids.emplace_back(2.0f, -2.0f, 0.0f);   // opposite side, width ~3m

  const double conf = scorer.computeConfidenceWithContext(
      feature, pcl::PointCloud<perception::PointType>::Ptr(), centroids, 0);
  EXPECT_GT(conf, 0.0);
  EXPECT_LE(conf, 1.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
