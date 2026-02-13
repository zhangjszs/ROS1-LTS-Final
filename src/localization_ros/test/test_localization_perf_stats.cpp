#include <gtest/gtest.h>

#include "localization_ros/localization_perf_stats.hpp"

TEST(LocalizationPerfStatsTest, SnapshotStatsBasic) {
  localization_ros::LocPerfStats stats;
  stats.Configure("test_localization", true, 3, 1000);

  localization_ros::LocPerfSample s1;
  s1.t_total_ms = 1.0;
  s1.landmark_count = 10.0;
  s1.chi2_normalized = 1.0;
  stats.Add(s1);

  localization_ros::LocPerfSample s2;
  s2.t_total_ms = 2.0;
  s2.landmark_count = 20.0;
  s2.chi2_normalized = 2.0;
  stats.Add(s2);

  localization_ros::LocPerfSample s3;
  s3.t_total_ms = 3.0;
  s3.landmark_count = 30.0;
  s3.chi2_normalized = 3.0;
  stats.Add(s3);

  localization_ros::LocPerfStats::Snapshot snap = stats.SnapshotStats();
  EXPECT_NEAR(snap.t_total_ms.mean, 2.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p50, 2.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p95, 2.9, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p99, 2.98, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.max, 3.0, 1e-6);
  EXPECT_NEAR(snap.landmark_count.mean, 20.0, 1e-6);
  EXPECT_NEAR(snap.chi2_normalized.mean, 2.0, 1e-6);
}

TEST(LocalizationPerfStatsTest, WindowRollsForward) {
  localization_ros::LocPerfStats stats;
  stats.Configure("test_localization", true, 3, 1000);

  for (int i = 1; i <= 4; ++i) {
    localization_ros::LocPerfSample sample;
    sample.t_total_ms = static_cast<double>(i);
    sample.landmark_count = static_cast<double>(i * 10);
    sample.chi2_normalized = static_cast<double>(i);
    stats.Add(sample);
  }

  localization_ros::LocPerfStats::Snapshot snap = stats.SnapshotStats();
  EXPECT_NEAR(snap.t_total_ms.mean, 3.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p50, 3.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.max, 4.0, 1e-6);
  EXPECT_NEAR(snap.landmark_count.max, 40.0, 1e-6);
  EXPECT_NEAR(snap.chi2_normalized.max, 4.0, 1e-6);
}

TEST(LocalizationPerfStatsTest, KeepsLocalizationSpecificMetrics) {
  localization_ros::LocPerfStats stats;
  stats.Configure("test_localization", true, 3, 1000);

  localization_ros::LocPerfSample s1;
  s1.t_opt_ms = 1.0;
  s1.t_preint_ms = 2.0;
  s1.t_assoc_ms = 3.0;
  s1.t_prune_ms = 4.0;
  s1.t_total_ms = 5.0;
  s1.landmark_count = 10.0;
  s1.chi2_normalized = 0.5;
  s1.cone_match_ratio = 0.6;
  s1.gnss_availability = 1.0;
  stats.Add(s1);

  localization_ros::LocPerfSample s2;
  s2.t_opt_ms = 2.0;
  s2.t_preint_ms = 3.0;
  s2.t_assoc_ms = 4.0;
  s2.t_prune_ms = 5.0;
  s2.t_total_ms = 6.0;
  s2.landmark_count = 20.0;
  s2.chi2_normalized = 1.5;
  s2.cone_match_ratio = 0.8;
  s2.gnss_availability = 0.0;
  stats.Add(s2);

  localization_ros::LocPerfSample s3;
  s3.t_opt_ms = 3.0;
  s3.t_preint_ms = 4.0;
  s3.t_assoc_ms = 5.0;
  s3.t_prune_ms = 6.0;
  s3.t_total_ms = 7.0;
  s3.landmark_count = 30.0;
  s3.chi2_normalized = 2.5;
  s3.cone_match_ratio = 1.0;
  s3.gnss_availability = 1.0;
  stats.Add(s3);

  localization_ros::LocPerfStats::Snapshot snap = stats.SnapshotStats();
  EXPECT_NEAR(snap.t_opt_ms.mean, 2.0, 1e-6);
  EXPECT_NEAR(snap.t_preint_ms.mean, 3.0, 1e-6);
  EXPECT_NEAR(snap.t_assoc_ms.mean, 4.0, 1e-6);
  EXPECT_NEAR(snap.t_prune_ms.mean, 5.0, 1e-6);
  EXPECT_NEAR(snap.cone_match_ratio.mean, 0.8, 1e-6);
  EXPECT_NEAR(snap.gnss_availability.mean, 2.0 / 3.0, 1e-6);
  EXPECT_NEAR(snap.t_prune_ms.max, 6.0, 1e-6);
  EXPECT_NEAR(snap.cone_match_ratio.max, 1.0, 1e-6);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
