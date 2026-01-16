#include <gtest/gtest.h>
#include <ros/ros.h>

#include "utils/PerfStats.hpp"

TEST(PerfStatsTest, SnapshotStatsBasic) {
  PerfStats stats;
  stats.Configure("test", true, 3, 1000);

  PerfSample s1;
  s1.t_total_ms = 1.0;
  s1.n_points = 10.0;
  stats.Add(s1);

  PerfSample s2;
  s2.t_total_ms = 2.0;
  s2.n_points = 20.0;
  stats.Add(s2);

  PerfSample s3;
  s3.t_total_ms = 3.0;
  s3.n_points = 30.0;
  stats.Add(s3);

  PerfStats::Snapshot snap = stats.SnapshotStats();
  EXPECT_NEAR(snap.t_total_ms.mean, 2.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p50, 2.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p95, 2.9, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p99, 2.98, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.max, 3.0, 1e-6);
  EXPECT_NEAR(snap.n_points.mean, 20.0, 1e-6);
}

TEST(PerfStatsTest, WindowRollsForward) {
  PerfStats stats;
  stats.Configure("test", true, 3, 1000);

  for (int i = 1; i <= 4; ++i) {
    PerfSample s;
    s.t_total_ms = static_cast<double>(i);
    s.n_points = static_cast<double>(i * 10);
    stats.Add(s);
  }

  PerfStats::Snapshot snap = stats.SnapshotStats();
  EXPECT_NEAR(snap.t_total_ms.mean, 3.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.p50, 3.0, 1e-6);
  EXPECT_NEAR(snap.t_total_ms.max, 4.0, 1e-6);
  EXPECT_NEAR(snap.n_points.max, 40.0, 1e-6);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "high_speed_tracking_perf_stats_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
