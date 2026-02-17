#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "planning_core/speed_profile.hpp"

TEST(SpeedProfileTest, EmptyPath) {
  std::vector<planning_core::Point2D> path;
  std::vector<double> curvatures;
  std::vector<double> speeds;

  planning_core::SpeedProfileParams params;
  planning_core::ComputeSpeedProfile(path, curvatures, params, speeds);

  EXPECT_TRUE(speeds.empty());
}

TEST(SpeedProfileTest, SinglePoint) {
  std::vector<planning_core::Point2D> path = {{0.0, 0.0}};
  std::vector<double> curvatures = {0.0};
  std::vector<double> speeds;

  planning_core::SpeedProfileParams params;
  params.speed_cap = 10.0;
  planning_core::ComputeSpeedProfile(path, curvatures, params, speeds);

  EXPECT_EQ(speeds.size(), 1u);
  EXPECT_GE(speeds[0], 0.0);
}

TEST(SpeedProfileTest, StraightLineNoCurvature) {
  std::vector<planning_core::Point2D> path = {
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}
  };
  std::vector<double> curvatures(path.size(), 0.0);
  std::vector<double> speeds;

  planning_core::SpeedProfileParams params;
  params.speed_cap = 10.0;
  params.max_accel = 3.0;
  params.max_brake = 4.0;
  params.current_speed = 5.0;
  planning_core::ComputeSpeedProfile(path, curvatures, params, speeds);

  EXPECT_EQ(speeds.size(), path.size());
  for (double v : speeds) {
    EXPECT_GE(v, 0.0);
    EXPECT_LE(v, params.speed_cap);
  }
  EXPECT_GE(speeds[0], params.current_speed - 0.1);
}

TEST(SpeedProfileTest, CurvatureLimitsSpeed) {
  std::vector<planning_core::Point2D> path = {
      {0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 1.0}, {4.0, 0.0}
  };
  std::vector<double> curvatures;
  planning_core::ComputeCurvatures(path, curvatures);

  std::vector<double> speeds;
  planning_core::SpeedProfileParams params;
  params.speed_cap = 20.0;
  params.max_lateral_acc = 6.5;
  planning_core::ComputeSpeedProfile(path, curvatures, params, speeds);

  EXPECT_EQ(speeds.size(), path.size());
  for (size_t i = 0; i < curvatures.size(); ++i) {
    if (std::abs(curvatures[i]) > 1e-6) {
      double v_lat_max = std::sqrt(params.max_lateral_acc / std::abs(curvatures[i]));
      EXPECT_LE(speeds[i], v_lat_max + 0.1);
    }
  }
}

TEST(SpeedProfileTest, SpeedMonotonicityWithAcceleration) {
  std::vector<planning_core::Point2D> path;
  for (int i = 0; i < 20; ++i) {
    path.push_back({static_cast<double>(i), 0.0});
  }
  std::vector<double> curvatures(path.size(), 0.0);
  std::vector<double> speeds;

  planning_core::SpeedProfileParams params;
  params.speed_cap = 20.0;
  params.max_accel = 3.0;
  params.current_speed = 1.0;
  planning_core::ComputeSpeedProfile(path, curvatures, params, speeds);

  for (size_t i = 1; i < speeds.size(); ++i) {
    double reachable = std::sqrt(std::max(0.0, speeds[i-1] * speeds[i-1] + 2.0 * params.max_accel * 1.0));
    EXPECT_LE(speeds[i], reachable + 0.01);
  }
}

TEST(SpeedProfileTest, SpeedSeedsFromCurrentSpeed) {
  std::vector<planning_core::Point2D> path = {
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}
  };
  std::vector<double> curvatures(path.size(), 0.0);
  std::vector<double> speeds;

  planning_core::SpeedProfileParams params;
  params.speed_cap = 20.0;
  params.current_speed = 8.0;
  planning_core::ComputeSpeedProfile(path, curvatures, params, speeds);

  EXPECT_GE(speeds[0], 7.5);
  EXPECT_LE(speeds[0], 8.5);
}

TEST(SpeedProfileTest, ZeroCurrentSpeedStartsFromZero) {
  std::vector<planning_core::Point2D> path = {
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}
  };
  std::vector<double> curvatures(path.size(), 0.0);
  std::vector<double> speeds;

  planning_core::SpeedProfileParams params;
  params.speed_cap = 20.0;
  params.current_speed = 0.0;
  planning_core::ComputeSpeedProfile(path, curvatures, params, speeds);

  EXPECT_GE(speeds[0], 0.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
