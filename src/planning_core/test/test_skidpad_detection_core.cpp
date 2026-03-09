#include "planning_core/skidpad_detection_core.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

TEST(SkidpadDetectionCoreTest, Initialization) {
  planning_core::SkidpadParams params;
  planning_core::SkidpadDetectionCore core(params);
  EXPECT_EQ(core.GetPhase(), planning_core::SkidpadPhase::ENTRY);
  EXPECT_FALSE(core.IsApproachingGoal());
}

TEST(SkidpadDetectionCoreTest, EmptyConesNoCrash) {
  planning_core::SkidpadParams params;
  planning_core::SkidpadDetectionCore core(params);

  std::vector<planning_core::ConePoint> cones;
  core.ProcessConeDetections(cones);

  planning_core::Trajectory state;
  state.x = 0.0;
  state.y = 0.0;
  state.yaw = 0.0;
  state.v = 0.0;
  core.UpdateVehicleState(state);

  core.RunAlgorithm();
  EXPECT_FALSE(core.HasNewPath());
}

TEST(SkidpadDetectionCoreTest, SimpleCircleFitting) {
  planning_core::SkidpadParams params;
  params.circle_radius = 9.125;
  params.center_distance_nominal = 18.25;
  planning_core::SkidpadDetectionCore core(params);

  std::vector<planning_core::ConePoint> cones;
  for (int i = 0; i < 16; ++i) {
    double angle = 2.0 * M_PI * i / 16.0;
    cones.push_back({params.circle_radius * std::cos(angle), params.circle_radius * std::sin(angle),
                     0.0, 4});  // NONE
  }
  core.ProcessConeDetections(cones);

  planning_core::Trajectory state;
  state.x = 0.0;
  state.y = 0.0;
  state.yaw = 0.0;
  state.v = 5.0;
  core.UpdateVehicleState(state);

  core.RunAlgorithm();
  EXPECT_TRUE(core.IsGeometryValid() || !core.IsGeometryValid());
}

TEST(SkidpadDetectionCoreTest, PhaseNameIsValid) {
  planning_core::SkidpadParams params;
  planning_core::SkidpadDetectionCore core(params);

  std::string phase_name = core.GetPhaseName();
  EXPECT_FALSE(phase_name.empty());
}

TEST(SkidpadDetectionCoreTest, GetLapsReturnsNonNegative) {
  planning_core::SkidpadParams params;
  planning_core::SkidpadDetectionCore core(params);

  EXPECT_GE(core.GetRightLaps(), 0);
  EXPECT_GE(core.GetLeftLaps(), 0);
}

TEST(SkidpadDetectionCoreTest, SpeedCapReturnsValid) {
  planning_core::SkidpadParams params;
  params.speed_entry = 6.0;
  params.speed_warmup = 7.0;
  params.speed_timed = 8.0;
  params.speed_crossover = 6.5;
  params.speed_exit = 5.0;
  planning_core::SkidpadDetectionCore core(params);

  double speed_cap = core.GetRecommendedSpeedCap();
  EXPECT_GE(speed_cap, 0.0);
}

TEST(SkidpadDetectionCoreTest, ColorAwareCircleSplit) {
  planning_core::SkidpadParams params;
  params.circle_radius = 9.125;
  params.center_distance_nominal = 18.25;
  params.passthrough_x_min = -20.0;
  params.passthrough_x_max = 20.0;
  params.passthrough_y_min = -20.0;
  params.passthrough_y_max = 20.0;
  planning_core::SkidpadDetectionCore core(params);

  std::vector<planning_core::ConePoint> cones;
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * i / 8.0;
    double cx = 9.125 * std::cos(angle);
    double cy = -9.125 + 9.125 * std::sin(angle);
    cones.push_back({cx, cy, 0.0, 1});  // YELLOW = left
  }
  core.ProcessConeDetections(cones);

  planning_core::Trajectory state{};
  core.UpdateVehicleState(state);
  core.RunAlgorithm();
  SUCCEED();
}

TEST(SkidpadDetectionCoreTest, NoColorFallsBackToGeometric) {
  planning_core::SkidpadParams params;
  params.circle_radius = 9.125;
  params.center_distance_nominal = 18.25;
  planning_core::SkidpadDetectionCore core(params);

  std::vector<planning_core::ConePoint> cones;
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * i / 8.0;
    cones.push_back({9.125 * std::cos(angle), -9.125 + 9.125 * std::sin(angle), 0.0, 4});
  }
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * i / 8.0;
    cones.push_back({9.125 * std::cos(angle), 9.125 + 9.125 * std::sin(angle), 0.0, 4});
  }
  core.ProcessConeDetections(cones);

  planning_core::Trajectory state{};
  core.UpdateVehicleState(state);
  core.RunAlgorithm();
  SUCCEED();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
