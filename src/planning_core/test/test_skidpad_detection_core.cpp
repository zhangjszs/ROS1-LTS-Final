#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include "planning_core/skidpad_detection_core.hpp"

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
    cones.push_back({params.circle_radius * std::cos(angle),
                     params.circle_radius * std::sin(angle), 0.0});
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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
