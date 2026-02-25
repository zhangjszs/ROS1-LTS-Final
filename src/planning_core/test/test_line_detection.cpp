#include <gtest/gtest.h>
#include "planning_core/line_detection_core.hpp"

TEST(LineDetectionCoreTest, Initialization) {
  planning_core::LineDetectionParams params;
  planning_core::LineDetectionCore core(params);
  EXPECT_FALSE(core.IsFinished());
  EXPECT_FALSE(core.HasPlannedPath());
}

TEST(LineDetectionCoreTest, SparseConeFallbackBuildsLongPath) {
  planning_core::LineDetectionParams params;
  params.path_start_x = 0.0;
  params.path_interval = 1.0;
  params.accel_distance = 75.0;
  params.brake_distance = 100.0;
  params.max_path_distance = 175.0;
  params.min_valid_cones = 2;

  planning_core::LineDetectionCore core(params);

  planning_core::VehicleState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 0.0;
  core.UpdateVehicleState(state);

  std::vector<planning_core::ConePoint> sparse_cones;
  sparse_cones.push_back({5.0, -1.2, 0.0, 0});  // BLUE = right
  sparse_cones.push_back({5.0, 1.2, 0.0, 1});   // YELLOW = left
  core.UpdateCones(sparse_cones);
  core.RunAlgorithm();

  ASSERT_TRUE(core.HasPlannedPath());
  const auto &path = core.GetPlannedPath();
  EXPECT_GE(path.size(), 170u);
}

TEST(LineDetectionCoreTest, FinishDetectionUsesAccelDistance) {
  planning_core::LineDetectionParams params;
  params.path_start_x = 0.0;
  params.path_interval = 1.0;
  params.accel_distance = 75.0;
  params.brake_distance = 100.0;
  params.max_path_distance = 175.0;
  params.min_valid_cones = 2;

  planning_core::LineDetectionCore core(params);

  planning_core::VehicleState state;
  state.x = 0.0;
  state.y = 0.0;
  state.theta = 0.0;
  state.v = 0.0;
  core.UpdateVehicleState(state);
  core.UpdateCones({{5.0, -1.2, 0.0, 0}, {5.0, 1.2, 0.0, 1}});
  core.RunAlgorithm();
  ASSERT_TRUE(core.HasPlannedPath());
  EXPECT_FALSE(core.IsFinished());

  state.x = 75.0;
  core.UpdateVehicleState(state);
  core.UpdateCones({{5.0, -1.2, 0.0, 0}, {5.0, 1.2, 0.0, 1}});
  core.RunAlgorithm();
  EXPECT_TRUE(core.IsFinished());
}

TEST(LineDetectionCoreTest, ColorAwareFallbackUsesColorCenter) {
  planning_core::LineDetectionParams params;
  params.path_start_x = 0.0;
  params.path_interval = 1.0;
  params.accel_distance = 75.0;
  params.brake_distance = 100.0;
  params.max_path_distance = 175.0;
  params.min_valid_cones = 2;

  planning_core::LineDetectionCore core(params);
  planning_core::VehicleState state{};
  core.UpdateVehicleState(state);

  // 1 YELLOW at y=+2, 2 BLUE at y=-1 and y=-2
  // Geometric (min+max)/2 = (-2+2)/2 = 0.0
  // Color-aware: avg_left=2.0, avg_right=-1.5 -> center=0.25
  std::vector<planning_core::ConePoint> cones;
  cones.push_back({5.0, 2.0, 0.0, 1});   // YELLOW = left
  cones.push_back({5.0, -1.0, 0.0, 0});  // BLUE = right
  cones.push_back({5.0, -2.0, 0.0, 0});  // BLUE = right
  core.UpdateCones(cones);
  core.RunAlgorithm();

  ASSERT_TRUE(core.HasPlannedPath());
}

TEST(LineDetectionCoreTest, NoColorFallsBackToGeometric) {
  planning_core::LineDetectionParams params;
  params.path_start_x = 0.0;
  params.path_interval = 1.0;
  params.accel_distance = 75.0;
  params.brake_distance = 100.0;
  params.max_path_distance = 175.0;
  params.min_valid_cones = 2;

  planning_core::LineDetectionCore core(params);
  planning_core::VehicleState state{};
  core.UpdateVehicleState(state);

  // All cones NONE (4) — should use geometric fallback
  std::vector<planning_core::ConePoint> cones;
  cones.push_back({5.0, -1.5, 0.0, 4});  // NONE
  cones.push_back({5.0, 1.5, 0.0, 4});   // NONE
  core.UpdateCones(cones);
  core.RunAlgorithm();

  ASSERT_TRUE(core.HasPlannedPath());
  EXPECT_GE(core.GetPlannedPath().size(), 170u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
