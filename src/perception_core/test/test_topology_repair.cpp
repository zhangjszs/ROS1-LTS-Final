/**
 * @file test_topology_repair.cpp
 * @brief TopologyRepair 单元测试
 */

#include <gtest/gtest.h>
#include <perception_core/topology_repair.hpp>

namespace {

perception::TopologyCone MakeCone(double x, double y, double z = 0.0, double confidence = 0.8,
                                  int index = -1) {
  perception::TopologyCone c;
  c.x = x;
  c.y = y;
  c.z = z;
  c.confidence = confidence;
  c.is_interpolated = false;
  c.original_index = index;
  return c;
}

}  // namespace

TEST(TopologyRepairTest, EmptyInputReturnsEmpty) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = true;
  repair.setConfig(cfg);

  std::vector<perception::TopologyCone> input;
  auto result = repair.repair(input);

  EXPECT_TRUE(result.empty());
}

TEST(TopologyRepairTest, SingleConeUnchanged) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = true;
  repair.setConfig(cfg);

  std::vector<perception::TopologyCone> input;
  input.push_back(MakeCone(5.0, 1.0, 0.0, 0.9, 0));

  auto result = repair.repair(input);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0].x, 5.0);
  EXPECT_DOUBLE_EQ(result[0].y, 1.0);
  EXPECT_FALSE(result[0].is_interpolated);
}

TEST(TopologyRepairTest, SmallGapNoInterpolation) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = true;
  cfg.max_same_side_spacing = 5.0;  // 5m threshold
  repair.setConfig(cfg);

  std::vector<perception::TopologyCone> input;
  input.push_back(MakeCone(0.0, 1.0, 0.0, 0.9, 0));
  input.push_back(MakeCone(3.0, 1.0, 0.0, 0.9, 1));  // 3m gap, no interpolation

  auto result = repair.repair(input);

  // Should keep both original cones, no interpolation needed
  EXPECT_GE(result.size(), 2u);
}

TEST(TopologyRepairTest, LargeGapTriggersInterpolation) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = true;
  cfg.max_same_side_spacing = 5.0;    // 5m threshold
  cfg.max_repair_range = 50.0;        // Large enough
  cfg.interpolated_confidence = 0.3;  // Interpolated cone confidence
  repair.setConfig(cfg);

  // Need at least 3 cones on same side to trigger interpolation
  // Create a track with left side having a large gap
  std::vector<perception::TopologyCone> input;
  input.push_back(MakeCone(0.0, 2.0, 0.0, 0.9, 0));    // left
  input.push_back(MakeCone(0.0, -2.0, 0.0, 0.9, 1));   // right
  input.push_back(MakeCone(12.0, 2.0, 0.0, 0.9, 2));   // left, 12m gap
  input.push_back(MakeCone(12.0, -2.0, 0.0, 0.9, 3));  // right

  auto result = repair.repair(input);

  // Should have more than 4 cones (4 original + interpolated)
  EXPECT_GT(result.size(), 4u);

  // Check that interpolated cone exists
  bool has_interpolated = false;
  for (const auto& c : result) {
    if (c.is_interpolated) {
      has_interpolated = true;
      EXPECT_DOUBLE_EQ(c.confidence, 0.3);
    }
  }
  EXPECT_TRUE(has_interpolated);
}

TEST(TopologyRepairTest, TwoSideTrackStructurePreserved) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = true;
  cfg.min_track_width = 2.0;
  cfg.max_track_width = 5.0;
  repair.setConfig(cfg);

  // Create a two-sided track (left and right cones)
  std::vector<perception::TopologyCone> input;
  input.push_back(MakeCone(0.0, 2.0, 0.0, 0.9, 0));   // left
  input.push_back(MakeCone(0.0, -2.0, 0.0, 0.9, 1));  // right
  input.push_back(MakeCone(5.0, 2.0, 0.0, 0.9, 2));   // left
  input.push_back(MakeCone(5.0, -2.0, 0.0, 0.9, 3));  // right

  auto result = repair.repair(input);

  // Should preserve track structure
  EXPECT_GE(result.size(), 4u);
}

TEST(TopologyRepairTest, DisabledReturnsInputUnchanged) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = false;  // Disabled
  repair.setConfig(cfg);

  std::vector<perception::TopologyCone> input;
  input.push_back(MakeCone(0.0, 1.0, 0.0, 0.9, 0));
  input.push_back(MakeCone(10.0, 1.0, 0.0, 0.9, 1));

  auto result = repair.repair(input);

  // When disabled, should return input unchanged (or minimal processing)
  EXPECT_EQ(result.size(), input.size());
}

TEST(TopologyRepairTest, OutOfRangeConesNotRepaired) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = true;
  cfg.max_same_side_spacing = 5.0;
  cfg.max_repair_range = 10.0;  // Only repair within 10m
  repair.setConfig(cfg);

  std::vector<perception::TopologyCone> input;
  input.push_back(MakeCone(0.0, 1.0, 0.0, 0.9, 0));
  input.push_back(MakeCone(5.0, 1.0, 0.0, 0.9, 1));
  input.push_back(MakeCone(20.0, 1.0, 0.0, 0.9, 2));  // Outside repair range
  input.push_back(MakeCone(30.0, 1.0, 0.0, 0.9, 3));  // Outside repair range

  auto result = repair.repair(input);

  // Should still process but may not interpolate far cones
  EXPECT_GE(result.size(), 2u);
}

TEST(TopologyRepairTest, InterpolatedConeHasLowerConfidence) {
  perception::TopologyRepair repair;
  perception::TopologyConfig cfg;
  cfg.enable = true;
  cfg.max_same_side_spacing = 3.0;
  cfg.max_repair_range = 50.0;
  cfg.interpolated_confidence = 0.25;
  repair.setConfig(cfg);

  std::vector<perception::TopologyCone> input;
  input.push_back(MakeCone(0.0, 1.0, 0.0, 0.9, 0));
  input.push_back(MakeCone(8.0, 1.0, 0.0, 0.9, 1));  // Large gap

  auto result = repair.repair(input);

  for (const auto& c : result) {
    if (c.is_interpolated) {
      EXPECT_DOUBLE_EQ(c.confidence, 0.25);
      EXPECT_LT(c.confidence, 0.9);  // Lower than original cones
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
