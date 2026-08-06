#include "planning_core/boundary_graph.hpp"

#include <gtest/gtest.h>

using planning_core::AutocrossModeStateMachine;
using planning_core::BoundaryEdge;
using planning_core::BoundaryGraph;
using planning_core::BoundaryGraphConfig;

// ==================== AutocrossModeStateMachine Tests ====================

TEST(AutocrossModeStateMachine, StartsInMapBuildSafe) {
  AutocrossModeStateMachine sm;
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::MAP_BUILD_SAFE);
  EXPECT_DOUBLE_EQ(sm.GetSpeedMultiplier(), 0.7);
}

TEST(AutocrossModeStateMachine, EntersFastLapAfterStableFrames) {
  AutocrossModeStateMachine sm;
  // Feed 30 consecutive stable frames (kEnterFastLapThreshold = 30)
  for (int i = 0; i < 30; ++i) {
    sm.Update(true);
  }
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::FAST_LAP);
  EXPECT_DOUBLE_EQ(sm.GetSpeedMultiplier(), 1.0);
}

TEST(AutocrossModeStateMachine, ExitsFastLapAfterUnstableFrames) {
  AutocrossModeStateMachine sm;

  // Enter FAST_LAP (need kEnterFastLapThreshold=30 stable frames)
  for (int i = 0; i < 30; ++i) {
    sm.Update(true);
  }
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::FAST_LAP);

  // To exit FAST_LAP, the counter must drop from +30 to -10
  // (kExitFastLapThreshold=10), requiring 40 unstable frames.
  for (int i = 0; i < 40; ++i) {
    sm.Update(false);
  }
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::MAP_BUILD_SAFE);
  EXPECT_DOUBLE_EQ(sm.GetSpeedMultiplier(), 0.7);
}

TEST(AutocrossModeStateMachine, DoesNotEnterFastLapWithInsufficientFrames) {
  AutocrossModeStateMachine sm;
  // Feed only 29 stable frames (one short of threshold)
  for (int i = 0; i < 29; ++i) {
    sm.Update(true);
  }
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::MAP_BUILD_SAFE);
}

TEST(AutocrossModeStateMachine, DoesNotExitFastLapWithInsufficientUnstableFrames) {
  AutocrossModeStateMachine sm;

  // Enter FAST_LAP
  for (int i = 0; i < 30; ++i) {
    sm.Update(true);
  }
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::FAST_LAP);

  // Feed only 39 unstable frames (one short of the 40 needed to go +30→-10)
  for (int i = 0; i < 39; ++i) {
    sm.Update(false);
  }
  // Should still be in FAST_LAP (counter at -9, not yet -10)
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::FAST_LAP);
}

TEST(AutocrossModeStateMachine, ResetReturnsToMapBuildSafe) {
  AutocrossModeStateMachine sm;

  // Enter FAST_LAP
  for (int i = 0; i < 30; ++i) {
    sm.Update(true);
  }
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::FAST_LAP);

  sm.Reset();
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::MAP_BUILD_SAFE);
}

TEST(AutocrossModeStateMachine, HysteresisOscillation) {
  AutocrossModeStateMachine sm;

  // Alternate: should not trigger either transition
  for (int i = 0; i < 60; ++i) {
    sm.Update(i % 2 == 0);
  }
  // Counter oscillates around 0, should stay in MAP_BUILD_SAFE
  EXPECT_EQ(sm.GetMode(), AutocrossModeStateMachine::Mode::MAP_BUILD_SAFE);
}

// ==================== BoundaryGraph Basic Tests ====================

TEST(BoundaryGraph, EmptyInput) {
  BoundaryGraph graph;
  std::vector<std::tuple<double, double, int, double>> cones;
  EXPECT_FALSE(graph.BuildGraph(cones));
}

TEST(BoundaryGraph, SingleConeInsufficient) {
  BoundaryGraph graph;
  std::vector<std::tuple<double, double, int, double>> cones = {{1.0, 0.5, 0, 0.9}};
  EXPECT_FALSE(graph.BuildGraph(cones));
}

TEST(BoundaryGraph, TwoConesDifferentColors) {
  BoundaryGraphConfig cfg;
  cfg.use_color_rules = true;
  BoundaryGraph graph(cfg);

  // Blue cone on right (y < 0), Red cone on left (y > 0)
  std::vector<std::tuple<double, double, int, double>> cones = {
      {2.0, -1.5, cfg.right_color, 0.9},  // BLUE = right
      {2.0, 1.5, cfg.left_color, 0.9},    // RED = left
  };
  EXPECT_TRUE(graph.BuildGraph(cones));
  EXPECT_TRUE(graph.IsValid());
}

TEST(BoundaryGraph, ExtractCorridorWithTwoBoundaries) {
  BoundaryGraphConfig cfg;
  BoundaryGraph graph(cfg);

  std::vector<std::tuple<double, double, int, double>> cones = {
      {1.0, -1.5, cfg.right_color, 0.9},
      {1.0, 1.5, cfg.left_color, 0.9},
      {3.0, -1.5, cfg.right_color, 0.9},
      {3.0, 1.5, cfg.left_color, 0.9},
  };
  EXPECT_TRUE(graph.BuildGraph(cones));

  auto corridor = graph.ExtractCorridor();
  EXPECT_TRUE(corridor.is_valid);
  EXPECT_EQ(corridor.centerline.size(), 2u);
  // Centerline y should be near 0 (midpoint of -1.5 and 1.5)
  for (const auto& [cx, cy] : corridor.centerline) {
    EXPECT_NEAR(cy, 0.0, 0.01);
  }
  // Width should be ~3.0
  for (double w : corridor.width) {
    EXPECT_NEAR(w, 3.0, 0.01);
  }
}

TEST(BoundaryGraph, BuildEdgesSameBoundary) {
  // Two nearby nodes on the same boundary should be connected
  BoundaryGraphConfig cfg;
  cfg.max_edge_length = 5.0;
  BoundaryGraph graph(cfg);

  std::vector<std::tuple<double, double, int, double>> cones = {
      {0.0, 0.0, cfg.left_color, 0.9},
      {2.0, 0.0, cfg.left_color, 0.9},
      {10.0, 0.0, cfg.left_color, 0.9},  // Far away — no edge
      {0.0, -3.0, cfg.right_color, 0.9},
  };
  EXPECT_TRUE(graph.BuildGraph(cones));

  const auto& edges = graph.GetEdges();
  // Should have at least 1 edge (between the two nearby left nodes)
  // and 1 cross-track edge (left-right within track width)
  EXPECT_GE(edges.size(), 2u);

  // Verify the two close left nodes are connected
  bool found_close_pair = false;
  for (const auto& e : edges) {
    if (e.length < 5.0 && e.length > 1.5) {  // ~2.0 apart
      found_close_pair = true;
    }
  }
  EXPECT_TRUE(found_close_pair);
}

TEST(BoundaryGraph, BuildEdgesRespectsMaxEdgeLength) {
  // Nodes farther than max_edge_length should NOT be connected
  BoundaryGraphConfig cfg;
  cfg.max_edge_length = 3.0;
  BoundaryGraph graph(cfg);

  std::vector<std::tuple<double, double, int, double>> cones = {
      {0.0, 0.0, cfg.left_color, 0.9},
      {5.0, 0.0, cfg.left_color, 0.9},   // 5m apart > max_edge_length=3
      {0.0, -3.0, cfg.right_color, 0.9},
  };
  EXPECT_TRUE(graph.BuildGraph(cones));

  const auto& edges = graph.GetEdges();
  // The two left nodes are 5m apart, exceeding max_edge_length=3
  // So no same-boundary edge between them. Only cross-track edges expected.
  for (const auto& e : edges) {
    if (e.type == BoundaryEdge::Type::LEFT_BOUNDARY) {
      EXPECT_LT(e.length, cfg.max_edge_length);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
