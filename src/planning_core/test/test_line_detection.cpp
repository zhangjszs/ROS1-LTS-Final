#include <gtest/gtest.h>
#include "planning_core/line_detection_core.hpp"

TEST(LineDetectionCoreTest, Initialization) {
  planning_core::LineDetectionParams params;
  planning_core::LineDetectionCore core(params);
  EXPECT_FALSE(core.IsFinished());
  EXPECT_FALSE(core.HasPlannedPath());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
