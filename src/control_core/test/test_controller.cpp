#include <gtest/gtest.h>
#include "control_core/controller_base.hpp"
#include "control_core/line_controller.hpp"

TEST(ControllerBaseTest, Initialization) {
  control_core::LineController controller;
  EXPECT_FALSE(controller.HasPath());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
