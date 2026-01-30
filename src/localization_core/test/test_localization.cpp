#include <gtest/gtest.h>
#include "localization_core/imu_state_estimator.hpp"

TEST(LocalizationCoreTest, Initialization) {
  localization_core::IMUStateEstimator estimator;
  EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
