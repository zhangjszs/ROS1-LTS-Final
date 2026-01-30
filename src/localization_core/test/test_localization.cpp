#include <gtest/gtest.h>
#include "localization_core/imu_state_estimator.hpp"

TEST(LocalizationCoreTest, Initialization) {
  localization_core::ImuStateEstimatorParams params;
  localization_core::ImuStateEstimator estimator(params);
  EXPECT_FALSE(estimator.initialized());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
