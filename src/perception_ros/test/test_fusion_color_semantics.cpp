#include <cstdint>

#include <gtest/gtest.h>

#include "perception_ros/fusion_color_semantics.hpp"

namespace {

constexpr std::uint8_t kBlue = 0;
constexpr std::uint8_t kYellowSmall = 1;
constexpr std::uint8_t kYellowBig = 2;
constexpr std::uint8_t kRed = 3;
constexpr std::uint8_t kNone = 4;

TEST(FusionColorSemanticsTest, KeepsDefinitiveBoundaryColors) {
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kBlue, 2.0f), kBlue);
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kRed, -2.0f), kRed);
}

TEST(FusionColorSemanticsTest, MapsYellowAndNoneByLateralPosition) {
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kYellowSmall, 1.0f), kRed);
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kYellowBig, 0.2f), kRed);
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kNone, -0.2f), kBlue);
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kYellowSmall, -1.0f), kBlue);
}

TEST(FusionColorSemanticsTest, ZeroLateralDefaultsToRightBoundaryBlue) {
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kYellowSmall, 0.0f), kBlue);
  EXPECT_EQ(perception_ros::FusionFallbackSemanticColor(kNone, 0.0f), kBlue);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
