#include <gtest/gtest.h>

#include "fsd_common/cone_types.hpp"

using fsd_common::ConeType;

TEST(ConeTypeTest, ConstantsHaveCorrectValues) {
  EXPECT_EQ(ConeType::kBlue, 0);
  EXPECT_EQ(ConeType::kYellowSmall, 1);
  EXPECT_EQ(ConeType::kYellowBig, 2);
  EXPECT_EQ(ConeType::kRed, 3);
  EXPECT_EQ(ConeType::kNone, 4);
}

TEST(ConeTypeTest, IsColoredIdentifiesColoredCones) {
  EXPECT_TRUE(ConeType::IsColored(ConeType::kBlue));
  EXPECT_TRUE(ConeType::IsColored(ConeType::kRed));
  EXPECT_TRUE(ConeType::IsColored(ConeType::kYellowSmall));
  EXPECT_TRUE(ConeType::IsColored(ConeType::kYellowBig));
  EXPECT_FALSE(ConeType::IsColored(ConeType::kNone));
  EXPECT_FALSE(ConeType::IsColored(99));  // Invalid type
}

TEST(ConeTypeTest, IsBoundaryIdentifiesBoundaryCones) {
  EXPECT_TRUE(ConeType::IsBoundary(ConeType::kBlue));
  EXPECT_TRUE(ConeType::IsBoundary(ConeType::kRed));
  EXPECT_FALSE(ConeType::IsBoundary(ConeType::kYellowSmall));
  EXPECT_FALSE(ConeType::IsBoundary(ConeType::kYellowBig));
  EXPECT_FALSE(ConeType::IsBoundary(ConeType::kNone));
}

TEST(ConeTypeTest, IsYellowIdentifiesYellowCones) {
  EXPECT_TRUE(ConeType::IsYellow(ConeType::kYellowSmall));
  EXPECT_TRUE(ConeType::IsYellow(ConeType::kYellowBig));
  EXPECT_FALSE(ConeType::IsYellow(ConeType::kBlue));
  EXPECT_FALSE(ConeType::IsYellow(ConeType::kRed));
  EXPECT_FALSE(ConeType::IsYellow(ConeType::kNone));
}

TEST(ConeTypeTest, LegacyAliasesMatchCanonicalValues) {
  EXPECT_EQ(fsd_common::kConeBlue, ConeType::kBlue);
  EXPECT_EQ(fsd_common::kConeYellowSmall, ConeType::kYellowSmall);
  EXPECT_EQ(fsd_common::kConeYellowBig, ConeType::kYellowBig);
  EXPECT_EQ(fsd_common::kConeRed, ConeType::kRed);
  EXPECT_EQ(fsd_common::kConeNone, ConeType::kNone);
}

TEST(ConeTypeTest, LegacyLocationAliasesMapCorrectly) {
  // Legacy localization_core aliases should map to new values
  EXPECT_EQ(fsd_common::kConeYellow, ConeType::kYellowSmall);
  EXPECT_EQ(fsd_common::kConeOrangeSmall, ConeType::kYellowSmall);
  EXPECT_EQ(fsd_common::kConeOrangeBig, ConeType::kYellowBig);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
