#include <gtest/gtest.h>

#include "vision_core/types.hpp"

using namespace vision_core;

TEST(ClassMapping, UsesDefaultModelOrderWhenNoCustomMap) {
  EXPECT_EQ(modelClassToColorType(0), BLUE);
  EXPECT_EQ(modelClassToColorType(1), YELLOW);
  EXPECT_EQ(modelClassToColorType(2), ORANGE_SMALL);
  EXPECT_EQ(modelClassToColorType(3), ORANGE_BIG);
  EXPECT_EQ(modelClassToColorType(4), RED);
  EXPECT_EQ(modelClassToColorType(9), NONE);
}

TEST(ClassMapping, UsesCustomMapWhenProvided) {
  // For class names [red, blue, yellow], map to cone enums [RED, BLUE, YELLOW].
  std::vector<uint8_t> custom = {5, 0, 1};
  EXPECT_EQ(modelClassToColorType(0, custom), RED);
  EXPECT_EQ(modelClassToColorType(1, custom), BLUE);
  EXPECT_EQ(modelClassToColorType(2, custom), YELLOW);
  EXPECT_EQ(modelClassToColorType(4, custom), NONE);
}

TEST(ClassMapping, InvalidCustomValueFallsBackToNone) {
  std::vector<uint8_t> custom = {9};
  EXPECT_EQ(modelClassToColorType(0, custom), NONE);
}
