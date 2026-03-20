#include "planning_ros/contract_utils.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <autodrive_msgs/HUAT_PathLimits.h>
#include <autodrive_msgs/HUAT_PathLimitsV2.h>

namespace {

geometry_msgs::Point MakePoint(double x, double y) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

}  // namespace

TEST(PathLimitsV2ContractTest, ConvertPreservesShapeAndMode) {
  autodrive_msgs::HUAT_PathLimits v1;
  v1.path = {MakePoint(0.0, 0.0), MakePoint(1.0, 0.0), MakePoint(2.0, 0.0)};
  v1.target_speeds = {2.0, 2.5, 3.0};
  v1.curvatures = {0.0, 0.01, 0.02};

  autodrive_msgs::HUAT_PathLimitsV2 v2;
  planning_ros::contract::ConvertPathLimitsV1ToV2(v1, v2,
                                                   autodrive_msgs::HUAT_PathLimitsV2::MODE_TRACK);

  EXPECT_EQ(v2.mode, autodrive_msgs::HUAT_PathLimitsV2::MODE_TRACK);
  EXPECT_EQ(v2.path.size(), v1.path.size());
  EXPECT_EQ(v2.s.size(), v1.path.size());
  EXPECT_EQ(v2.yaw.size(), v1.path.size());
  EXPECT_EQ(v2.target_speeds.size(), v1.path.size());
  EXPECT_EQ(v2.target_accels.size(), v1.path.size());
  EXPECT_EQ(v2.time_to_point.size(), v1.path.size());
  EXPECT_DOUBLE_EQ(v2.max_available_speed, 3.0);
}

TEST(PathLimitsV2ContractTest, ConvertHandlesEmptyTargetSpeeds) {
  autodrive_msgs::HUAT_PathLimits v1;
  v1.path = {MakePoint(0.0, 0.0), MakePoint(1.0, 1.0)};
  v1.curvatures = {0.0, 0.1};
  v1.target_speeds.clear();

  autodrive_msgs::HUAT_PathLimitsV2 v2;
  planning_ros::contract::ConvertPathLimitsV1ToV2(v1, v2);

  EXPECT_DOUBLE_EQ(v2.max_available_speed, 0.0);
}

TEST(PathLimitsV2ContractTest, FinalizeAndValidateShape) {
  autodrive_msgs::HUAT_PathLimitsV2 v2;
  v2.path = {MakePoint(0.0, 0.0), MakePoint(1.0, 0.0), MakePoint(2.0, 0.0)};
  v2.target_speeds = {1.0, 1.1, 1.2};
  v2.curvatures = {0.0, 0.05, 0.05};
  v2.target_accels = {0.0, 0.0, 0.0};
  v2.time_to_point = {0.0, 1.0, 2.0};

  planning_ros::contract::FinalizePathLimitsV2Message(v2, ros::Time(12.0), "world");

  std::string error;
  EXPECT_TRUE(planning_ros::contract::ValidatePathLimitsV2Shape(v2, &error)) << error;
  EXPECT_EQ(v2.header.frame_id, "world");
  EXPECT_TRUE(v2.header.stamp.isValid());
  EXPECT_TRUE(v2.stamp.isValid());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
