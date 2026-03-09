#include "perception_ros/cone_detection_adapter.hpp"

#include <ros/time.h>

#include <chrono>

#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_FusedConeDetections.h>
#include <geometry_msgs/Point32.h>
#include <gtest/gtest.h>

namespace {

using SteadyPoint = perception_ros::ConeDetectionAdapter::SteadyTimePoint;

geometry_msgs::Point32 MakePoint(float x, float y, float z = 0.0f) {
  geometry_msgs::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

autodrive_msgs::HUAT_ConeDetections MakeRaw(const ros::Time& stamp,
                                            const std::vector<uint8_t>& colors) {
  autodrive_msgs::HUAT_ConeDetections msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "velodyne";
  for (size_t i = 0; i < colors.size(); ++i) {
    msg.points.push_back(MakePoint(static_cast<float>(i + 1), static_cast<float>(i)));
    msg.maxPoints.push_back(MakePoint(static_cast<float>(i + 1), static_cast<float>(i), 0.4f));
    msg.minPoints.push_back(MakePoint(static_cast<float>(i + 1), static_cast<float>(i), 0.0f));
    msg.confidence.push_back(0.8f);
    msg.obj_dist.push_back(static_cast<float>(i + 2));
    msg.track_ids.push_back(static_cast<int32_t>(i));
    msg.color_types.push_back(colors[i]);
  }
  return msg;
}

autodrive_msgs::HUAT_FusedConeDetections MakeFused(const ros::Time& stamp,
                                                   const std::vector<uint8_t>& colors) {
  autodrive_msgs::HUAT_FusedConeDetections msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "velodyne";
  msg.lidar_frame = "velodyne";
  for (size_t i = 0; i < colors.size(); ++i) {
    msg.points.push_back(MakePoint(static_cast<float>(i + 1), static_cast<float>(i)));
    msg.obj_dist.push_back(static_cast<float>(i + 2));
    msg.lidar_confidences.push_back(0.8f);
    msg.lidar_color_types.push_back(4);
    msg.track_ids.push_back(static_cast<int32_t>(i));
    msg.fused_color_types.push_back(colors[i]);
    msg.vision_confidences.push_back(900);
    msg.association_status.push_back(0);
    msg.association_scores.push_back(1.0f);
    msg.association_reasons.emplace_back("MATCHED");
  }
  return msg;
}

SteadyPoint MakeSteadyPoint(int64_t millis) {
  return SteadyPoint(std::chrono::milliseconds(millis));
}

}  // namespace

namespace perception_ros {
namespace {

TEST(ConeDetectionAdapterTest, PublishesMergedMessageWhenRawAndFusedShareStamp) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  ConeDetectionAdapter adapter(cfg);

  const ros::Time stamp(10, 0);
  const auto receive_time = MakeSteadyPoint(100);
  const auto raw = MakeRaw(stamp, {2, 3});
  const auto fused = MakeFused(stamp, {0, 1});

  EXPECT_TRUE(adapter.HandleRaw(raw, receive_time).empty());

  const auto outputs = adapter.HandleFused(fused, receive_time + std::chrono::milliseconds(10));
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_TRUE(outputs.front().used_fused);
  EXPECT_EQ(outputs.front().reason, "normal_merge");
  EXPECT_DOUBLE_EQ(outputs.front().wait_ms, 10.0);
  ASSERT_EQ(outputs.front().msg.color_types.size(), 2u);
  EXPECT_EQ(outputs.front().msg.color_types[0], 0);
  EXPECT_EQ(outputs.front().msg.color_types[1], 1);
  EXPECT_EQ(outputs.front().msg.maxPoints.size(), raw.maxPoints.size());
}

TEST(ConeDetectionAdapterTest, PublishesRawAfterHoldoffWhenNoFusedArrives) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  ConeDetectionAdapter adapter(cfg);

  const ros::Time stamp(11, 0);
  const auto receive_time = MakeSteadyPoint(200);
  const auto raw = MakeRaw(stamp, {2, 3});

  EXPECT_TRUE(adapter.HandleRaw(raw, receive_time).empty());
  EXPECT_TRUE(adapter.Flush(receive_time + std::chrono::milliseconds(40)).empty());

  const auto outputs = adapter.Flush(receive_time + std::chrono::milliseconds(60));
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_FALSE(outputs.front().used_fused);
  EXPECT_EQ(outputs.front().reason, "no_fused");
  EXPECT_DOUBLE_EQ(outputs.front().wait_ms, 60.0);
  ASSERT_EQ(outputs.front().msg.color_types.size(), 2u);
  EXPECT_EQ(outputs.front().msg.color_types[0], 2);
  EXPECT_EQ(outputs.front().msg.color_types[1], 3);
  const auto& stats = adapter.stats();
  EXPECT_EQ(stats.published_raw_fallback, 1u);
  EXPECT_EQ(stats.no_fused, 1u);
}

TEST(ConeDetectionAdapterTest, PublishesMergedWhenFusedArrivesBeforeRaw) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  ConeDetectionAdapter adapter(cfg);

  const ros::Time stamp(12, 0);
  const auto receive_time = MakeSteadyPoint(300);
  const auto raw = MakeRaw(stamp, {2, 3});
  const auto fused = MakeFused(stamp, {0, 1});

  EXPECT_TRUE(adapter.HandleFused(fused, receive_time).empty());

  const auto outputs = adapter.HandleRaw(raw, receive_time + std::chrono::milliseconds(10));
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_TRUE(outputs.front().used_fused);
  EXPECT_EQ(outputs.front().reason, "normal_merge");
  EXPECT_DOUBLE_EQ(outputs.front().wait_ms, 0.0);
  EXPECT_EQ(outputs.front().msg.color_types[0], 0);
  EXPECT_EQ(outputs.front().msg.color_types[1], 1);
}

TEST(ConeDetectionAdapterTest, FallsBackToRawWhenFusedCountMismatches) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  ConeDetectionAdapter adapter(cfg);

  const ros::Time stamp(13, 0);
  const auto receive_time = MakeSteadyPoint(400);
  const auto raw = MakeRaw(stamp, {2, 3});
  const auto fused = MakeFused(stamp, {0});

  EXPECT_TRUE(adapter.HandleRaw(raw, receive_time).empty());

  const auto outputs = adapter.HandleFused(fused, receive_time + std::chrono::milliseconds(10));
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_FALSE(outputs.front().used_fused);
  EXPECT_EQ(outputs.front().reason, "count_mismatch");
  EXPECT_DOUBLE_EQ(outputs.front().wait_ms, 10.0);
  EXPECT_EQ(outputs.front().msg.color_types[0], 2);
  EXPECT_EQ(outputs.front().msg.color_types[1], 3);
  const auto& stats = adapter.stats();
  EXPECT_EQ(stats.count_mismatch, 1u);
  EXPECT_EQ(stats.published_raw_fallback, 1u);
}

TEST(ConeDetectionAdapterTest, RejectsOverdueFusedAndPublishesRawFallbackOnMergePath) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  ConeDetectionAdapter adapter(cfg);

  const ros::Time stamp(14, 0);
  const auto receive_time = MakeSteadyPoint(500);
  const auto raw = MakeRaw(stamp, {2, 3});
  const auto fused = MakeFused(stamp, {0, 1});

  EXPECT_TRUE(adapter.HandleRaw(raw, receive_time).empty());

  const auto outputs = adapter.HandleFused(fused, receive_time + std::chrono::milliseconds(60));
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_FALSE(outputs.front().used_fused);
  EXPECT_EQ(outputs.front().reason, "late_fused");
  EXPECT_DOUBLE_EQ(outputs.front().wait_ms, 60.0);
  EXPECT_EQ(outputs.front().msg.color_types[0], 2);
  EXPECT_EQ(outputs.front().msg.color_types[1], 3);

  const auto& stats = adapter.stats();
  EXPECT_EQ(stats.published_raw_fallback, 1u);
  EXPECT_EQ(stats.no_fused, 1u);
  EXPECT_EQ(stats.late_fused, 1u);
}

TEST(ConeDetectionAdapterTest, IgnoresFusedAfterRawFallbackFinalized) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  ConeDetectionAdapter adapter(cfg);

  const ros::Time stamp(15, 0);
  const auto receive_time = MakeSteadyPoint(600);
  const auto raw = MakeRaw(stamp, {2, 3});
  const auto fused = MakeFused(stamp, {0, 1});

  EXPECT_TRUE(adapter.HandleRaw(raw, receive_time).empty());
  const auto raw_outputs = adapter.Flush(receive_time + std::chrono::milliseconds(60));
  ASSERT_EQ(raw_outputs.size(), 1u);
  EXPECT_FALSE(raw_outputs.front().used_fused);
  EXPECT_EQ(raw_outputs.front().reason, "no_fused");
  EXPECT_DOUBLE_EQ(raw_outputs.front().wait_ms, 60.0);

  EXPECT_TRUE(adapter.HandleFused(fused, receive_time + std::chrono::milliseconds(70)).empty());

  const auto& stats = adapter.stats();
  EXPECT_EQ(stats.published_raw_fallback, 1u);
  EXPECT_EQ(stats.no_fused, 1u);
  EXPECT_EQ(stats.late_fused, 1u);
  EXPECT_EQ(stats.duplicate_after_finalize, 1u);
}

TEST(ConeDetectionAdapterTest, IgnoresRawAfterFusedPublishFinalized) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  ConeDetectionAdapter adapter(cfg);

  const ros::Time stamp(16, 0);
  const auto receive_time = MakeSteadyPoint(700);
  const auto raw = MakeRaw(stamp, {2, 3});
  const auto fused = MakeFused(stamp, {0, 1});

  EXPECT_TRUE(adapter.HandleFused(fused, receive_time).empty());
  const auto fused_outputs = adapter.HandleRaw(raw, receive_time + std::chrono::milliseconds(10));
  ASSERT_EQ(fused_outputs.size(), 1u);
  EXPECT_TRUE(fused_outputs.front().used_fused);
  EXPECT_EQ(fused_outputs.front().reason, "normal_merge");
  EXPECT_DOUBLE_EQ(fused_outputs.front().wait_ms, 0.0);

  EXPECT_TRUE(adapter.HandleRaw(raw, receive_time + std::chrono::milliseconds(20)).empty());

  const auto& stats = adapter.stats();
  EXPECT_EQ(stats.published_fused, 1u);
  EXPECT_EQ(stats.duplicate_after_finalize, 1u);
}

TEST(ConeDetectionAdapterTest, CountsStaleFusedAndCacheEvictions) {
  ConeDetectionAdapter::Config cfg;
  cfg.raw_holdoff = ros::Duration(0.05);
  cfg.finalized_ttl = ros::Duration(2.0);
  cfg.max_pending = 1;
  ConeDetectionAdapter adapter(cfg);

  const auto receive_time = MakeSteadyPoint(800);
  const auto fused_a = MakeFused(ros::Time(17, 0), {0, 1});
  const auto fused_b = MakeFused(ros::Time(18, 0), {1, 0});

  EXPECT_TRUE(adapter.HandleFused(fused_a, receive_time).empty());
  EXPECT_TRUE(adapter.HandleFused(fused_b, receive_time + std::chrono::milliseconds(1)).empty());
  EXPECT_TRUE(adapter.Flush(receive_time + std::chrono::milliseconds(70)).empty());

  const auto& stats = adapter.stats();
  EXPECT_EQ(stats.cache_evicted, 1u);
  EXPECT_EQ(stats.stale_fused, 1u);
  EXPECT_EQ(stats.pending_cache_size_max, 2u);
}

}  // namespace
}  // namespace perception_ros

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
