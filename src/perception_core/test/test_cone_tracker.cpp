/**
 * @file test_cone_tracker.cpp
 * @brief ConeTracker 单元测试
 */

#include <gtest/gtest.h>

#include <perception_core/cone_tracker.hpp>

namespace {

perception::ConeTracker::Detection MakeDetection(
    double x, double y, double z = 0.0, double confidence = 0.8,
    std::uint8_t color_type = 4) {
  perception::ConeTracker::Detection d;
  d.x = x;
  d.y = y;
  d.z = z;
  d.confidence = confidence;
  d.color_type = color_type;
  return d;
}

}  // namespace

TEST(ConeTrackerTest, ConfirmAfterConsecutiveAssociations) {
  perception::ConeTracker tracker;
  perception::ConeTracker::Config cfg;
  cfg.association_threshold = 1.0;
  cfg.confirm_frames = 2;
  cfg.delete_frames = 5;
  tracker.setConfig(cfg);

  tracker.update({MakeDetection(5.0, 1.0)}, 0.1);
  auto tracks = tracker.getAllTracks();
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].id, 0);
  EXPECT_FALSE(tracks[0].confirmed);

  tracker.update({MakeDetection(5.1, 1.0)}, 0.1);
  auto confirmed = tracker.getConfirmedCones();
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0].id, 0);
  EXPECT_TRUE(confirmed[0].confirmed);
}

TEST(ConeTrackerTest, DeleteAfterConfiguredMisses) {
  perception::ConeTracker tracker;
  perception::ConeTracker::Config cfg;
  cfg.association_threshold = 1.0;
  cfg.confirm_frames = 1;
  cfg.delete_frames = 2;
  tracker.setConfig(cfg);

  tracker.update({MakeDetection(3.0, -0.5)}, 0.1);
  tracker.update({MakeDetection(3.1, -0.5)}, 0.1);
  ASSERT_EQ(tracker.getAllTracks().size(), 1u);

  tracker.update({}, 0.1);
  ASSERT_EQ(tracker.getAllTracks().size(), 1u);

  tracker.update({}, 0.1);
  EXPECT_TRUE(tracker.getAllTracks().empty());
}

TEST(ConeTrackerTest, ReorderedDetectionsKeepTrackIdentity) {
  perception::ConeTracker tracker;
  perception::ConeTracker::Config cfg;
  cfg.association_threshold = 1.0;
  cfg.confirm_frames = 1;
  cfg.delete_frames = 5;
  tracker.setConfig(cfg);

  tracker.update({MakeDetection(-2.0, 1.0), MakeDetection(6.0, -1.0)}, 0.1);
  auto tracks = tracker.getAllTracks();
  ASSERT_EQ(tracks.size(), 2u);

  tracker.update({MakeDetection(6.1, -1.1), MakeDetection(-1.9, 0.9)}, 0.1);
  tracks = tracker.getAllTracks();
  ASSERT_EQ(tracks.size(), 2u);

  int left_id = -1;
  int right_id = -1;
  for (const auto& track : tracks) {
    if (track.x < 2.0) {
      left_id = track.id;
    } else {
      right_id = track.id;
    }
  }
  EXPECT_EQ(left_id, 0);
  EXPECT_EQ(right_id, 1);
}

TEST(ConeTrackerTest, EgoMotionPredictionShiftsTrackInEgoFrame) {
  perception::ConeTracker tracker;
  perception::ConeTracker::Config cfg;
  cfg.association_threshold = 1.0;
  cfg.confirm_frames = 1;
  cfg.delete_frames = 5;
  tracker.setConfig(cfg);

  tracker.update({MakeDetection(10.0, 0.0)}, 0.1);
  ASSERT_EQ(tracker.getAllTracks().size(), 1u);

  perception::EgoMotion ego;
  ego.dx = 1.0;
  ego.dy = 0.0;
  ego.dyaw = 0.0;
  tracker.update({}, 0.1, ego);

  auto tracks = tracker.getAllTracks();
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_NEAR(tracks[0].x, 9.0, 1e-6);
  EXPECT_NEAR(tracks[0].y, 0.0, 1e-6);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
