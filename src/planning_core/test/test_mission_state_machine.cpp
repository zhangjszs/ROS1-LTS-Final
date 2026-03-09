#include "planning_core/mission_state_machine.hpp"

#include <gtest/gtest.h>

using planning_core::MissionFSMConfig;
using planning_core::MissionFSMInput;
using planning_core::MissionState;
using planning_core::MissionStateMachine;

namespace {

MissionFSMConfig DefaultCfg() {
  MissionFSMConfig cfg;
  cfg.required_laps = 2;
  cfg.finish_grace_frames = 5;
  cfg.wait_full_before_stop = true;
  return cfg;
}

MissionFSMInput EmptyInput() {
  MissionFSMInput in;
  return in;
}

TEST(MissionStateMachine, StartsInInit) {
  MissionStateMachine fsm(DefaultCfg());
  EXPECT_EQ(fsm.GetState(), MissionState::INIT);
  EXPECT_EQ(fsm.TotalFrames(), 0);
}

TEST(MissionStateMachine, InitToMapBuilding) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  EXPECT_TRUE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::MAP_BUILDING);
}

TEST(MissionStateMachine, StaysInInit) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  EXPECT_FALSE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::INIT);
}

TEST(MissionStateMachine, MapBuildingToRacing) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);  // INIT -> MAP_BUILDING
  in.loop_closed = true;
  EXPECT_TRUE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::RACING);
}

TEST(MissionStateMachine, RacingToMapBuilding) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);  // -> MAP_BUILDING
  in.loop_closed = true;
  fsm.Update(in);  // -> RACING
  in.loop_closed = false;
  EXPECT_TRUE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::MAP_BUILDING);
}

TEST(MissionStateMachine, RacingToFinishing) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);  // -> MAP_BUILDING
  in.loop_closed = true;
  fsm.Update(in);    // -> RACING
  in.lap_count = 3;  // > required_laps (2)
  EXPECT_TRUE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::FINISHING);
}

TEST(MissionStateMachine, FinishingToStopped) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);
  in.loop_closed = true;
  fsm.Update(in);
  in.lap_count = 3;
  fsm.Update(in);  // -> FINISHING
  in.stop_requested = true;
  EXPECT_TRUE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::STOPPED);
}

TEST(MissionStateMachine, FinishingWaitsForFullPath) {
  MissionFSMConfig cfg = DefaultCfg();
  cfg.wait_full_before_stop = true;
  MissionStateMachine fsm(cfg);
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);
  in.loop_closed = true;
  fsm.Update(in);
  in.lap_count = 3;
  fsm.Update(in);  // -> FINISHING
  in.full_path_published = true;
  in.stop_requested = false;
  EXPECT_TRUE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::STOPPED);
}

TEST(MissionStateMachine, FinishingGraceExpired) {
  MissionFSMConfig cfg = DefaultCfg();
  cfg.finish_grace_frames = 2;
  cfg.wait_full_before_stop = false;
  MissionStateMachine fsm(cfg);
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);
  in.loop_closed = true;
  fsm.Update(in);
  in.lap_count = 3;
  fsm.Update(in);  // -> FINISHING
  // frames_in_state_ increments each Update; need > 2 frames
  fsm.Update(in);               // frame 1
  fsm.Update(in);               // frame 2
  EXPECT_TRUE(fsm.Update(in));  // frame 3 > 2
  EXPECT_EQ(fsm.GetState(), MissionState::STOPPED);
}

TEST(MissionStateMachine, EmergencyStopFromAnyState) {
  for (int start = 0; start < 4; ++start) {
    MissionStateMachine fsm(DefaultCfg());
    MissionFSMInput in = EmptyInput();
    // Drive to desired start state
    if (start >= 1) {
      in.has_valid_path = true;
      fsm.Update(in);
    }
    if (start >= 2) {
      in.loop_closed = true;
      fsm.Update(in);
    }
    if (start >= 3) {
      in.lap_count = 3;
      fsm.Update(in);
    }
    in.emergency_stop = true;
    EXPECT_TRUE(fsm.Update(in));
    EXPECT_EQ(fsm.GetState(), MissionState::STOPPED);
  }
}

TEST(MissionStateMachine, StoppedIsTerminal) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.emergency_stop = true;
  fsm.Update(in);  // -> STOPPED
  in.emergency_stop = false;
  in.has_valid_path = true;
  in.loop_closed = true;
  EXPECT_FALSE(fsm.Update(in));
  EXPECT_EQ(fsm.GetState(), MissionState::STOPPED);
}

TEST(MissionStateMachine, FrameCountTracking) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  fsm.Update(in);
  fsm.Update(in);
  fsm.Update(in);
  EXPECT_EQ(fsm.TotalFrames(), 3);
  EXPECT_EQ(fsm.FramesInCurrentState(), 3);
  in.has_valid_path = true;
  fsm.Update(in);  // transition resets frames_in_state
  EXPECT_EQ(fsm.TotalFrames(), 4);
  EXPECT_EQ(fsm.FramesInCurrentState(), 0);
}

TEST(MissionStateMachine, TransitionHistoryRecorded) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);  // INIT -> MAP_BUILDING
  in.loop_closed = true;
  fsm.Update(in);  // MAP_BUILDING -> RACING
  EXPECT_EQ(fsm.GetHistory().size(), 2u);
  EXPECT_EQ(fsm.GetHistory()[0].from, MissionState::INIT);
  EXPECT_EQ(fsm.GetHistory()[0].to, MissionState::MAP_BUILDING);
  EXPECT_EQ(fsm.GetHistory()[1].from, MissionState::MAP_BUILDING);
  EXPECT_EQ(fsm.GetHistory()[1].to, MissionState::RACING);
}

TEST(MissionStateMachine, ResetClearsState) {
  MissionStateMachine fsm(DefaultCfg());
  MissionFSMInput in = EmptyInput();
  in.has_valid_path = true;
  fsm.Update(in);
  fsm.Reset();
  EXPECT_EQ(fsm.GetState(), MissionState::INIT);
  EXPECT_EQ(fsm.TotalFrames(), 0);
  EXPECT_EQ(fsm.FramesInCurrentState(), 0);
  EXPECT_TRUE(fsm.GetHistory().empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
