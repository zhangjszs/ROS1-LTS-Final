#ifndef PLANNING_CORE_MISSION_STATE_MACHINE_HPP_
#define PLANNING_CORE_MISSION_STATE_MACHINE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace planning_core
{

enum class MissionState : uint8_t
{
  INIT = 0,
  MAP_BUILDING = 1,
  RACING = 2,
  FINISHING = 3,
  STOPPED = 4
};

struct MissionFSMConfig
{
  int required_laps{10};
  int finish_grace_frames{300};
  bool wait_full_before_stop{true};
};

struct MissionFSMInput
{
  bool has_valid_path{false};
  bool loop_closed{false};
  int lap_count{0};
  bool full_path_published{false};
  bool stop_requested{false};
  bool emergency_stop{false};
};

struct MissionTransition
{
  MissionState from;
  MissionState to;
  int frame{0};
  std::string reason;
};

class MissionStateMachine
{
public:
  MissionStateMachine() = default;

  explicit MissionStateMachine(const MissionFSMConfig &cfg)
    : cfg_(cfg)
  {
  }

  /// Returns true if state changed.
  bool Update(const MissionFSMInput &in)
  {
    ++total_frames_;
    ++frames_in_state_;

    if (state_ == MissionState::STOPPED)
    {
      return false;  // terminal
    }

    // Emergency stop from any state
    if (in.emergency_stop)
    {
      return Transition(MissionState::STOPPED, "emergency_stop");
    }

    switch (state_)
    {
      case MissionState::INIT:
        if (in.has_valid_path)
        {
          return Transition(MissionState::MAP_BUILDING, "has_valid_path");
        }
        break;

      case MissionState::MAP_BUILDING:
        if (in.loop_closed)
        {
          return Transition(MissionState::RACING, "loop_closed");
        }
        break;

      case MissionState::RACING:
        if (in.lap_count > cfg_.required_laps)
        {
          return Transition(MissionState::FINISHING, "lap_count_exceeded");
        }
        if (!in.loop_closed)
        {
          return Transition(MissionState::MAP_BUILDING, "loop_lost");
        }
        break;

      case MissionState::FINISHING:
        if (in.stop_requested)
        {
          return Transition(MissionState::STOPPED, "stop_requested");
        }
        if (cfg_.wait_full_before_stop && in.full_path_published)
        {
          return Transition(MissionState::STOPPED, "full_path_published");
        }
        if (frames_in_state_ > cfg_.finish_grace_frames)
        {
          return Transition(MissionState::STOPPED, "grace_expired");
        }
        break;

      case MissionState::STOPPED:
        break;  // unreachable, handled above
    }
    return false;
  }

  MissionState GetState() const { return state_; }
  int FramesInCurrentState() const { return frames_in_state_; }
  int TotalFrames() const { return total_frames_; }
  const std::vector<MissionTransition> &GetHistory() const { return history_; }

  void Reset()
  {
    state_ = MissionState::INIT;
    total_frames_ = 0;
    frames_in_state_ = 0;
    history_.clear();
  }

  static const char *StateName(MissionState s)
  {
    switch (s)
    {
      case MissionState::INIT:          return "INIT";
      case MissionState::MAP_BUILDING:  return "MAP_BUILDING";
      case MissionState::RACING:        return "RACING";
      case MissionState::FINISHING:     return "FINISHING";
      case MissionState::STOPPED:       return "STOPPED";
    }
    return "UNKNOWN";
  }

private:
  bool Transition(MissionState to, const char *reason)
  {
    MissionTransition t;
    t.from = state_;
    t.to = to;
    t.frame = total_frames_;
    t.reason = reason;
    history_.push_back(t);
    state_ = to;
    frames_in_state_ = 0;
    return true;
  }

  MissionFSMConfig cfg_;
  MissionState state_{MissionState::INIT};
  int total_frames_{0};
  int frames_in_state_{0};
  std::vector<MissionTransition> history_;
};

} // namespace planning_core

#endif // PLANNING_CORE_MISSION_STATE_MACHINE_HPP_
