#pragma once
#include <string>

namespace fsd_common {

struct ControlMode {
  static constexpr int kTest    = 1;
  static constexpr int kLine    = 2;
  static constexpr int kSkidpad = 3;
  static constexpr int kTrack   = 4;
  static constexpr int kEbs     = 5;

  static int FromString(const std::string &name) {
    if (name == "test")    return kTest;
    if (name == "line")    return kLine;
    if (name == "skidpad") return kSkidpad;
    if (name == "track")   return kTrack;
    if (name == "ebs")     return kEbs;
    return -1;
  }

  static const char* ToString(int mode) {
    switch (mode) {
      case kTest:    return "test";
      case kLine:    return "line";
      case kSkidpad: return "skidpad";
      case kTrack:   return "track";
      case kEbs:     return "ebs";
      default:       return "unknown";
    }
  }
};

}  // namespace fsd_common
