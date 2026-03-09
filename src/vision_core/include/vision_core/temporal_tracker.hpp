#pragma once
#include "vision_core/types.hpp"

#include <deque>
#include <vector>

namespace vision_core {

struct TrackerConfig {
  int history_frames = 3;
  float iou_threshold = 0.3f;
  int min_hits_to_output = 2;
  int max_miss_frames = 1;
};

class TemporalTracker {
 public:
  explicit TemporalTracker(const TrackerConfig& config = {});
  std::vector<Detection> update(const std::vector<Detection>& dets);
  void reset();

 private:
  struct Track {
    Detection last_det;
    int hits = 0;
    int misses = 0;
  };
  float computeIoU(const Detection& a, const Detection& b) const;
  TrackerConfig config_;
  std::vector<Track> tracks_;
};

}  // namespace vision_core
