#include "vision_core/temporal_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace vision_core {

TemporalTracker::TemporalTracker(const TrackerConfig& c) : config_(c) {}

void TemporalTracker::reset() { tracks_.clear(); }

float TemporalTracker::computeIoU(const Detection& a, const Detection& b) const {
  float ax1 = a.x - a.w / 2, ay1 = a.y - a.h / 2;
  float ax2 = a.x + a.w / 2, ay2 = a.y + a.h / 2;
  float bx1 = b.x - b.w / 2, by1 = b.y - b.h / 2;
  float bx2 = b.x + b.w / 2, by2 = b.y + b.h / 2;

  float ix1 = std::max(ax1, bx1), iy1 = std::max(ay1, by1);
  float ix2 = std::min(ax2, bx2), iy2 = std::min(ay2, by2);
  float inter = std::max(0.0f, ix2 - ix1) * std::max(0.0f, iy2 - iy1);
  float area_a = a.w * a.h, area_b = b.w * b.h;
  float union_area = area_a + area_b - inter;
  return (union_area > 0) ? inter / union_area : 0.0f;
}

std::vector<Detection> TemporalTracker::update(
    const std::vector<Detection>& dets) {
  for (auto& t : tracks_) t.misses++;

  std::vector<bool> matched(dets.size(), false);
  for (auto& track : tracks_) {
    float best_iou = 0.0f;
    int best_idx = -1;
    for (size_t i = 0; i < dets.size(); ++i) {
      if (matched[i]) continue;
      float iou = computeIoU(track.last_det, dets[i]);
      if (iou > best_iou) { best_iou = iou; best_idx = static_cast<int>(i); }
    }
    if (best_idx >= 0 && best_iou >= config_.iou_threshold) {
      track.last_det = dets[best_idx];
      track.hits++;
      track.misses = 0;
      matched[best_idx] = true;
    }
  }

  for (size_t i = 0; i < dets.size(); ++i) {
    if (!matched[i]) {
      tracks_.push_back({dets[i], 1, 0});
    }
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const Track& t) {
        return t.misses > config_.max_miss_frames;
      }), tracks_.end());

  std::vector<Detection> out;
  for (const auto& t : tracks_) {
    if (t.hits >= config_.min_hits_to_output) {
      out.push_back(t.last_det);
    }
  }
  return out;
}

}  // namespace vision_core
