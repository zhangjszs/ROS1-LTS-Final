#include "perception_ros/cone_detection_adapter.hpp"

#include <algorithm>

namespace perception_ros {

namespace {

double WaitMilliseconds(const ConeDetectionAdapter::SteadyTimePoint& start,
                        const ConeDetectionAdapter::SteadyTimePoint& end) {
  return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
}

}  // namespace

ConeDetectionAdapter::ConeDetectionAdapter(const Config& config)
    : config_(config),
      raw_holdoff_(ToSteadyDuration(config_.raw_holdoff)),
      finalized_ttl_(ToSteadyDuration(config_.finalized_ttl)) {}

auto ConeDetectionAdapter::StampToKey(const ros::Time& stamp) -> StampKey {
  return stamp.toNSec();
}

auto ConeDetectionAdapter::ToSteadyDuration(const ros::Duration& duration)
    -> SteadyClock::duration {
  const auto nanoseconds = std::max<int64_t>(0, duration.toNSec());
  return std::chrono::duration_cast<SteadyClock::duration>(std::chrono::nanoseconds(nanoseconds));
}

auto ConeDetectionAdapter::BuildRawOutput(const autodrive_msgs::HUAT_ConeDetections& raw) const
    -> PublishOutput {
  PublishOutput output;
  output.msg = raw;
  output.used_fused = false;
  return output;
}

auto ConeDetectionAdapter::BuildMergedOutput(
    const autodrive_msgs::HUAT_ConeDetections& raw,
    const autodrive_msgs::HUAT_FusedConeDetections& fused) const -> PublishOutput {
  if (raw.points.size() != fused.points.size() ||
      fused.fused_color_types.size() < raw.points.size()) {
    return BuildRawOutput(raw);
  }

  PublishOutput output;
  output.msg = raw;
  output.used_fused = true;
  output.msg.color_types.clear();
  output.msg.color_types.reserve(raw.points.size());

  for (std::size_t i = 0; i < raw.points.size(); ++i) {
    output.msg.color_types.push_back(fused.fused_color_types[i]);
  }

  return output;
}

bool ConeDetectionAdapter::CanMerge(const autodrive_msgs::HUAT_ConeDetections& raw,
                                    const autodrive_msgs::HUAT_FusedConeDetections& fused) const {
  return raw.points.size() == fused.points.size() &&
         fused.fused_color_types.size() >= raw.points.size();
}

const ConeDetectionAdapter::Stats& ConeDetectionAdapter::stats() const {
  return stats_;
}

void ConeDetectionAdapter::Finalize(StampKey key, const SteadyTimePoint& now) {
  finalized_[key] = FinalizedEntry{now + finalized_ttl_};
}

void ConeDetectionAdapter::NotePendingCacheSize() {
  stats_.pending_cache_size_max =
      std::max(stats_.pending_cache_size_max, pending_raw_.size() + pending_fused_.size());
}

void ConeDetectionAdapter::PruneFinalized(const SteadyTimePoint& now) {
  for (auto it = finalized_.begin(); it != finalized_.end();) {
    if (now >= it->second.expires_at) {
      it = finalized_.erase(it);
      continue;
    }
    ++it;
  }
}

std::vector<ConeDetectionAdapter::PublishOutput> ConeDetectionAdapter::HandleRaw(
    const autodrive_msgs::HUAT_ConeDetections& msg, const SteadyTimePoint& received_at) {
  PruneFinalized(received_at);
  const StampKey key = StampToKey(msg.header.stamp);

  if (finalized_.count(key) > 0) {
    ++stats_.duplicate_after_finalize;
    return {};
  }

  auto fused_it = pending_fused_.find(key);
  if (fused_it != pending_fused_.end()) {
    std::vector<PublishOutput> outputs;
    if (CanMerge(msg, fused_it->second.msg)) {
      outputs.push_back(BuildMergedOutput(msg, fused_it->second.msg));
      ++stats_.published_fused;
      outputs.back().reason = "normal_merge";
      outputs.back().wait_ms = 0.0;
    } else {
      outputs.push_back(BuildRawOutput(msg));
      ++stats_.published_raw_fallback;
      ++stats_.count_mismatch;
      outputs.back().reason = "count_mismatch";
      outputs.back().wait_ms = 0.0;
    }
    pending_fused_.erase(fused_it);
    Finalize(key, received_at);
    return outputs;
  }

  pending_raw_[key] = RawEntry{msg, received_at, received_at + raw_holdoff_};
  NotePendingCacheSize();
  TrimCaches();
  return {};
}

std::vector<ConeDetectionAdapter::PublishOutput> ConeDetectionAdapter::HandleFused(
    const autodrive_msgs::HUAT_FusedConeDetections& msg, const SteadyTimePoint& received_at) {
  PruneFinalized(received_at);
  const StampKey key = StampToKey(msg.header.stamp);

  if (finalized_.count(key) > 0) {
    ++stats_.duplicate_after_finalize;
    ++stats_.late_fused;
    return {};
  }

  auto raw_it = pending_raw_.find(key);
  if (raw_it != pending_raw_.end()) {
    std::vector<PublishOutput> outputs;
    if (received_at > raw_it->second.deadline) {
      outputs.push_back(BuildRawOutput(raw_it->second.msg));
      ++stats_.published_raw_fallback;
      ++stats_.no_fused;
      ++stats_.late_fused;
      outputs.back().reason = "late_fused";
      outputs.back().wait_ms = WaitMilliseconds(raw_it->second.received_at, received_at);
    } else if (CanMerge(raw_it->second.msg, msg)) {
      outputs.push_back(BuildMergedOutput(raw_it->second.msg, msg));
      ++stats_.published_fused;
      outputs.back().reason = "normal_merge";
      outputs.back().wait_ms = WaitMilliseconds(raw_it->second.received_at, received_at);
    } else {
      outputs.push_back(BuildRawOutput(raw_it->second.msg));
      ++stats_.published_raw_fallback;
      ++stats_.count_mismatch;
      outputs.back().reason = "count_mismatch";
      outputs.back().wait_ms = WaitMilliseconds(raw_it->second.received_at, received_at);
    }
    pending_raw_.erase(raw_it);
    Finalize(key, received_at);
    return outputs;
  }

  pending_fused_[key] = FusedEntry{msg, received_at};
  NotePendingCacheSize();
  TrimCaches();
  return {};
}

std::vector<ConeDetectionAdapter::PublishOutput> ConeDetectionAdapter::Flush(
    const SteadyTimePoint& now) {
  PruneFinalized(now);
  std::vector<PublishOutput> outputs;

  for (auto it = pending_raw_.begin(); it != pending_raw_.end();) {
    if (now >= it->second.deadline) {
      outputs.push_back(BuildRawOutput(it->second.msg));
      ++stats_.published_raw_fallback;
      ++stats_.no_fused;
      outputs.back().reason = "no_fused";
      outputs.back().wait_ms = WaitMilliseconds(it->second.received_at, now);
      Finalize(it->first, now);
      it = pending_raw_.erase(it);
      continue;
    }
    ++it;
  }

  for (auto it = pending_fused_.begin(); it != pending_fused_.end();) {
    if (now >= it->second.received_at && now - it->second.received_at >= raw_holdoff_) {
      ++stats_.stale_fused;
      it = pending_fused_.erase(it);
      continue;
    }
    ++it;
  }

  return outputs;
}

void ConeDetectionAdapter::TrimCaches() {
  while (pending_raw_.size() + pending_fused_.size() > config_.max_pending) {
    auto oldest_raw = pending_raw_.end();
    auto oldest_fused = pending_fused_.end();

    if (!pending_raw_.empty()) {
      oldest_raw = std::min_element(pending_raw_.begin(), pending_raw_.end(),
                                    [](const auto& lhs, const auto& rhs) {
                                      return lhs.second.received_at < rhs.second.received_at;
                                    });
    }
    if (!pending_fused_.empty()) {
      oldest_fused = std::min_element(pending_fused_.begin(), pending_fused_.end(),
                                      [](const auto& lhs, const auto& rhs) {
                                        return lhs.second.received_at < rhs.second.received_at;
                                      });
    }

    const bool evict_raw = oldest_fused == pending_fused_.end() ||
                           (oldest_raw != pending_raw_.end() &&
                            oldest_raw->second.received_at <= oldest_fused->second.received_at);
    if (evict_raw) {
      pending_raw_.erase(oldest_raw);
    } else {
      pending_fused_.erase(oldest_fused);
    }
    ++stats_.cache_evicted;
  }
}

}  // namespace perception_ros
