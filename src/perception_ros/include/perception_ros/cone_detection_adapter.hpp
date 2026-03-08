#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <autodrive_msgs/HUAT_ConeDetections.h>
#include <autodrive_msgs/HUAT_FusedConeDetections.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace perception_ros {

class ConeDetectionAdapter {
 public:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTimePoint = SteadyClock::time_point;

  struct Config {
    ros::Duration raw_holdoff{0.05};
    ros::Duration finalized_ttl{2.0};
    std::size_t max_pending = 100;
  };

  struct Stats {
    std::size_t published_fused = 0;
    std::size_t published_raw_fallback = 0;
    std::size_t no_fused = 0;
    std::size_t late_fused = 0;
    std::size_t stale_fused = 0;
    std::size_t count_mismatch = 0;
    std::size_t cache_evicted = 0;
    std::size_t duplicate_after_finalize = 0;
    std::size_t pending_cache_size_max = 0;
  };

  struct PublishOutput {
    autodrive_msgs::HUAT_ConeDetections msg;
    bool used_fused = false;
    std::string reason;
    double wait_ms = 0.0;
  };

  explicit ConeDetectionAdapter(const Config &config);

  std::vector<PublishOutput> HandleRaw(const autodrive_msgs::HUAT_ConeDetections &msg,
                                       const SteadyTimePoint &received_at);
  std::vector<PublishOutput> HandleFused(const autodrive_msgs::HUAT_FusedConeDetections &msg,
                                         const SteadyTimePoint &received_at);
  std::vector<PublishOutput> Flush(const SteadyTimePoint &now);
  const Stats &stats() const;

 private:
  struct RawEntry {
    autodrive_msgs::HUAT_ConeDetections msg;
    SteadyTimePoint received_at;
    SteadyTimePoint deadline;
  };

  struct FusedEntry {
    autodrive_msgs::HUAT_FusedConeDetections msg;
    SteadyTimePoint received_at;
  };

  struct FinalizedEntry {
    SteadyTimePoint expires_at;
  };

  using StampKey = std::uint64_t;

  static StampKey StampToKey(const ros::Time &stamp);
  static SteadyClock::duration ToSteadyDuration(const ros::Duration &duration);
  PublishOutput BuildMergedOutput(const autodrive_msgs::HUAT_ConeDetections &raw,
                                  const autodrive_msgs::HUAT_FusedConeDetections &fused) const;
  PublishOutput BuildRawOutput(const autodrive_msgs::HUAT_ConeDetections &raw) const;
  bool CanMerge(const autodrive_msgs::HUAT_ConeDetections &raw,
                const autodrive_msgs::HUAT_FusedConeDetections &fused) const;
  void Finalize(StampKey key, const SteadyTimePoint &now);
  void NotePendingCacheSize();
  void PruneFinalized(const SteadyTimePoint &now);
  void TrimCaches();

  Config config_;
  Stats stats_;
  SteadyClock::duration raw_holdoff_;
  SteadyClock::duration finalized_ttl_;
  std::map<StampKey, RawEntry> pending_raw_;
  std::map<StampKey, FusedEntry> pending_fused_;
  std::map<StampKey, FinalizedEntry> finalized_;
};

}  // namespace perception_ros
