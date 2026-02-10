#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

namespace localization_ros {

struct LocPerfSample {
  double t_opt_ms = 0.0;        // iSAM2 optimization time
  double t_preint_ms = 0.0;     // IMU preintegration accumulation time
  double t_assoc_ms = 0.0;      // data association time
  double t_prune_ms = 0.0;      // landmark pruning time
  double t_total_ms = 0.0;      // total keyframe processing time
  double landmark_count = 0.0;
  double chi2_normalized = 0.0;
  double cone_match_ratio = 0.0;
  double gnss_availability = 0.0;  // 1.0 if GNSS used this frame, 0.0 otherwise
};

class LocPerfStats {
 public:
  struct MetricStats {
    double mean = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
    double max = 0.0;
  };

  struct Snapshot {
    MetricStats t_opt_ms;
    MetricStats t_preint_ms;
    MetricStats t_assoc_ms;
    MetricStats t_prune_ms;
    MetricStats t_total_ms;
    MetricStats landmark_count;
    MetricStats chi2_normalized;
    MetricStats cone_match_ratio;
    MetricStats gnss_availability;
  };

  LocPerfStats() = default;

  void Configure(const std::string &name, bool enabled, size_t window_size, size_t log_every) {
    name_ = name;
    enabled_ = enabled;
    window_size_ = window_size == 0 ? 1 : window_size;
    log_every_ = log_every == 0 ? 1 : log_every;
    samples_.assign(window_size_, LocPerfSample());
    count_ = 0;
    index_ = 0;
    since_log_ = 0;
  }

  void Add(const LocPerfSample &sample) {
    if (!enabled_) return;
    samples_[index_] = sample;
    index_ = (index_ + 1) % window_size_;
    count_++;
    since_log_++;
    if (since_log_ >= log_every_ && SampleCount() >= window_size_) {
      LogStats();
      since_log_ = 0;
    }
  }

  Snapshot SnapshotStats() const {
    Snapshot snap;
    snap.t_opt_ms = ComputeStatsFor([](const LocPerfSample &s) { return s.t_opt_ms; });
    snap.t_preint_ms = ComputeStatsFor([](const LocPerfSample &s) { return s.t_preint_ms; });
    snap.t_assoc_ms = ComputeStatsFor([](const LocPerfSample &s) { return s.t_assoc_ms; });
    snap.t_prune_ms = ComputeStatsFor([](const LocPerfSample &s) { return s.t_prune_ms; });
    snap.t_total_ms = ComputeStatsFor([](const LocPerfSample &s) { return s.t_total_ms; });
    snap.landmark_count = ComputeStatsFor([](const LocPerfSample &s) { return s.landmark_count; });
    snap.chi2_normalized = ComputeStatsFor([](const LocPerfSample &s) { return s.chi2_normalized; });
    snap.cone_match_ratio = ComputeStatsFor([](const LocPerfSample &s) { return s.cone_match_ratio; });
    snap.gnss_availability = ComputeStatsFor([](const LocPerfSample &s) { return s.gnss_availability; });
    return snap;
  }

 private:
  size_t SampleCount() const { return std::min(count_, window_size_); }

  size_t SampleIndex(size_t i) const {
    if (count_ < window_size_) return i;
    return (index_ + i) % window_size_;
  }

  static double Percentile(const std::vector<double> &sorted, double p) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted.front();
    double idx = p * (sorted.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(idx));
    size_t hi = static_cast<size_t>(std::ceil(idx));
    double frac = idx - static_cast<double>(lo);
    return sorted[lo] + (sorted[hi] - sorted[lo]) * frac;
  }

  static MetricStats ComputeStats(std::vector<double> values) {
    MetricStats stats;
    if (values.empty()) return stats;
    std::sort(values.begin(), values.end());
    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    stats.mean = sum / static_cast<double>(values.size());
    stats.p50 = Percentile(values, 0.50);
    stats.p95 = Percentile(values, 0.95);
    stats.p99 = Percentile(values, 0.99);
    stats.max = values.back();
    return stats;
  }

  template <typename Getter>
  MetricStats ComputeStatsFor(Getter getter) const {
    size_t n = SampleCount();
    std::vector<double> values;
    values.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      values.push_back(getter(samples_[SampleIndex(i)]));
    }
    return ComputeStats(values);
  }

  static void AppendMetric(std::ostringstream &os, const char *name, const MetricStats &stats) {
    os << name << "{mean=" << stats.mean << ",p50=" << stats.p50 << ",p95=" << stats.p95
       << ",p99=" << stats.p99 << ",max=" << stats.max << "} ";
  }

  void LogStats() const {
    std::ostringstream os;
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::setprecision(3);
    os << "[perf] node=" << name_ << " window=" << SampleCount() << " ";

    AppendMetric(os, "t_opt_ms", ComputeStatsFor([](const LocPerfSample &s) { return s.t_opt_ms; }));
    AppendMetric(os, "t_preint_ms", ComputeStatsFor([](const LocPerfSample &s) { return s.t_preint_ms; }));
    AppendMetric(os, "t_assoc_ms", ComputeStatsFor([](const LocPerfSample &s) { return s.t_assoc_ms; }));
    AppendMetric(os, "t_prune_ms", ComputeStatsFor([](const LocPerfSample &s) { return s.t_prune_ms; }));
    AppendMetric(os, "t_total_ms", ComputeStatsFor([](const LocPerfSample &s) { return s.t_total_ms; }));
    AppendMetric(os, "LM", ComputeStatsFor([](const LocPerfSample &s) { return s.landmark_count; }));
    AppendMetric(os, "chi2", ComputeStatsFor([](const LocPerfSample &s) { return s.chi2_normalized; }));
    AppendMetric(os, "match", ComputeStatsFor([](const LocPerfSample &s) { return s.cone_match_ratio; }));
    AppendMetric(os, "gnss", ComputeStatsFor([](const LocPerfSample &s) { return s.gnss_availability; }));

    ROS_INFO_STREAM(os.str());
  }

  std::string name_ = "localization";
  bool enabled_ = false;
  size_t window_size_ = 300;
  size_t log_every_ = 30;
  size_t count_ = 0;
  size_t index_ = 0;
  size_t since_log_ = 0;
  std::vector<LocPerfSample> samples_;
};

}  // namespace localization_ros
