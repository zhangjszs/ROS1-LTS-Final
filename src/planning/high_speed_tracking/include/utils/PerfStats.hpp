#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

struct PerfSample {
  double t_pass_ms = 0.0;
  double t_ground_ms = 0.0;
  double t_cluster_ms = 0.0;
  double t_delaunay_ms = 0.0;
  double t_way_ms = 0.0;
  double t_total_ms = 0.0;
  double n_points = 0.0;
  double n_clusters = 0.0;
  double n_triangles = 0.0;
  double n_edges = 0.0;
  double n_detections = 0.0;
  double bytes_pub = 0.0;
};

class PerfStats {
 public:
  struct MetricStats {
    double mean = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
    double max = 0.0;
  };

  struct Snapshot {
    MetricStats t_pass_ms;
    MetricStats t_ground_ms;
    MetricStats t_cluster_ms;
    MetricStats t_delaunay_ms;
    MetricStats t_way_ms;
    MetricStats t_total_ms;
    MetricStats n_points;
    MetricStats n_clusters;
    MetricStats n_triangles;
    MetricStats n_edges;
    MetricStats n_detections;
    MetricStats bytes_pub;
  };

  PerfStats() = default;

  void Configure(const std::string &name, bool enabled, size_t window_size, size_t log_every) {
    name_ = name;
    enabled_ = enabled;
    window_size_ = window_size == 0 ? 1 : window_size;
    log_every_ = log_every == 0 ? 1 : log_every;
    samples_.assign(window_size_, PerfSample());
    count_ = 0;
    index_ = 0;
    since_log_ = 0;
  }

  void Add(const PerfSample &sample) {
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
    snap.t_pass_ms = ComputeStatsFor([](const PerfSample &s) { return s.t_pass_ms; });
    snap.t_ground_ms = ComputeStatsFor([](const PerfSample &s) { return s.t_ground_ms; });
    snap.t_cluster_ms = ComputeStatsFor([](const PerfSample &s) { return s.t_cluster_ms; });
    snap.t_delaunay_ms = ComputeStatsFor([](const PerfSample &s) { return s.t_delaunay_ms; });
    snap.t_way_ms = ComputeStatsFor([](const PerfSample &s) { return s.t_way_ms; });
    snap.t_total_ms = ComputeStatsFor([](const PerfSample &s) { return s.t_total_ms; });
    snap.n_points = ComputeStatsFor([](const PerfSample &s) { return s.n_points; });
    snap.n_clusters = ComputeStatsFor([](const PerfSample &s) { return s.n_clusters; });
    snap.n_triangles = ComputeStatsFor([](const PerfSample &s) { return s.n_triangles; });
    snap.n_edges = ComputeStatsFor([](const PerfSample &s) { return s.n_edges; });
    snap.n_detections = ComputeStatsFor([](const PerfSample &s) { return s.n_detections; });
    snap.bytes_pub = ComputeStatsFor([](const PerfSample &s) { return s.bytes_pub; });
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

    AppendMetric(os, "t_pass_ms", ComputeStatsFor([](const PerfSample &s) { return s.t_pass_ms; }));
    AppendMetric(os, "t_ground_ms", ComputeStatsFor([](const PerfSample &s) { return s.t_ground_ms; }));
    AppendMetric(os, "t_cluster_ms", ComputeStatsFor([](const PerfSample &s) { return s.t_cluster_ms; }));
    AppendMetric(os, "t_delaunay_ms", ComputeStatsFor([](const PerfSample &s) { return s.t_delaunay_ms; }));
    AppendMetric(os, "t_way_ms", ComputeStatsFor([](const PerfSample &s) { return s.t_way_ms; }));
    AppendMetric(os, "t_total_ms", ComputeStatsFor([](const PerfSample &s) { return s.t_total_ms; }));
    AppendMetric(os, "N", ComputeStatsFor([](const PerfSample &s) { return s.n_points; }));
    AppendMetric(os, "K", ComputeStatsFor([](const PerfSample &s) { return s.n_clusters; }));
    AppendMetric(os, "T", ComputeStatsFor([](const PerfSample &s) { return s.n_triangles; }));
    AppendMetric(os, "E", ComputeStatsFor([](const PerfSample &s) { return s.n_edges; }));
    AppendMetric(os, "D", ComputeStatsFor([](const PerfSample &s) { return s.n_detections; }));
    AppendMetric(os, "bytes", ComputeStatsFor([](const PerfSample &s) { return s.bytes_pub; }));

    ROS_INFO_STREAM(os.str());
  }

  std::string name_ = "node";
  bool enabled_ = false;
  size_t window_size_ = 300;
  size_t log_every_ = 30;
  size_t count_ = 0;
  size_t index_ = 0;
  size_t since_log_ = 0;
  std::vector<PerfSample> samples_;
};
