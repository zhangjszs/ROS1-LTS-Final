#pragma once

#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <fsd_common/perf_stats_skeleton.hpp>

namespace fsd_common {
namespace perf {

/// Descriptor for a single metric extracted from a PerfSample.
template <typename SampleT>
struct MetricDescriptor {
  const char *name;
  std::function<double(const SampleT &)> getter;
};

/// Generic PerfStats base that eliminates per-package boilerplate.
///
/// Usage:
///   1. Define your PerfSample struct
///   2. Define a static MetricDescriptor array
///   3. Inherit: class MyPerfStats : public PerfStatsBase<PerfSample> { ... }
///      or just use PerfStatsBase<PerfSample> directly
///
/// Snapshot is a vector of {name, MetricStats} pairs.
template <typename SampleT>
class PerfStatsBase {
 public:
  using Descriptor = MetricDescriptor<SampleT>;
  using SnapshotEntry = std::pair<std::string, MetricStats>;
  using Snapshot = std::vector<SnapshotEntry>;

  PerfStatsBase() = default;

  /// Construct with metric descriptors.
  explicit PerfStatsBase(std::vector<Descriptor> descriptors)
      : descriptors_(std::move(descriptors)) {}

  void SetDescriptors(std::vector<Descriptor> descriptors) {
    descriptors_ = std::move(descriptors);
  }

  void Configure(const std::string &name, bool enabled, size_t window_size, size_t log_every) {
    window_.Configure(name, enabled, window_size, log_every);
  }

  void Add(const SampleT &sample) {
    if (window_.PushSample(sample)) {
      LogStats();
    }
  }

  Snapshot SnapshotStats() const {
    Snapshot snap;
    snap.reserve(descriptors_.size());
    for (const auto &d : descriptors_) {
      snap.emplace_back(d.name, window_.ComputeStatsFor(d.getter));
    }
    return snap;
  }

  /// Look up a single metric by name.
  MetricStats GetMetric(const char *name) const {
    for (const auto &d : descriptors_) {
      if (std::string(d.name) == name) {
        return window_.ComputeStatsFor(d.getter);
      }
    }
    return {};
  }

 private:
  static void AppendMetric(std::ostringstream &os, const char *name, const MetricStats &stats) {
    os << name << "{mean=" << stats.mean << ",p50=" << stats.p50 << ",p95=" << stats.p95
       << ",p99=" << stats.p99 << ",max=" << stats.max << "} ";
  }

  void LogStats() const {
    std::ostringstream os;
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::setprecision(3);
    os << "[perf] node=" << window_.Name() << " window=" << window_.SampleCount() << " ";
    for (const auto &d : descriptors_) {
      AppendMetric(os, d.name, window_.ComputeStatsFor(d.getter));
    }
    ROS_INFO_STREAM(os.str());
  }

  RollingStatsWindow<SampleT> window_;
  std::vector<Descriptor> descriptors_;
};

}  // namespace perf
}  // namespace fsd_common
