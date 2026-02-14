#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

namespace fsd_common {
namespace perf {

struct MetricStats {
  double mean = 0.0;
  double p50 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double max = 0.0;
};

template <typename SampleT>
class RollingStatsWindow {
 public:
  RollingStatsWindow() = default;

  void Configure(const std::string &name, bool enabled, size_t window_size, size_t log_every) {
    name_ = name;
    enabled_ = enabled;
    window_size_ = window_size == 0 ? 1 : window_size;
    log_every_ = log_every == 0 ? 1 : log_every;
    samples_.assign(window_size_, SampleT());
    count_ = 0;
    index_ = 0;
    since_log_ = 0;
  }

  bool PushSample(const SampleT &sample) {
    if (!enabled_) return false;
    samples_[index_] = sample;
    index_ = (index_ + 1) % window_size_;
    count_++;
    since_log_++;
    if (since_log_ >= log_every_ && SampleCount() >= window_size_) {
      since_log_ = 0;
      return true;
    }
    return false;
  }

  size_t SampleCount() const { return std::min(count_, window_size_); }

  const std::string &Name() const { return name_; }

  template <typename Getter>
  MetricStats ComputeStatsFor(Getter getter) const {
    const size_t n = SampleCount();
    std::vector<double> values;
    values.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      values.push_back(getter(samples_[SampleIndex(i)]));
    }
    return ComputeStats(values);
  }

 private:
  size_t SampleIndex(size_t i) const {
    if (count_ < window_size_) return i;
    return (index_ + i) % window_size_;
  }

  static double Percentile(const std::vector<double> &sorted, double p) {
    if (sorted.empty()) return 0.0;
    if (sorted.size() == 1) return sorted.front();
    const double idx = p * (sorted.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    const double frac = idx - static_cast<double>(lo);
    return sorted[lo] + (sorted[hi] - sorted[lo]) * frac;
  }

  static MetricStats ComputeStats(std::vector<double> values) {
    MetricStats stats;
    if (values.empty()) return stats;
    std::sort(values.begin(), values.end());
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    stats.mean = sum / static_cast<double>(values.size());
    stats.p50 = Percentile(values, 0.50);
    stats.p95 = Percentile(values, 0.95);
    stats.p99 = Percentile(values, 0.99);
    stats.max = values.back();
    return stats;
  }

  std::string name_ = "node";
  bool enabled_ = false;
  size_t window_size_ = 300;
  size_t log_every_ = 30;
  size_t count_ = 0;
  size_t index_ = 0;
  size_t since_log_ = 0;
  std::vector<SampleT> samples_;
};

}  // namespace perf
}  // namespace fsd_common
