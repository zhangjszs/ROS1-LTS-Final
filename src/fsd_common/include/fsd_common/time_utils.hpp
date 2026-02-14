#pragma once
#include <chrono>
#include <functional>
#include <string>

namespace fsd_common {

class ScopedTimer {
 public:
  using Clock = std::chrono::steady_clock;
  explicit ScopedTimer(std::function<void(double)> on_done)
      : on_done_(std::move(on_done)), start_(Clock::now()) {}
  ~ScopedTimer() {
    if (on_done_) {
      double ms = std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
      on_done_(ms);
    }
  }
  double ElapsedMs() const {
    return std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
  }
  ScopedTimer(const ScopedTimer &) = delete;
  ScopedTimer &operator=(const ScopedTimer &) = delete;
 private:
  std::function<void(double)> on_done_;
  Clock::time_point start_;
};

}  // namespace fsd_common
