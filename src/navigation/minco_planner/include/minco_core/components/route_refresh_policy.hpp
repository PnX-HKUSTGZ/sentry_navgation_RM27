#pragma once

#include <cmath>
#include <cstdint>

namespace minco_planner {

// Only gates replacement of the global guide. Every local trajectory continues
// to require fresh collision checks, braking or BLOCK while the guide is held.
// Caller serializes access and supplies a consistent progress clock and route
// arc length. Simulation uses ROS time to match odometry and map evidence.
class RouteRefreshPolicy {
public:
  void reset(uint64_t session, double progress, double now) {
    session_ = session;
    progress_ = progress;
    last_progress_ = now;
  }

  void observe(uint64_t session, double progress, double now) {
    if (session != session_ || !std::isfinite(progress) || !std::isfinite(now)) {return;}
    // Returning to a previously reached point or moving sideways does not
    // restart the waiting period and cannot indefinitely prevent a detour.
    if (progress >= progress_ + 0.20) {
      progress_ = progress;
      last_progress_ = now;
    }
  }

  bool defer(uint64_t session, double progress, double now, double stall_time) {
    if (session != session_ || session == 0 || !std::isfinite(progress) ||
        !std::isfinite(now) || !std::isfinite(last_progress_) ||
        !std::isfinite(stall_time) || stall_time <= 0.0 || now < last_progress_) {
      return false;
    }
    observe(session, progress, now);
    return now - last_progress_ < stall_time;
  }

private:
  uint64_t session_{0};
  double progress_{0.0};
  double last_progress_{0.0};
};

}  // namespace minco_planner
