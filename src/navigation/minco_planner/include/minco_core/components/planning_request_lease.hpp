#ifndef MINCO_PLANNER__PLANNING_REQUEST_LEASE_HPP_
#define MINCO_PLANNER__PLANNING_REQUEST_LEASE_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace minco_planner {

// Non-thread-safe heartbeat state. MincoPlanner owns the synchronization and
// injects steady-clock time so ROS time pauses or jumps cannot extend a lease.
class PlanningRequestLease {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void configure(double timeout_seconds)
  {
    timeout_ = std::chrono::duration<double>(
      timeout_seconds > 0.0 ? timeout_seconds : 0.0);
    reset();
  }

  bool enabled() const { return timeout_ > std::chrono::duration<double>::zero(); }
  std::chrono::duration<double> timeout() const { return timeout_; }

  void reset()
  {
    session_ = 0U;
    last_refresh_ = TimePoint{};
    armed_ = false;
  }

  void refresh(uint64_t session, TimePoint now)
  {
    if (!enabled() || session == 0U) {
      reset();
      return;
    }
    session_ = session;
    last_refresh_ = now;
    armed_ = true;
  }

  std::optional<uint64_t> consumeExpired(TimePoint now)
  {
    if (!enabled() || !armed_ || now < last_refresh_ ||
      std::chrono::duration<double>(now - last_refresh_) < timeout_)
    {
      return std::nullopt;
    }

    const uint64_t expired_session = session_;
    reset();
    return expired_session;
  }

private:
  std::chrono::duration<double> timeout_{0.0};
  TimePoint last_refresh_{};
  uint64_t session_{0U};
  bool armed_{false};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNING_REQUEST_LEASE_HPP_
