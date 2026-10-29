#ifndef MINCO_PLANNER__PLANNING_SESSION_STATE_HPP_
#define MINCO_PLANNER__PLANNING_SESSION_STATE_HPP_

#include <cstdint>
#include <cmath>
#include <optional>

namespace minco_planner {

// This state is intentionally small and non-thread-safe. MincoPlanner protects
// it with mutex_ so checking a token and publishing a trajectory are atomic
// with respect to goal replacement and lifecycle transitions.
class PlanningSessionState {
public:
  uint64_t activate() {
    active_ = true;
    cruise_heading_.reset();
    return ++generation_;
  }

  uint64_t beginSession() { cruise_heading_.reset(); return ++generation_; }

  uint64_t deactivate() {
    active_ = false;
    cruise_heading_.reset();
    return ++generation_;
  }

  bool accepts(uint64_t generation) const {
    return active_ && generation != 0U && generation == generation_;
  }

  bool active() const { return active_; }
  uint64_t generation() const { return generation_; }

  std::optional<double> cruiseHeading(uint64_t generation, double measured) {
    if (!accepts(generation) || !std::isfinite(measured)) {return std::nullopt;}
    if (!cruise_heading_) {cruise_heading_ = measured;}
    return cruise_heading_;
  }

private:
  bool active_{false};
  uint64_t generation_{0};
  std::optional<double> cruise_heading_;
};

} // namespace minco_planner

#endif // MINCO_PLANNER__PLANNING_SESSION_STATE_HPP_
