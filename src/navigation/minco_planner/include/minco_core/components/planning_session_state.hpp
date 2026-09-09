#ifndef MINCO_PLANNER__PLANNING_SESSION_STATE_HPP_
#define MINCO_PLANNER__PLANNING_SESSION_STATE_HPP_

#include <cstdint>

namespace minco_planner {

// This state is intentionally small and non-thread-safe. MincoPlanner protects
// it with mutex_ so checking a token and publishing a trajectory are atomic
// with respect to goal replacement and lifecycle transitions.
class PlanningSessionState {
public:
  uint64_t activate() {
    active_ = true;
    return ++generation_;
  }

  uint64_t beginSession() { return ++generation_; }

  uint64_t deactivate() {
    active_ = false;
    return ++generation_;
  }

  bool accepts(uint64_t generation) const {
    return active_ && generation != 0U && generation == generation_;
  }

  bool active() const { return active_; }
  uint64_t generation() const { return generation_; }

private:
  bool active_{false};
  uint64_t generation_{0};
};

} // namespace minco_planner

#endif // MINCO_PLANNER__PLANNING_SESSION_STATE_HPP_
