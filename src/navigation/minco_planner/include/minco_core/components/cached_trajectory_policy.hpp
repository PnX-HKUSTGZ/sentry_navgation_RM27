#pragma once

#include <cmath>
#include <limits>
#include <string_view>

namespace minco_planner::cached_trajectory_policy {

inline bool isCollisionRelated(std::string_view rejection_reason) {
  return rejection_reason.find("COLLISION") != std::string_view::npos;
}

inline bool isNumericalFailure(std::string_view rejection_reason) {
  // A failed MINCO solve is not evidence that the previously published
  // trajectory remains valid.  Reusing a long old trajectory here can make a
  // local optimization failure look like successful progress and repeatedly
  // send the robot along a stale path while the global planner never gets a
  // chance to choose a new homotopy.
  return rejection_reason.find("OPTIMIZER_FAILED") != std::string_view::npos;
}

inline bool reuseDurationAllowed(std::string_view rejection_reason,
                                 double remaining_duration,
                                 double collision_reuse_max_duration,
                                 double optimizer_failure_reuse_max_duration) {
  if (!std::isfinite(remaining_duration) || remaining_duration <= 0.0) {
    return false;
  }
  double reuse_limit = std::numeric_limits<double>::infinity();
  if (isCollisionRelated(rejection_reason)) {
    reuse_limit = collision_reuse_max_duration;
  } else if (isNumericalFailure(rejection_reason)) {
    reuse_limit = optimizer_failure_reuse_max_duration;
  }
  return reuse_limit == std::numeric_limits<double>::infinity() ||
         (std::isfinite(reuse_limit) && reuse_limit >= 0.0 &&
          remaining_duration <= reuse_limit + 1.0e-9);
}

} // namespace minco_planner::cached_trajectory_policy
