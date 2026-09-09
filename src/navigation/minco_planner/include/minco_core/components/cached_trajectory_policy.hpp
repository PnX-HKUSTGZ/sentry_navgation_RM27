#pragma once

#include <cmath>
#include <string_view>

namespace minco_planner::cached_trajectory_policy {

inline bool isCollisionRelated(std::string_view rejection_reason)
{
  return rejection_reason.find("COLLISION") != std::string_view::npos;
}

inline bool reuseDurationAllowed(
  std::string_view rejection_reason, double remaining_duration,
  double collision_reuse_max_duration)
{
  if (!std::isfinite(remaining_duration) || remaining_duration <= 0.0) {
    return false;
  }
  if (!isCollisionRelated(rejection_reason)) {
    return true;
  }
  return std::isfinite(collision_reuse_max_duration) &&
         collision_reuse_max_duration >= 0.0 &&
         remaining_duration <= collision_reuse_max_duration + 1.0e-9;
}

}  // namespace minco_planner::cached_trajectory_policy
