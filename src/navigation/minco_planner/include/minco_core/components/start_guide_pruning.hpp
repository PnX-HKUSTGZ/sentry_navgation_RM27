#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>
#include <Eigen/Core>

namespace minco_planner {

// A grid-scale sideways pickup can demand an instantaneous turn from a moving
// chassis. Remove it only in observed open space, with the entire replacement
// chord checked; keep doorway/corner guide points and the route endpoint.
inline void pruneOpenStartGuide(std::vector<Eigen::Vector3d> & path,
    const Eigen::Vector3d & start, const Eigen::Vector3d & velocity,
    double sample_step, const std::function<bool(const Eigen::Vector3d &)> & open_and_safe) {
  if (!start.allFinite() || !velocity.allFinite() ||
      velocity.head<2>().norm() <= 0.1 || !std::isfinite(sample_step) ||
      sample_step <= 0.0 || !open_and_safe) {return;}
  while (path.size() > 2U) {
    const Eigen::Vector2d pickup = (path[1] - start).head<2>();
    const Eigen::Vector2d forward = (path[2] - start).head<2>();
    if (!pickup.allFinite() || !forward.allFinite() || pickup.norm() >= 0.20 ||
        forward.norm() <= pickup.norm() || velocity.head<2>().dot(forward) <= 0.0) {break;}
    const int samples = std::max(1, static_cast<int>(std::ceil(forward.norm() / sample_step)));
    bool safe = true;
    for (int sample = 0; sample <= samples; ++sample) {
      if (!open_and_safe(start + (path[2] - start) * (static_cast<double>(sample) / samples))) {
        safe = false;
        break;
      }
    }
    if (!safe) {break;}
    path.erase(path.begin() + 1);
  }
}
}  // namespace minco_planner
