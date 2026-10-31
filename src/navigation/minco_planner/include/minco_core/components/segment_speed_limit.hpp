#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace minco_planner {

// Shape only the segment containing the obstacle. Forward/backward acceleration
// propagation can then slow its neighbours by the necessary braking distance.
// Collision authorization remains the responsibility of the footprint checker.
template <typename DistanceQuery>
double segmentObstacleSpeedLimit(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    double maximum_speed, double slowdown_distance, double obstacle_speed,
    const DistanceQuery &distance_query) {
  if (slowdown_distance <= 0.0 || obstacle_speed >= maximum_speed) {
    return maximum_speed;
  }
  const int intervals = std::max(1, static_cast<int>(
    std::ceil((end - start).head<2>().norm() / 0.05)));
  for (int i = 0; i <= intervals; ++i) {
    const Eigen::Vector3d point = start + (end - start) *
      (static_cast<double>(i) / static_cast<double>(intervals));
    const double distance = distance_query(point);
    if (std::isfinite(distance) && distance < slowdown_distance) {
      return std::min(maximum_speed, obstacle_speed);
    }
  }
  return maximum_speed;
}

}  // namespace minco_planner
