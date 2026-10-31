#pragma once

#include <cmath>

namespace point_lio {
// Point-LIO curvature stores acquisition time in milliseconds. A Gazebo GPU
// frame is a snapshot (the converter uses 1 ns to distinguish it from missing
// timestamps), so intermediate estimator updates must not deskew that frame.
template<typename Points>
bool isInstantaneousScan(const Points & points) {
  if (points.empty()) {return false;}
  const double stamp = points.front().curvature;
  if (!std::isfinite(stamp)) {return false;}
  for (const auto & point : points) {
    if (!std::isfinite(point.curvature) ||
        std::abs(static_cast<double>(point.curvature) - stamp) > 1.0e-6) {
      return false;
    }
  }
  return true;
}
}  // namespace point_lio
