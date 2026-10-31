#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace relocalization_manager {
// Only blend candidates that have already passed the independent GICP gates.
inline Eigen::Isometry3d blendVerifiedCorrection(
  const Eigen::Isometry3d & current, const Eigen::Isometry3d & verified, double alpha)
{
  if (!std::isfinite(alpha) || alpha <= 0.0 || alpha > 1.0 ||
      !current.matrix().allFinite() || !verified.matrix().allFinite()) {
    throw std::invalid_argument("Invalid verified correction interpolation");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = current.translation() + alpha * (verified.translation() - current.translation());
  result.linear() = Eigen::Quaterniond(current.linear()).slerp(
    alpha, Eigen::Quaterniond(verified.linear())).normalized().toRotationMatrix();
  return result;
}
}  // namespace relocalization_manager
