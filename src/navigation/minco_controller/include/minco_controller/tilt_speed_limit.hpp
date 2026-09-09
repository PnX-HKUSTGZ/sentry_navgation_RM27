#pragma once

#include <algorithm>
#include <cmath>

namespace minco_controller::tilt_speed_limit
{

inline double speedLimit(
    double roll, double pitch, double slowdown_start_angle,
    double full_slowdown_angle, double flat_speed_limit,
    double slope_speed_limit) noexcept
{
  if (!std::isfinite(roll) || !std::isfinite(pitch) ||
      !std::isfinite(slowdown_start_angle) ||
      !std::isfinite(full_slowdown_angle) ||
      !std::isfinite(flat_speed_limit) ||
      !std::isfinite(slope_speed_limit) || slowdown_start_angle < 0.0 ||
      full_slowdown_angle <= slowdown_start_angle || flat_speed_limit <= 0.0 ||
      slope_speed_limit <= 0.0)
  {
    return 0.0;
  }

  const double tilt = std::hypot(roll, pitch);
  const double low_limit = std::min(flat_speed_limit, slope_speed_limit);
  if (tilt <= slowdown_start_angle)
  {
    return flat_speed_limit;
  }
  if (tilt >= full_slowdown_angle)
  {
    return low_limit;
  }

  const double ratio = (tilt - slowdown_start_angle) /
      (full_slowdown_angle - slowdown_start_angle);
  return flat_speed_limit + ratio * (low_limit - flat_speed_limit);
}

inline bool apply(double limit, double &vx, double &vy) noexcept
{
  if (!std::isfinite(limit) || limit <= 0.0 ||
      !std::isfinite(vx) || !std::isfinite(vy))
  {
    vx = 0.0;
    vy = 0.0;
    return false;
  }

  const double speed = std::hypot(vx, vy);
  if (!std::isfinite(speed))
  {
    vx = 0.0;
    vy = 0.0;
    return false;
  }
  if (speed <= limit || speed <= 1.0e-12)
  {
    return true;
  }

  const double scale = limit / speed;
  vx *= scale;
  vy *= scale;
  return true;
}

}  // namespace minco_controller::tilt_speed_limit
