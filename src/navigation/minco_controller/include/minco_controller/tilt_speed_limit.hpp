#pragma once

#include <algorithm>
#include <cmath>

namespace minco_controller::tilt_speed_limit {

inline double speedLimit(double roll, double pitch, double slowdown_start_angle,
                         double full_slowdown_angle, double flat_speed_limit,
                         double slope_speed_limit) noexcept {
  if (!std::isfinite(roll) || !std::isfinite(pitch) ||
      !std::isfinite(slowdown_start_angle) ||
      !std::isfinite(full_slowdown_angle) || !std::isfinite(flat_speed_limit) ||
      !std::isfinite(slope_speed_limit) || slowdown_start_angle < 0.0 ||
      full_slowdown_angle <= slowdown_start_angle || flat_speed_limit <= 0.0 ||
      slope_speed_limit <= 0.0) {
    return 0.0;
  }

  const double tilt = std::hypot(roll, pitch);
  const double low_limit = std::min(flat_speed_limit, slope_speed_limit);
  if (tilt <= slowdown_start_angle) {
    return flat_speed_limit;
  }
  if (tilt >= full_slowdown_angle) {
    return low_limit;
  }

  const double ratio = (tilt - slowdown_start_angle) /
                       (full_slowdown_angle - slowdown_start_angle);
  return flat_speed_limit + ratio * (low_limit - flat_speed_limit);
}

inline bool apply(double limit, double &vx, double &vy) noexcept {
  if (!std::isfinite(limit) || limit <= 0.0 || !std::isfinite(vx) ||
      !std::isfinite(vy)) {
    vx = 0.0;
    vy = 0.0;
    return false;
  }

  const double speed = std::hypot(vx, vy);
  if (!std::isfinite(speed)) {
    vx = 0.0;
    vy = 0.0;
    return false;
  }
  if (speed <= limit || speed <= 1.0e-12) {
    return true;
  }

  const double scale = limit / speed;
  vx *= scale;
  vy *= scale;
  return true;
}

inline bool
shouldApplyUphillCommandFloor(bool stationary_startup_active,
                              double peak_reference_speed,
                              double minimum_reference_speed) noexcept {
  if (!std::isfinite(peak_reference_speed) || peak_reference_speed < 0.0 ||
      !std::isfinite(minimum_reference_speed) ||
      minimum_reference_speed <= 0.0) {
    return false;
  }
  return stationary_startup_active ||
         peak_reference_speed + 1.0e-9 >= minimum_reference_speed;
}

inline bool applyUphillCommandFloor(bool assist_active,
                                    double assist_start_grade,
                                    double full_assist_grade,
                                    double maximum_command_speed, double roll,
                                    double pitch, double yaw, double &vx_global,
                                    double &vy_global, double &uphill_grade,
                                    double &command_floor) noexcept {
  uphill_grade = 0.0;
  command_floor = 0.0;
  if (!std::isfinite(assist_start_grade) || assist_start_grade < 0.0 ||
      assist_start_grade >= 1.0 || !std::isfinite(full_assist_grade) ||
      full_assist_grade <= assist_start_grade || full_assist_grade > 1.0 ||
      !std::isfinite(maximum_command_speed) || maximum_command_speed < 0.0 ||
      !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw) ||
      !std::isfinite(vx_global) || !std::isfinite(vy_global)) {
    return false;
  }
  if (!assist_active || maximum_command_speed <= 0.0) {
    return true;
  }

  const double speed = std::hypot(vx_global, vy_global);
  if (!std::isfinite(speed)) {
    return false;
  }
  command_floor = speed;
  if (speed <= 1.0e-9) {
    return true;
  }

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double vx_body = cos_yaw * vx_global + sin_yaw * vy_global;
  const double vy_body = -sin_yaw * vx_global + cos_yaw * vy_global;

  // The third row of the body-to-world RPY rotation gives the vertical
  // component of a planar body-frame command. Positive means uphill.
  uphill_grade = (-std::sin(pitch) * vx_body +
                  std::cos(pitch) * std::sin(roll) * vy_body) /
                 speed;
  uphill_grade = std::clamp(uphill_grade, -1.0, 1.0);
  if (uphill_grade <= assist_start_grade ||
      speed + 1.0e-9 >= maximum_command_speed) {
    return true;
  }

  // Blend from the MPC command to full uphill assistance with zero slope at
  // both ends. This avoids the command step caused by a fixed speed floor when
  // attitude noise crosses the ramp threshold.
  const double ratio = std::clamp((uphill_grade - assist_start_grade) /
                                      (full_assist_grade - assist_start_grade),
                                  0.0, 1.0);
  const double blend = ratio * ratio * (3.0 - 2.0 * ratio);
  command_floor = speed + blend * (maximum_command_speed - speed);
  const double scale = command_floor / speed;
  vx_global *= scale;
  vy_global *= scale;
  return true;
}

} // namespace minco_controller::tilt_speed_limit
