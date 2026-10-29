#pragma once

#include <algorithm>
#include <cmath>
#include "minco_controller/mpc_types.hpp"

namespace minco_controller {

// Bring a timestamped, world-frame velocity state to the control instant.
// This is measurement transport age, separate from actuator lookahead. Do not
// extend the freshness window or integrate backwards for small future skew.
inline bool predictOdometryToNow(State & state, double stamp, double now,
                                 double max_age, double future_tolerance) {
  const double age = now - stamp;
  if (!std::isfinite(stamp) || !std::isfinite(now) ||
      !std::isfinite(max_age) || max_age <= 0.0 ||
      !std::isfinite(future_tolerance) || future_tolerance < 0.0 ||
      !std::isfinite(age) || age > max_age || age < -future_tolerance ||
      !std::isfinite(state.x) || !std::isfinite(state.y) ||
      !std::isfinite(state.yaw) || !std::isfinite(state.vx) ||
      !std::isfinite(state.vy) || !std::isfinite(state.omega)) {
    return false;
  }
  const double dt = std::max(0.0, age);
  const double x = state.x + state.vx * dt;
  const double y = state.y + state.vy * dt;
  const double yaw = state.yaw + state.omega * dt;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {return false;}
  state.x = x;
  state.y = y;
  state.yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  return true;
}

}  // namespace minco_controller
