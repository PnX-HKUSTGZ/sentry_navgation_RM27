// Copyright 2026 RM27 Contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef RELOCALIZATION_MANAGER__STATIONARY_GATE_HPP_
#define RELOCALIZATION_MANAGER__STATIONARY_GATE_HPP_

#include <cmath>
#include <limits>

namespace relocalization_manager
{
// ROS-time gate. A missing command is not evidence of a stop; a received zero
// command remains usable only while fresh measured odometry confirms the stop.
class StationaryGate
{
public:
  double linear_threshold{0.05};
  double angular_threshold{0.08};
  double settle_time{0.75};
  double odometry_timeout{0.5};

  bool validParameters() const
  {
    return std::isfinite(linear_threshold) && linear_threshold >= 0.0 &&
           std::isfinite(angular_threshold) && angular_threshold >= 0.0 &&
           std::isfinite(settle_time) && settle_time > 0.0 && std::isfinite(odometry_timeout) &&
           odometry_timeout > 0.0;
  }

  void command(double now, double linear, double angular)
  {
    observeClock(now);
    command_busy_ = moving(linear, angular);
    if (command_busy_) {
      quiet_since_ = now;
    }
  }

  void odometry(double now, double stamp, double linear, double angular, bool valid_frame)
  {
    observeClock(now);
    const bool fresh = valid_frame && std::isfinite(stamp) && stamp > odometry_stamp_ &&
                       stamp <= now && now - stamp <= odometry_timeout;
    if (
      !fresh || moving(linear, angular) || command_busy_ ||
      now - odometry_stamp_ > odometry_timeout) {
      quiet_since_ = now;
    }
    odometry_valid_ = fresh && !moving(linear, angular);
    if (fresh) {
      odometry_stamp_ = stamp;
    }
  }

  bool stationary(double now)
  {
    observeClock(now);
    if (!odometry_valid_ || command_busy_ || now - odometry_stamp_ > odometry_timeout) {
      quiet_since_ = now;
      return false;
    }
    return now >= quiet_since_ + settle_time;
  }

  double settledSince() const { return quiet_since_ + settle_time; }

private:
  bool moving(double linear, double angular) const
  {
    return !std::isfinite(linear) || !std::isfinite(angular) ||
           std::abs(linear) > linear_threshold || std::abs(angular) > angular_threshold;
  }
  void observeClock(double now)
  {
    if (now < last_time_) {
      command_busy_ = true;
      odometry_valid_ = false;
      odometry_stamp_ = -std::numeric_limits<double>::infinity();
      quiet_since_ = now;
    }
    last_time_ = now;
  }
  bool command_busy_{true};
  bool odometry_valid_{false};
  double odometry_stamp_{-std::numeric_limits<double>::infinity()};
  double quiet_since_{0.0};
  double last_time_{0.0};
};
}  // namespace relocalization_manager
#endif  // RELOCALIZATION_MANAGER__STATIONARY_GATE_HPP_
