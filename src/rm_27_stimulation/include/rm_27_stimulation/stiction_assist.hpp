#pragma once

#include <algorithm>
#include <cmath>

#include "rm_27_stimulation/planar_velocity_pi.hpp"

namespace rm_27_stimulation {

struct StictionAssistConfig {
  double command_threshold{0.0};
  double engage_velocity{0.0};
  double release_velocity{0.0};
  double delay{0.0};
  double ramp_rate{0.0};
  double max_force{0.0};
};

class StictionAssist {
public:
  PlanarForce Update(double _targetX, double _targetY, double _currentX,
                     double _currentY, double _dt,
                     const StictionAssistConfig &_config) {
    if (!Valid(_targetX, _targetY, _currentX, _currentY, _dt, _config)) {
      this->Reset();
      return {};
    }

    const auto targetMagnitude = std::hypot(_targetX, _targetY);
    if (_config.max_force <= 0.0 ||
        targetMagnitude < _config.command_threshold) {
      this->Reset();
      return {};
    }

    const double directionX = _targetX / targetMagnitude;
    const double directionY = _targetY / targetMagnitude;
    if (this->hasDirection_ &&
        directionX * this->directionX_ + directionY * this->directionY_ < 0.5) {
      this->Reset();
    }
    this->directionX_ = directionX;
    this->directionY_ = directionY;
    this->hasDirection_ = true;

    const double forwardVelocity =
        _currentX * directionX + _currentY * directionY;
    if (forwardVelocity <= _config.engage_velocity) {
      const double previousActiveTime =
          std::max(0.0, this->stallTime_ - _config.delay);
      this->stallTime_ += _dt;
      const double activeTime = std::max(0.0, this->stallTime_ - _config.delay);
      this->force_ = std::min(
          _config.max_force,
          this->force_ + _config.ramp_rate * (activeTime - previousActiveTime));
    } else if (forwardVelocity >= _config.release_velocity) {
      this->stallTime_ = 0.0;
      this->force_ = 0.0;
    }

    return {this->force_ * directionX, this->force_ * directionY};
  }

  void Reset() {
    this->stallTime_ = 0.0;
    this->force_ = 0.0;
    this->directionX_ = 0.0;
    this->directionY_ = 0.0;
    this->hasDirection_ = false;
  }

  double Force() const { return this->force_; }

private:
  static bool Valid(double _targetX, double _targetY, double _currentX,
                    double _currentY, double _dt,
                    const StictionAssistConfig &_config) {
    return std::isfinite(_targetX) && std::isfinite(_targetY) &&
           std::isfinite(_currentX) && std::isfinite(_currentY) &&
           std::isfinite(_dt) && _dt > 0.0 &&
           std::isfinite(_config.command_threshold) &&
           std::isfinite(_config.engage_velocity) &&
           std::isfinite(_config.release_velocity) &&
           std::isfinite(_config.delay) && std::isfinite(_config.ramp_rate) &&
           std::isfinite(_config.max_force) &&
           _config.command_threshold >= 0.0 && _config.engage_velocity >= 0.0 &&
           _config.release_velocity >= _config.engage_velocity &&
           _config.delay >= 0.0 && _config.ramp_rate >= 0.0 &&
           _config.max_force >= 0.0;
  }

  double stallTime_{0.0};
  double force_{0.0};
  double directionX_{0.0};
  double directionY_{0.0};
  bool hasDirection_{false};
};

} // namespace rm_27_stimulation
