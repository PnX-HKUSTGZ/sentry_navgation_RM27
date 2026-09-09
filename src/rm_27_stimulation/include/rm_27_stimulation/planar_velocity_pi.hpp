#pragma once

#include <algorithm>
#include <cmath>

namespace rm_27_stimulation
{

struct PlanarForce
{
  double x{0.0};
  double y{0.0};
};

class PlanarVelocityPi
{
public:
  PlanarForce Update(
      double _errorX, double _errorY, double _dt,
      double _proportionalGain, double _integralGain,
      double _maxIntegralForce, double _maxTotalForce)
  {
    if (!AllFinite(
          _errorX, _errorY, _dt, _proportionalGain, _integralGain,
          _maxIntegralForce, _maxTotalForce) ||
        _dt <= 0.0 || _proportionalGain < 0.0 || _integralGain < 0.0 ||
        _maxIntegralForce < 0.0 || _maxTotalForce < 0.0)
    {
      this->Reset();
      return {};
    }

    const PlanarForce proportional{
        _proportionalGain * _errorX,
        _proportionalGain * _errorY};
    if (!std::isfinite(proportional.x) || !std::isfinite(proportional.y))
    {
      this->Reset();
      return {};
    }

    if (_integralGain > 0.0 && _maxIntegralForce > 0.0)
    {
      auto candidate = PlanarForce{
          this->integral_.x + _integralGain * _errorX * _dt,
          this->integral_.y + _integralGain * _errorY * _dt};
      if (!std::isfinite(candidate.x) || !std::isfinite(candidate.y))
      {
        this->Reset();
        return {};
      }
      candidate = ClampMagnitude(candidate, _maxIntegralForce);

      const PlanarForce candidateForce{
          proportional.x + candidate.x,
          proportional.y + candidate.y};
      const PlanarForce integralStep{
          candidate.x - this->integral_.x,
          candidate.y - this->integral_.y};

      // Freeze only the component update that would push an already saturated
      // total force farther into saturation. Opposing error remains able to
      // unwind the integrator.
      const bool totalWouldSaturate =
          _maxTotalForce > 0.0 &&
          Magnitude(candidateForce) > _maxTotalForce;
      const bool pushesFurtherIntoSaturation =
          Dot(integralStep, candidateForce) > 0.0;
      if (!totalWouldSaturate || !pushesFurtherIntoSaturation)
      {
        this->integral_ = candidate;
      }
    }
    else
    {
      this->Reset();
    }

    return ClampMagnitude(
        {proportional.x + this->integral_.x,
         proportional.y + this->integral_.y},
        _maxTotalForce);
  }

  void Reset()
  {
    this->integral_ = {};
  }

  PlanarForce IntegralForce() const
  {
    return this->integral_;
  }

private:
  static bool AllFinite(
      double _a, double _b, double _c, double _d,
      double _e, double _f, double _g)
  {
    return std::isfinite(_a) && std::isfinite(_b) && std::isfinite(_c) &&
           std::isfinite(_d) && std::isfinite(_e) && std::isfinite(_f) &&
           std::isfinite(_g);
  }

  static double Magnitude(const PlanarForce &_value)
  {
    return std::hypot(_value.x, _value.y);
  }

  static double Dot(const PlanarForce &_lhs, const PlanarForce &_rhs)
  {
    return _lhs.x * _rhs.x + _lhs.y * _rhs.y;
  }

  static PlanarForce ClampMagnitude(PlanarForce _value, double _limit)
  {
    if (_limit <= 0.0)
    {
      return _value;
    }

    const auto magnitude = Magnitude(_value);
    if (magnitude > _limit)
    {
      const auto scale = _limit / magnitude;
      _value.x *= scale;
      _value.y *= scale;
    }
    return _value;
  }

  PlanarForce integral_;
};

}  // namespace rm_27_stimulation
