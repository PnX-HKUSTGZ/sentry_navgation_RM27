#ifndef RM_27_STIMULATION__GROUND_TRUTH_STATE_BUFFER_HPP_
#define RM_27_STIMULATION__GROUND_TRUTH_STATE_BUFFER_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <utility>

#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace rm_27_stimulation
{

struct GroundTruthState
{
  int64_t stamp_nanoseconds{0};
  tf2::Transform odom_to_base{tf2::Transform::getIdentity()};
  geometry_msgs::msg::TwistWithCovariance base_twist;
  std::array<double, 36> pose_covariance{};
};

enum class GroundTruthLookupStatus
{
  READY,
  NEED_BRACKET,
  GAP_TOO_LARGE,
  INVALID_QUERY,
};

class GroundTruthStateBuffer {
public:
  explicit GroundTruthStateBuffer(double history_duration = 2.0)
  {
    setHistoryDuration(history_duration);
  }

  void setHistoryDuration(double history_duration)
  {
    if (!std::isfinite(history_duration) || history_duration <= 0.0) {
      history_nanoseconds_ = 0;
      states_.clear();
      return;
    }
    history_nanoseconds_ = static_cast<int64_t>(
      history_duration * static_cast<double>(kNanosecondsPerSecond));
    prune();
  }

  bool insert(GroundTruthState state)
  {
    if (history_nanoseconds_ <= 0 || !normalizeRotation(state.odom_to_base) ||
      !validState(state))
    {
      return false;
    }

    auto position = std::lower_bound(
        states_.begin(), states_.end(), state.stamp_nanoseconds,
      [](const GroundTruthState & candidate, int64_t stamp) {
        return candidate.stamp_nanoseconds < stamp;
        });
    if (position != states_.end() &&
      position->stamp_nanoseconds == state.stamp_nanoseconds)
    {
      *position = std::move(state);
    } else {
      states_.insert(position, std::move(state));
    }
    prune();
    return true;
  }

  GroundTruthLookupStatus interpolate(
    int64_t stamp_nanoseconds,
    double max_interpolation_gap,
    GroundTruthState & output) const
  {
    if (stamp_nanoseconds < 0 || !std::isfinite(max_interpolation_gap) ||
      max_interpolation_gap <= 0.0)
    {
      return GroundTruthLookupStatus::INVALID_QUERY;
    }

    const auto upper =
      std::lower_bound(states_.begin(), states_.end(), stamp_nanoseconds,
        [](const GroundTruthState & candidate, int64_t stamp) {
          return candidate.stamp_nanoseconds < stamp;
                         });
    if (upper != states_.end() &&
      upper->stamp_nanoseconds == stamp_nanoseconds)
    {
      output = *upper;
      return GroundTruthLookupStatus::READY;
    }
    if (upper == states_.begin() || upper == states_.end()) {
      return GroundTruthLookupStatus::NEED_BRACKET;
    }

    const auto lower = std::prev(upper);
    const int64_t span_nanoseconds =
      upper->stamp_nanoseconds - lower->stamp_nanoseconds;
    const long double max_gap_nanoseconds =
      static_cast<long double>(max_interpolation_gap) * kNanosecondsPerSecond;
    if (span_nanoseconds <= 0 ||
      static_cast<long double>(span_nanoseconds) > max_gap_nanoseconds)
    {
      return GroundTruthLookupStatus::GAP_TOO_LARGE;
    }

    const double ratio =
      static_cast<double>(stamp_nanoseconds - lower->stamp_nanoseconds) /
      static_cast<double>(span_nanoseconds);
    if (!std::isfinite(ratio) || ratio < 0.0 || ratio > 1.0) {
      return GroundTruthLookupStatus::INVALID_QUERY;
    }

    output.stamp_nanoseconds = stamp_nanoseconds;
    const tf2::Vector3 position =
      lower->odom_to_base.getOrigin() +
      (upper->odom_to_base.getOrigin() - lower->odom_to_base.getOrigin()) *
      ratio;
    tf2::Quaternion lower_rotation = lower->odom_to_base.getRotation();
    tf2::Quaternion upper_rotation = upper->odom_to_base.getRotation();
    if (lower_rotation.dot(upper_rotation) < 0.0) {
      upper_rotation =
        tf2::Quaternion(-upper_rotation.x(), -upper_rotation.y(),
                          -upper_rotation.z(), -upper_rotation.w());
    }
    tf2::Quaternion rotation = lower_rotation.slerp(upper_rotation, ratio);
    if (!std::isfinite(rotation.length2()) ||
      rotation.length2() <= kQuaternionNormEpsilon)
    {
      return GroundTruthLookupStatus::INVALID_QUERY;
    }
    rotation.normalize();
    output.odom_to_base = tf2::Transform(rotation, position);

    interpolateTwist(lower->base_twist, upper->base_twist, ratio,
                     output.base_twist);
    for (std::size_t index = 0; index < output.pose_covariance.size();
      ++index)
    {
      output.pose_covariance[index] = interpolateScalar(
          lower->pose_covariance[index], upper->pose_covariance[index], ratio);
    }
    return validState(output) ? GroundTruthLookupStatus::READY :
           GroundTruthLookupStatus::INVALID_QUERY;
  }

  std::size_t size() const {return states_.size();}

private:
  static constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
  static constexpr double kQuaternionNormEpsilon = 1.0e-12;

  static double interpolateScalar(double lower, double upper, double ratio)
  {
    return lower + (upper - lower) * ratio;
  }

  static bool finiteVector(const geometry_msgs::msg::Vector3 & vector)
  {
    return std::isfinite(vector.x) && std::isfinite(vector.y) &&
           std::isfinite(vector.z);
  }

  static bool normalizeRotation(tf2::Transform & transform)
  {
    tf2::Quaternion rotation = transform.getRotation();
    if (!std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) ||
      !std::isfinite(rotation.z()) || !std::isfinite(rotation.w()) ||
      !std::isfinite(rotation.length2()) ||
      rotation.length2() <= kQuaternionNormEpsilon)
    {
      return false;
    }
    rotation.normalize();
    transform.setRotation(rotation);
    return true;
  }

  static bool validState(const GroundTruthState & state)
  {
    const auto & origin = state.odom_to_base.getOrigin();
    const auto & rotation = state.odom_to_base.getRotation();
    if (state.stamp_nanoseconds < 0 || !std::isfinite(origin.x()) ||
      !std::isfinite(origin.y()) || !std::isfinite(origin.z()) ||
      !std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) ||
      !std::isfinite(rotation.z()) || !std::isfinite(rotation.w()) ||
      !std::isfinite(rotation.length2()) ||
      rotation.length2() <= kQuaternionNormEpsilon ||
      !finiteVector(state.base_twist.twist.linear) ||
      !finiteVector(state.base_twist.twist.angular))
    {
      return false;
    }
    return std::all_of(state.pose_covariance.begin(),
                       state.pose_covariance.end(),
             [](double value) {return std::isfinite(value);}) &&
           std::all_of(state.base_twist.covariance.begin(),
                       state.base_twist.covariance.end(),
             [](double value) {return std::isfinite(value);});
  }

  static void
  interpolateTwist(
    const geometry_msgs::msg::TwistWithCovariance & lower,
    const geometry_msgs::msg::TwistWithCovariance & upper,
    double ratio,
    geometry_msgs::msg::TwistWithCovariance & output)
  {
    output.twist.linear.x =
      interpolateScalar(lower.twist.linear.x, upper.twist.linear.x, ratio);
    output.twist.linear.y =
      interpolateScalar(lower.twist.linear.y, upper.twist.linear.y, ratio);
    output.twist.linear.z =
      interpolateScalar(lower.twist.linear.z, upper.twist.linear.z, ratio);
    output.twist.angular.x =
      interpolateScalar(lower.twist.angular.x, upper.twist.angular.x, ratio);
    output.twist.angular.y =
      interpolateScalar(lower.twist.angular.y, upper.twist.angular.y, ratio);
    output.twist.angular.z =
      interpolateScalar(lower.twist.angular.z, upper.twist.angular.z, ratio);
    for (std::size_t index = 0; index < output.covariance.size(); ++index) {
      output.covariance[index] = interpolateScalar(
          lower.covariance[index], upper.covariance[index], ratio);
    }
  }

  void prune()
  {
    if (states_.empty() || history_nanoseconds_ <= 0) {
      return;
    }
    const int64_t oldest_allowed =
      states_.back().stamp_nanoseconds - history_nanoseconds_;
    while (!states_.empty() &&
      states_.front().stamp_nanoseconds < oldest_allowed)
    {
      states_.pop_front();
    }
  }

  int64_t history_nanoseconds_{2000000000LL};
  std::deque<GroundTruthState> states_;
};

inline bool
scanMatchExpired(
  const std::chrono::steady_clock::time_point & received,
  const std::chrono::steady_clock::time_point & now,
  double timeout)
{
  if (!std::isfinite(timeout) || timeout <= 0.0 || now < received) {
    return true;
  }
  return std::chrono::duration<double>(now - received).count() >= timeout;
}

} // namespace rm_27_stimulation

#endif // RM_27_STIMULATION__GROUND_TRUTH_STATE_BUFFER_HPP_
