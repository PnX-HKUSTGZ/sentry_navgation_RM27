#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>
#include <utility>

namespace rog_map::detail
{

// Non-thread-safe bounded synchronizer. The owner supplies synchronization and
// callback receive times so the matching policy can be tested without ROS.
template<typename PoseT, typename PayloadT>
class TimestampedPoseMatcher
{
public:
  struct Config
  {
    double sync_tolerance{0.1};
    double pending_timeout{0.25};
    double pose_receive_timeout{0.25};
    std::size_t max_pose_count{32U};
    std::size_t max_pending_count{4U};
  };

  enum class CloudInsertStatus
  {
    STORED,
    INVALID,
    DUPLICATE_OR_OUT_OF_ORDER
  };

  struct CloudInsertResult
  {
    CloudInsertStatus status{CloudInsertStatus::INVALID};
    std::size_t evicted_count{0U};
  };

  struct Match
  {
    PayloadT payload;
    PoseT pose;
    double cloud_stamp{0.0};
    double pose_stamp{0.0};
    double pose_receive_time{0.0};
    double stamp_delta{0.0};
  };

  enum class TakeStatus
  {
    NONE,
    MATCHED,
    DROPPED_NO_ODOMETRY,
    DROPPED_ODOMETRY_TIMEOUT,
    DROPPED_SYNC_TOLERANCE
  };

  struct TakeResult
  {
    TakeStatus status{TakeStatus::NONE};
    std::optional<Match> match;
    double cloud_stamp{0.0};
    double closest_pose_stamp{0.0};
    double closest_stamp_delta{std::numeric_limits<double>::infinity()};
  };

  explicit TimestampedPoseMatcher(Config config)
  : config_(std::move(config))
  {
    config_.sync_tolerance = std::max(0.0, config_.sync_tolerance);
    config_.pending_timeout = std::max(0.0, config_.pending_timeout);
    config_.pose_receive_timeout = std::max(0.0, config_.pose_receive_timeout);
    config_.max_pose_count = std::max<std::size_t>(1U, config_.max_pose_count);
    config_.max_pending_count = std::max<std::size_t>(1U, config_.max_pending_count);
  }

  bool addPose(double stamp, double receive_time, PoseT pose)
  {
    if (!validTime(stamp) || !validTime(receive_time)) {
      return false;
    }
    if (!poses_.empty() && stamp + kStampEpsilon < poses_.back().stamp) {
      return false;
    }
    if (!poses_.empty() && std::abs(stamp - poses_.back().stamp) <= kStampEpsilon) {
      poses_.back() = TimedPose{stamp, receive_time, std::move(pose)};
      return true;
    }

    poses_.push_back(TimedPose{stamp, receive_time, std::move(pose)});
    while (poses_.size() > config_.max_pose_count) {
      poses_.pop_front();
    }
    return true;
  }

  CloudInsertResult addCloud(double stamp, double receive_time, PayloadT payload)
  {
    if (!validTime(stamp) || !validTime(receive_time)) {
      return {CloudInsertStatus::INVALID, 0U};
    }
    if (has_observed_cloud_ && stamp <= last_observed_cloud_stamp_ + kStampEpsilon) {
      return {CloudInsertStatus::DUPLICATE_OR_OUT_OF_ORDER, 0U};
    }

    last_observed_cloud_stamp_ = stamp;
    has_observed_cloud_ = true;
    pending_.push_back(TimedPayload{stamp, receive_time, std::move(payload)});
    std::size_t evicted_count = 0U;
    while (pending_.size() > config_.max_pending_count) {
      pending_.pop_front();
      ++evicted_count;
    }
    return {CloudInsertStatus::STORED, evicted_count};
  }

  TakeResult takeNext(double now)
  {
    if (pending_.empty()) {
      return {};
    }

    const TimedPayload & cloud = pending_.front();
    const bool pending_expired = !std::isfinite(now) ||
      now - cloud.receive_time > config_.pending_timeout + kStampEpsilon;
    if (poses_.empty()) {
      return pending_expired ? dropFront(TakeStatus::DROPPED_NO_ODOMETRY) : TakeResult{};
    }

    const TimedPose * closest_any = closestPose(cloud.stamp, false, cloud.receive_time);
    if (poses_.back().stamp + kStampEpsilon < cloud.stamp) {
      if (!pending_expired) {
        return {};
      }
    }

    // Judge odometry freshness at cloud receipt, as the original callback did.
    // Waiting for cross-topic reordering must not make an otherwise valid pose stale.
    const TimedPose * closest_fresh = closestPose(cloud.stamp, true, cloud.receive_time);
    if (!closest_fresh) {
      return dropFront(TakeStatus::DROPPED_ODOMETRY_TIMEOUT, closest_any);
    }

    const double delta = std::abs(cloud.stamp - closest_fresh->stamp);
    if (delta > config_.sync_tolerance + kStampEpsilon) {
      return dropFront(TakeStatus::DROPPED_SYNC_TOLERANCE, closest_fresh);
    }

    TakeResult result;
    result.status = TakeStatus::MATCHED;
    result.cloud_stamp = cloud.stamp;
    result.closest_pose_stamp = closest_fresh->stamp;
    result.closest_stamp_delta = delta;
    result.match.emplace(Match{
      std::move(pending_.front().payload), closest_fresh->pose, cloud.stamp,
      closest_fresh->stamp, closest_fresh->receive_time, delta});
    pending_.pop_front();
    return result;
  }

  std::size_t poseCount() const { return poses_.size(); }
  std::size_t pendingCount() const { return pending_.size(); }

  void clear()
  {
    poses_.clear();
    pending_.clear();
    last_observed_cloud_stamp_ = 0.0;
    has_observed_cloud_ = false;
  }

private:
  struct TimedPose
  {
    double stamp;
    double receive_time;
    PoseT pose;
  };

  struct TimedPayload
  {
    double stamp;
    double receive_time;
    PayloadT payload;
  };

  static constexpr double kStampEpsilon = 1.0e-9;

  static bool validTime(double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  const TimedPose * closestPose(double stamp, bool require_fresh, double reference_time) const
  {
    const TimedPose * closest = nullptr;
    double closest_delta = std::numeric_limits<double>::infinity();
    for (const TimedPose & candidate : poses_) {
      const double receive_age = reference_time - candidate.receive_time;
      if (require_fresh &&
        (!std::isfinite(receive_age) || receive_age > config_.pose_receive_timeout + kStampEpsilon))
      {
        continue;
      }
      const double delta = std::abs(stamp - candidate.stamp);
      if (delta + kStampEpsilon < closest_delta) {
        closest = &candidate;
        closest_delta = delta;
      }
    }
    return closest;
  }

  TakeResult dropFront(TakeStatus status, const TimedPose * closest_pose = nullptr)
  {
    TakeResult result;
    result.status = status;
    result.cloud_stamp = pending_.front().stamp;
    if (closest_pose) {
      result.closest_pose_stamp = closest_pose->stamp;
      result.closest_stamp_delta = std::abs(pending_.front().stamp - closest_pose->stamp);
    }
    pending_.pop_front();
    return result;
  }

  Config config_;
  std::deque<TimedPose> poses_;
  std::deque<TimedPayload> pending_;
  double last_observed_cloud_stamp_{0.0};
  bool has_observed_cloud_{false};
};

}  // namespace rog_map::detail
