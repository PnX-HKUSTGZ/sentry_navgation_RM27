#include "minco_core/components/trajectory_safety_checker.hpp"

#include "data_structure/base/trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minco_planner {

const char *TrajectorySafetyChecker::failureReasonName(FailureReason reason) {
  switch (reason) {
  case FailureReason::NONE:
    return "NONE";
  case FailureReason::QUERY_UNAVAILABLE:
    return "QUERY_UNAVAILABLE";
  case FailureReason::INVALID_TRAJECTORY_DURATION:
    return "INVALID_TRAJECTORY_DURATION";
  case FailureReason::INVALID_TRAJECTORY_START_TIME:
    return "INVALID_TRAJECTORY_START_TIME";
  case FailureReason::INVALID_YAW_TRAJECTORY:
    return "INVALID_YAW_TRAJECTORY";
  case FailureReason::NONFINITE_CENTER:
    return "NONFINITE_CENTER";
  case FailureReason::NONFINITE_YAW:
    return "NONFINITE_YAW";
  case FailureReason::INVALID_MAP_RESOLUTION:
    return "INVALID_MAP_RESOLUTION";
  case FailureReason::OUT_OF_MAP:
    return "OUT_OF_MAP";
  case FailureReason::COSTMAP_UNKNOWN:
    return "COSTMAP_UNKNOWN";
  case FailureReason::COSTMAP_LETHAL:
    return "COSTMAP_LETHAL";
  case FailureReason::COSTMAP_INSCRIBED:
    return "COSTMAP_INSCRIBED";
  case FailureReason::QUERY_FAILED:
    return "QUERY_FAILED";
  case FailureReason::INVALID_SNAPSHOT_STAMP:
    return "INVALID_SNAPSHOT_STAMP";
  case FailureReason::INVALID_ROS_TIME:
    return "INVALID_ROS_TIME";
  case FailureReason::STALE_SNAPSHOT:
    return "STALE_SNAPSHOT";
  case FailureReason::FUTURE_SNAPSHOT:
    return "FUTURE_SNAPSHOT";
  case FailureReason::NONFINITE_DISTANCE:
    return "NONFINITE_DISTANCE";
  case FailureReason::INSUFFICIENT_CLEARANCE:
    return "INSUFFICIENT_CLEARANCE";
  }
  return "UNKNOWN";
}

void TrajectorySafetyChecker::configure(const Config &config,
                                        rclcpp::Logger logger,
                                        rclcpp::Clock::SharedPtr clock) {
  if (!std::isfinite(config.safe_dist) || config.safe_dist < 0.0 ||
      !std::isfinite(config.footprint_length) ||
      config.footprint_length <= 0.0 ||
      !std::isfinite(config.footprint_width) || config.footprint_width <= 0.0 ||
      !std::isfinite(config.footprint_margin) ||
      config.footprint_margin <= 0.0 || !std::isfinite(config.sample_dt) ||
      config.sample_dt <= 0.0 || !std::isfinite(config.map_timeout) ||
      config.map_timeout <= 0.0 || !std::isfinite(config.future_tolerance) ||
      config.future_tolerance < 0.0) {
    throw std::invalid_argument(
        "TrajectorySafetyChecker configuration is invalid");
  }
  if (!clock) {
    throw std::invalid_argument(
        "TrajectorySafetyChecker requires a valid ROS clock");
  }

  safe_dist_ = config.safe_dist;
  footprint_length_ = config.footprint_length;
  footprint_width_ = config.footprint_width;
  footprint_margin_ = config.footprint_margin;
  sample_dt_ = config.sample_dt;
  map_timeout_ = config.map_timeout;
  future_tolerance_ = config.future_tolerance;
  planning_frame_ = config.planning_frame.empty() ? "unknown" : config.planning_frame;
  rog_frame_ = config.rog_frame.empty() ? "unknown" : config.rog_frame;
  logger_ = logger;
  clock_ = std::move(clock);
  clearTrajectoryFailure();
}

void TrajectorySafetyChecker::setQuery(
    std::shared_ptr<rog_map::MapQueryInterface> dynamic_query) {
  std::lock_guard<std::mutex> lock(query_mutex_);
  dynamic_query_ = std::move(dynamic_query);
}

std::shared_ptr<rog_map::MapQueryInterface>
TrajectorySafetyChecker::querySnapshot() const {
  std::shared_ptr<rog_map::MapQueryInterface> query;
  {
    std::lock_guard<std::mutex> lock(query_mutex_);
    query = dynamic_query_;
  }
  if (query) {
    return query;
  }
  RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000,
                        "[MincoPlanner] ROGMap dynamic query is unavailable "
                        "for trajectory safety check.");
  return nullptr;
}

bool TrajectorySafetyChecker::checkPoint(const Eigen::Vector3d &pos) const {
  return checkPoint(querySnapshot(), pos);
}

TrajectorySafetyChecker::PointCheckResult
TrajectorySafetyChecker::evaluatePoint(
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    const Eigen::Vector3d &pos) const {
  if (!query) {
    return evaluateQueryResult(
        query, pos, rog_map::QueryResult{},
        clock_ ? clock_->now().seconds()
               : std::numeric_limits<double>::quiet_NaN());
  }
  const auto results = query->queryBatch({pos});
  const rog_map::QueryResult result =
      results.size() == 1U ? results.front() : rog_map::QueryResult{};
  return evaluateQueryResult(
      query, pos, result,
      clock_ ? clock_->now().seconds()
             : std::numeric_limits<double>::quiet_NaN());
}

TrajectorySafetyChecker::PointCheckResult
TrajectorySafetyChecker::evaluateQueryResult(
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    const Eigen::Vector3d &pos, const rog_map::QueryResult &result,
    double query_time) const {
  PointCheckResult check;
  check.diagnostic.query_point = pos;
  check.diagnostic.safe_distance = safe_dist_;
  check.diagnostic.planning_frame = planning_frame_;
  check.diagnostic.rog_frame = rog_frame_;

  // The 2D projection rejects unknown/lethal cells first. The 3D ESDF then
  // enforces the configured clearance. Keep the exact first failed stage so a
  // fail-closed rejection can be diagnosed without repeating any successful
  // map query.
  if (!query) {
    check.diagnostic.reason = FailureReason::QUERY_UNAVAILABLE;
    return check;
  }
  if (!pos.allFinite()) {
    check.diagnostic.reason = FailureReason::NONFINITE_CENTER;
    check.diagnostic.query_status = rog_map::QueryStatus::NONFINITE_INPUT;
    return check;
  }

  check.diagnostic.query_attempted = true;
  check.diagnostic.query_status = result.status;
  check.diagnostic.distance = result.distance;
  check.diagnostic.projection = result.projection;
  check.diagnostic.snapshot_processing_age_ms =
      result.snapshot_processing_age_ms;
  if (std::isfinite(query_time) && std::isfinite(result.snapshot_stamp)) {
    check.diagnostic.snapshot_age = query_time - result.snapshot_stamp;
  }
  if (std::isfinite(query_time) &&
      std::isfinite(result.snapshot_commit_stamp)) {
    check.diagnostic.snapshot_commit_age =
        query_time - result.snapshot_commit_stamp;
  }

  unsigned char cost = nav2_costmap_2d::NO_INFORMATION;
  bool cost_valid = result.projected_cost_valid;
  if (cost_valid) {
    cost = result.projected_cost;
  } else {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (query->worldToMap(pos.x(), pos.y(), mx, my)) {
      cost = query->value(mx, my);
      cost_valid = true;
    }
  }
  if (!cost_valid) {
    check.diagnostic.reason =
        (!result.ok && result.status != rog_map::QueryStatus::OUT_OF_MAP)
            ? FailureReason::QUERY_FAILED
            : FailureReason::OUT_OF_MAP;
    return check;
  }

  check.diagnostic.cost = cost;
  check.diagnostic.cost_checked = true;
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    check.diagnostic.reason = FailureReason::COSTMAP_UNKNOWN;
    return check;
  }
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    check.diagnostic.reason = FailureReason::COSTMAP_LETHAL;
    return check;
  }
  if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    check.diagnostic.reason = FailureReason::COSTMAP_INSCRIBED;
    return check;
  }

  if (!result.ok) {
    check.diagnostic.reason = FailureReason::QUERY_FAILED;
    return check;
  }

  check.diagnostic.reason =
      freshnessFailureAt(result, query_time, check.diagnostic.snapshot_age);
  if (check.diagnostic.reason != FailureReason::NONE) {
    return check;
  }
  if (!std::isfinite(result.distance)) {
    check.diagnostic.reason = FailureReason::NONFINITE_DISTANCE;
    return check;
  }
  // With zero requested inflation, the exact projected cell checked above is
  // authoritative. Trilinear ESDF interpolation can become slightly negative
  // at a free cell next to an occupied/unknown cell; treating that interpolation
  // artifact as a collision makes a zero-clearance footprint larger than its
  // configured geometry. The query is still required here so TF, snapshot
  // freshness and finite-distance failures remain fail-closed.
  if (safe_dist_ > 0.0 && result.distance <= safe_dist_) {
    check.diagnostic.reason = FailureReason::INSUFFICIENT_CLEARANCE;
    return check;
  }

  check.safe = true;
  check.diagnostic.reason = FailureReason::NONE;
  return check;
}

bool TrajectorySafetyChecker::checkPoint(
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    const Eigen::Vector3d &pos) const {
  return evaluatePoint(query, pos).safe;
}

TrajectorySafetyChecker::FailureReason
TrajectorySafetyChecker::freshnessFailure(
    const rog_map::QueryResult &result, double &snapshot_age) const {
  if (!clock_) {
    snapshot_age = std::numeric_limits<double>::quiet_NaN();
    return FailureReason::INVALID_ROS_TIME;
  }
  return freshnessFailureAt(result, clock_->now().seconds(), snapshot_age);
}

TrajectorySafetyChecker::FailureReason
TrajectorySafetyChecker::freshnessFailureAt(
    const rog_map::QueryResult &result, double query_time,
    double &snapshot_age) const {
  snapshot_age = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(result.snapshot_stamp) || result.snapshot_stamp <= 0.0) {
    return FailureReason::INVALID_SNAPSHOT_STAMP;
  }

  if (!std::isfinite(query_time) || query_time <= 0.0) {
    return FailureReason::INVALID_ROS_TIME;
  }

  snapshot_age = query_time - result.snapshot_stamp;
  if (!std::isfinite(snapshot_age)) {
    return FailureReason::INVALID_ROS_TIME;
  }
  if (snapshot_age > map_timeout_) {
    return FailureReason::STALE_SNAPSHOT;
  }
  if (snapshot_age < -future_tolerance_) {
    return FailureReason::FUTURE_SNAPSHOT;
  }
  return FailureReason::NONE;
}

bool TrajectorySafetyChecker::queryIsFresh(
    const rog_map::QueryResult &result) const {
  double snapshot_age = std::numeric_limits<double>::quiet_NaN();
  const auto reason = freshnessFailure(result, snapshot_age);
  if (reason != FailureReason::NONE && clock_) {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[MincoPlanner] ROGMap query rejected: reason=%s "
        "snapshot_age=%.3f timeout=%.3f future_tolerance=%.3f.",
        failureReasonName(reason), snapshot_age, map_timeout_, future_tolerance_);
  }
  return reason == FailureReason::NONE;
}

bool TrajectorySafetyChecker::checkFootprint(const Eigen::Vector3d &pos,
                                             double yaw) const {
  return checkFootprint(querySnapshot(), pos, yaw);
}

bool TrajectorySafetyChecker::checkFootprint(
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    const Eigen::Vector3d &pos, double yaw) const {
  FailureDiagnostic failure;
  const bool safe = evaluateFootprint(
      query, pos, yaw, std::numeric_limits<double>::quiet_NaN(), &failure);
  if (!safe) {
    recordTrajectoryFailure(failure,
                            std::numeric_limits<double>::quiet_NaN());
  } else {
    clearTrajectoryFailure();
  }
  return safe;
}

bool TrajectorySafetyChecker::checkSweptFootprint(
    const Eigen::Vector3d &from_pos, double from_yaw,
    const Eigen::Vector3d &to_pos, double to_yaw) const {
  const auto query = querySnapshot();
  FailureDiagnostic failure;
  if (!query) {
    failure.reason = FailureReason::QUERY_UNAVAILABLE;
    recordTrajectoryFailure(failure,
                            std::numeric_limits<double>::quiet_NaN());
    return false;
  }

  if (!from_pos.allFinite() || !to_pos.allFinite()) {
    failure.reason = FailureReason::NONFINITE_CENTER;
    failure.center = !from_pos.allFinite() ? from_pos : to_pos;
    failure.query_point = failure.center;
    recordTrajectoryFailure(failure,
                            std::numeric_limits<double>::quiet_NaN());
    return false;
  }
  if (!std::isfinite(from_yaw) || !std::isfinite(to_yaw)) {
    failure.reason = FailureReason::NONFINITE_YAW;
    failure.yaw = !std::isfinite(from_yaw) ? from_yaw : to_yaw;
    recordTrajectoryFailure(failure,
                            std::numeric_limits<double>::quiet_NaN());
    return false;
  }

  const double resolution = query->resolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    failure.reason = FailureReason::INVALID_MAP_RESOLUTION;
    recordTrajectoryFailure(failure,
                            std::numeric_limits<double>::quiet_NaN());
    return false;
  }

  const double yaw_delta =
      std::atan2(std::sin(to_yaw - from_yaw), std::cos(to_yaw - from_yaw));
  const double translation = (to_pos - from_pos).head<2>().norm();
  const double corner_radius = std::hypot(
      0.5 * footprint_length_ + footprint_margin_,
      0.5 * footprint_width_ + footprint_margin_);
  const double maximum_corner_travel =
      translation + corner_radius * std::abs(yaw_delta);
  if (!std::isfinite(maximum_corner_travel)) {
    failure.reason = FailureReason::NONFINITE_CENTER;
    recordTrajectoryFailure(failure,
                            std::numeric_limits<double>::quiet_NaN());
    return false;
  }

  // Bound every footprint corner's motion between checks to half a projected
  // grid cell. This covers both translation and in-place rotation while joining
  // the controller's spatially nearest point on a rolling trajectory.
  const int samples = std::max(
      1, static_cast<int>(std::ceil(maximum_corner_travel /
                                    std::max(1e-6, 0.5 * resolution))));
  for (int sample = 0; sample <= samples; ++sample) {
    const double ratio =
        static_cast<double>(sample) / static_cast<double>(samples);
    const Eigen::Vector3d position = from_pos + ratio * (to_pos - from_pos);
    const double yaw = from_yaw + ratio * yaw_delta;
    if (!evaluateFootprint(query, position, yaw,
                           std::numeric_limits<double>::quiet_NaN(), &failure)) {
      recordTrajectoryFailure(failure,
                              std::numeric_limits<double>::quiet_NaN());
      return false;
    }
  }
  clearTrajectoryFailure();
  return true;
}

bool TrajectorySafetyChecker::evaluateFootprint(
    const std::shared_ptr<rog_map::MapQueryInterface> &query,
    const Eigen::Vector3d &pos, double yaw, double trajectory_time,
    FailureDiagnostic *failure) const {
  FailureDiagnostic initial;
  initial.center = pos;
  initial.query_point = pos;
  initial.yaw = yaw;
  initial.trajectory_time = trajectory_time;
  initial.safe_distance = safe_dist_;
  initial.planning_frame = planning_frame_;
  initial.rog_frame = rog_frame_;

  if (!query) {
    initial.reason = FailureReason::QUERY_UNAVAILABLE;
    if (failure) {
      *failure = initial;
    }
    return false;
  }
  if (!pos.allFinite()) {
    initial.reason = FailureReason::NONFINITE_CENTER;
    initial.query_status = rog_map::QueryStatus::NONFINITE_INPUT;
    if (failure) {
      *failure = initial;
    }
    return false;
  }
  if (!std::isfinite(yaw)) {
    initial.reason = FailureReason::NONFINITE_YAW;
    if (failure) {
      *failure = initial;
    }
    return false;
  }

  const double resolution = query->resolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    initial.reason = FailureReason::INVALID_MAP_RESOLUTION;
    if (failure) {
      *failure = initial;
    }
    return false;
  }

  // footprint_length/width are full vehicle dimensions. Margin expands each
  // side independently before the rectangle is rotated by the planned yaw.
  const double half_length = 0.5 * footprint_length_ + footprint_margin_;
  const double half_width = 0.5 * footprint_width_ + footprint_margin_;
  const double clearance_spacing =
      safe_dist_ > 1e-6 ? std::sqrt(2.0) * safe_dist_ : 0.5 * resolution;
  const double target_spacing = std::max(0.5 * resolution, clearance_spacing);
  const size_t x_segments = std::max<size_t>(
      1U, static_cast<size_t>(std::ceil((2.0 * half_length) / target_spacing)));
  const size_t y_segments = std::max<size_t>(
      1U, static_cast<size_t>(std::ceil((2.0 * half_width) / target_spacing)));

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  std::vector<Eigen::Vector3d> samples;
  std::vector<Eigen::Vector2d> offsets;
  samples.reserve((x_segments + 1U) * (y_segments + 1U) + 1U);
  offsets.reserve(samples.capacity());
  for (size_t ix = 0; ix <= x_segments; ++ix) {
    const double local_x =
        -half_length + (2.0 * half_length * static_cast<double>(ix)) /
                           static_cast<double>(x_segments);
    for (size_t iy = 0; iy <= y_segments; ++iy) {
      const double local_y =
          -half_width + (2.0 * half_width * static_cast<double>(iy)) /
                            static_cast<double>(y_segments);
      Eigen::Vector3d sample = pos;
      sample.x() += cos_yaw * local_x - sin_yaw * local_y;
      sample.y() += sin_yaw * local_x + cos_yaw * local_y;
      samples.push_back(sample);
      offsets.emplace_back(local_x, local_y);
    }
  }

  // Even segment counts include the center, but odd counts do not. Keep an
  // explicit center sample so map/TF health never depends on the grid parity.
  const size_t footprint_sample_count = samples.size();
  samples.push_back(pos);
  offsets.emplace_back(Eigen::Vector2d::Zero());

  const auto results = query->queryBatch(samples);
  if (results.size() != samples.size()) {
    initial.reason = FailureReason::QUERY_FAILED;
    if (failure) {
      *failure = initial;
    }
    return false;
  }

  // All results produced by QueryAdapter come from one immutable ROG snapshot
  // and FrameAwareRogQuery applies one fixed transform to the complete batch.
  const double query_time =
      clock_ ? clock_->now().seconds()
             : std::numeric_limits<double>::quiet_NaN();
  for (size_t index = 0; index < samples.size(); ++index) {
    auto point_check =
        evaluateQueryResult(query, samples[index], results[index], query_time);
    if (point_check.safe) {
      continue;
    }
    point_check.diagnostic.center = pos;
    point_check.diagnostic.yaw = yaw;
    point_check.diagnostic.trajectory_time = trajectory_time;
    point_check.diagnostic.footprint_offset = offsets[index];
    point_check.diagnostic.footprint_sample = index < footprint_sample_count;
    if (failure) {
      *failure = point_check.diagnostic;
    }
    return false;
  }

  // Account for time spent validating a large batch. The oldest result is
  // checked at completion, so exceeding the freshness budget remains
  // fail-closed instead of being hidden by a fast initial probe.
  const double completion_time =
      clock_ ? clock_->now().seconds()
             : std::numeric_limits<double>::quiet_NaN();
  for (size_t index = 0; index < results.size(); ++index) {
    double snapshot_age = std::numeric_limits<double>::quiet_NaN();
    const auto reason =
        freshnessFailureAt(results[index], completion_time, snapshot_age);
    if (reason == FailureReason::NONE) {
      continue;
    }
    initial.reason = reason;
    initial.query_attempted = true;
    initial.query_status = results[index].status;
    initial.distance = results[index].distance;
    initial.snapshot_age = snapshot_age;
    initial.projection = results[index].projection;
    initial.snapshot_processing_age_ms =
        results[index].snapshot_processing_age_ms;
    if (std::isfinite(completion_time) &&
        std::isfinite(results[index].snapshot_commit_stamp)) {
      initial.snapshot_commit_age =
          completion_time - results[index].snapshot_commit_stamp;
    }
    initial.query_point = samples[index];
    initial.footprint_offset = offsets[index];
    initial.footprint_sample = index < footprint_sample_count;
    if (failure) {
      *failure = initial;
    }
    return false;
  }
  return true;
}

bool TrajectorySafetyChecker::computeSpatialCheckStartTime(
    const traj_opt::Trajectory &traj,
    const Eigen::Vector3d &actual_position,
    double &start_time) const {
  start_time = std::numeric_limits<double>::quiet_NaN();
  const double duration = traj.getTotalDuration();
  if (!actual_position.allFinite() || !std::isfinite(duration) || duration <= 0.0 ||
      !std::isfinite(sample_dt_) || sample_dt_ <= 0.0) {
    return false;
  }

  struct TrajectorySample {
    double time;
    Eigen::Vector3d position;
  };
  std::vector<TrajectorySample> samples;
  samples.reserve(static_cast<size_t>(std::ceil(duration / sample_dt_)) + 1U);
  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  size_t nearest_index = 0U;
  auto append_sample = [&](double t) {
    const Eigen::Vector3d position = traj.getPos(t);
    if (!position.allFinite()) {
      return false;
    }
    const double distance_squared =
        (position.head<2>() - actual_position.head<2>()).squaredNorm();
    if (!std::isfinite(distance_squared)) {
      return false;
    }
    samples.push_back({t, position});
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      nearest_index = samples.size() - 1U;
    }
    return true;
  };

  for (double t = 0.0; t < duration; t += sample_dt_) {
    if (!append_sample(t)) {
      return false;
    }
  }
  if (!append_sample(duration) || samples.empty()) {
    return false;
  }

  // Match MincoMpcController::buildReferenceFromOptPath: choose the nearest
  // published sample, then refine only on its forward segment. Wall-clock time
  // is deliberately not used because the controller itself is spatially
  // indexed. The swept-footprint join check covers any tracking offset.
  start_time = samples[nearest_index].time;
  if (nearest_index + 1U < samples.size()) {
    const auto &current = samples[nearest_index];
    const auto &next = samples[nearest_index + 1U];
    const Eigen::Vector2d segment =
        (next.position - current.position).head<2>();
    const Eigen::Vector2d offset =
        (actual_position - current.position).head<2>();
    const double length_squared = segment.squaredNorm();
    if (length_squared > 1e-6) {
      const double projection = segment.dot(offset) / length_squared;
      if (projection > -0.5 && projection < 1.0) {
        start_time += projection * (next.time - current.time);
      }
    }
  }
  start_time = std::clamp(start_time, 0.0, duration);
  return std::isfinite(start_time);
}

bool TrajectorySafetyChecker::checkTrajectory(
    const traj_opt::Trajectory &traj) const {
  return checkTrajectoryFromTime(traj, 0.0);
}

bool TrajectorySafetyChecker::checkTrajectoryFromTime(
    const traj_opt::Trajectory &traj, double start_time) const {
  const auto query = querySnapshot();
  if (!query) {
    FailureDiagnostic failure;
    failure.reason = FailureReason::QUERY_UNAVAILABLE;
    failure.planning_frame = planning_frame_;
    failure.rog_frame = rog_frame_;
    recordTrajectoryFailure(failure, traj.getTotalDuration());
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!std::isfinite(dur) || dur <= 0.0) {
    FailureDiagnostic failure;
    failure.reason = FailureReason::INVALID_TRAJECTORY_DURATION;
    failure.planning_frame = planning_frame_;
    failure.rog_frame = rog_frame_;
    recordTrajectoryFailure(failure, dur);
    return false;
  }
  if (!std::isfinite(start_time)) {
    FailureDiagnostic failure;
    failure.reason = FailureReason::INVALID_TRAJECTORY_START_TIME;
    failure.trajectory_time = start_time;
    failure.planning_frame = planning_frame_;
    failure.rog_frame = rog_frame_;
    recordTrajectoryFailure(failure, dur);
    return false;
  }

  const double first_time = std::clamp(start_time, 0.0, dur);
  for (double t = first_time; t < dur; t += sample_dt_) {
    const Eigen::Vector3d center = traj.getPos(t);
    auto point_check = evaluatePoint(query, center);
    if (!point_check.safe) {
      point_check.diagnostic.center = center;
      point_check.diagnostic.trajectory_time = t;
      recordTrajectoryFailure(point_check.diagnostic, dur);
      return false;
    }
  }

  const Eigen::Vector3d endpoint = traj.getPos(dur);
  auto endpoint_check = evaluatePoint(query, endpoint);
  if (!endpoint_check.safe) {
    endpoint_check.diagnostic.center = endpoint;
    endpoint_check.diagnostic.trajectory_time = dur;
    recordTrajectoryFailure(endpoint_check.diagnostic, dur);
    return false;
  }
  clearTrajectoryFailure();
  return true;
}

bool TrajectorySafetyChecker::checkTrajectory(
    const traj_opt::Trajectory &position_traj,
    const traj_opt::Trajectory &yaw_traj) const {
  return checkTrajectoryFromTime(position_traj, yaw_traj, 0.0);
}

bool TrajectorySafetyChecker::checkTrajectoryFromTime(
    const traj_opt::Trajectory &position_traj,
    const traj_opt::Trajectory &yaw_traj, double start_time) const {
  const auto query = querySnapshot();
  if (!query) {
    FailureDiagnostic failure;
    failure.reason = FailureReason::QUERY_UNAVAILABLE;
    failure.planning_frame = planning_frame_;
    failure.rog_frame = rog_frame_;
    recordTrajectoryFailure(failure, position_traj.getTotalDuration());
    return false;
  }

  const double position_duration = position_traj.getTotalDuration();
  const double yaw_duration = yaw_traj.getTotalDuration();
  if (!std::isfinite(position_duration) || position_duration <= 0.0 ||
      !std::isfinite(yaw_duration)) {
    FailureDiagnostic failure;
    failure.reason = FailureReason::INVALID_TRAJECTORY_DURATION;
    failure.planning_frame = planning_frame_;
    failure.rog_frame = rog_frame_;
    recordTrajectoryFailure(failure, position_duration);
    return false;
  }
  if (!std::isfinite(start_time)) {
    FailureDiagnostic failure;
    failure.reason = FailureReason::INVALID_TRAJECTORY_START_TIME;
    failure.trajectory_time = start_time;
    failure.planning_frame = planning_frame_;
    failure.rog_frame = rog_frame_;
    recordTrajectoryFailure(failure, position_duration);
    return false;
  }
  if (yaw_duration + 1e-6 < position_duration) {
    FailureDiagnostic failure;
    failure.reason = FailureReason::INVALID_YAW_TRAJECTORY;
    failure.planning_frame = planning_frame_;
    failure.rog_frame = rog_frame_;
    recordTrajectoryFailure(failure, position_duration);
    return false;
  }

  auto check_sample = [&](double t, FailureDiagnostic &failure) {
    const Eigen::Vector3d pos = position_traj.getPos(t);
    const Eigen::Vector3d yaw_state =
        yaw_traj.getPos(std::min(t, yaw_duration));
    if (!yaw_state.allFinite()) {
      failure.reason = FailureReason::NONFINITE_YAW;
      failure.center = pos;
      failure.query_point = pos;
      failure.trajectory_time = t;
      failure.safe_distance = safe_dist_;
      failure.planning_frame = planning_frame_;
      failure.rog_frame = rog_frame_;
      return false;
    }
    return evaluateFootprint(query, pos, yaw_state.x(), t, &failure);
  };

  const double first_time = std::clamp(start_time, 0.0, position_duration);
  for (double t = first_time; t < position_duration; t += sample_dt_) {
    FailureDiagnostic failure;
    if (!check_sample(t, failure)) {
      recordTrajectoryFailure(failure, position_duration);
      return false;
    }
  }
  FailureDiagnostic endpoint_failure;
  if (!check_sample(position_duration, endpoint_failure)) {
    recordTrajectoryFailure(endpoint_failure, position_duration);
    return false;
  }
  clearTrajectoryFailure();
  return true;
}

void TrajectorySafetyChecker::recordTrajectoryFailure(
    FailureDiagnostic diagnostic, double trajectory_duration) const {
  diagnostic.trajectory_duration = trajectory_duration;
  diagnostic.safe_distance = safe_dist_;
  diagnostic.planning_frame = planning_frame_;
  diagnostic.rog_frame = rog_frame_;
  {
    std::lock_guard<std::mutex> lock(failure_mutex_);
    last_failure_ = diagnostic;
  }

  if (!clock_) {
    return;
  }
  const char *query_status = diagnostic.query_attempted
                                 ? rog_map::queryStatusName(diagnostic.query_status)
                                 : "NOT_QUERIED";
  const int cost = diagnostic.cost_checked ? static_cast<int>(diagnostic.cost) : -1;
  const auto & projection = diagnostic.projection;
  const char * cost_source = projection.valid
                                ? rog_map::projectedCostSourceName(
                                    projection.cost_source)
                                : "UNAVAILABLE";
  const char * cost_cause = projection.valid
                               ? rog_map::projectedCostCauseName(
                                   projection.cost_cause)
                               : "UNAVAILABLE";
  const char * cell_type = projection.valid
                              ? rog_map::projectionCellTypeName(
                                  projection.cell_type)
                              : "UNAVAILABLE";
  const char * raw_type = projection.valid
                             ? rog_map::projectionCellTypeName(
                                 projection.raw_type)
                             : "UNAVAILABLE";
  const char * candidate_type = projection.valid
                                   ? rog_map::projectionCellTypeName(
                                       projection.candidate_type)
                                   : "UNAVAILABLE";
  const char * base_type = projection.valid
                              ? rog_map::projectionCellTypeName(
                                  projection.base_type)
                              : "UNAVAILABLE";
  const char * pending_type = projection.valid
                                 ? rog_map::projectionCellTypeName(
                                     projection.pending_type)
                                 : "UNAVAILABLE";
  const char * raw_reason = projection.valid
                               ? rog_map::projectionClassReasonName(
                                   projection.raw_reason)
                               : "UNAVAILABLE";
  const char * candidate_reason = projection.valid
                                     ? rog_map::projectionClassReasonName(
                                         projection.candidate_reason)
                                     : "UNAVAILABLE";
  const int dynamic_cost =
      projection.valid ? static_cast<int>(projection.dynamic_cost) : -1;
  RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[MincoPlanner] Trajectory safety rejected: reason=%s t=%.3f/%.3f "
      "sample=%s center=(%.3f,%.3f,%.3f) query_point=(%.3f,%.3f,%.3f) "
      "footprint_offset=(%.3f,%.3f) yaw=%.3f esdf_distance=%.3f "
      "safe_distance=%.3f cost=%d query_status=%s snapshot_age=%.3f "
      "snapshot_commit_age=%.3f pipeline_age_ms=%.1f "
      "cost_source=%s cost_cause=%s dynamic_cost=%d cell_type=%s raw_type=%s "
      "candidate_type=%s base_type=%s pending_type=%s pending_count=%u "
      "hole_filled=%d hold_remaining=%.3f raw_reason=%s "
      "candidate_reason=%s prior_occupied=%d prior_free=%d "
      "occupied_z=[%.3f,%.3f] height_delta=%.3f vertical_ratio=%.3f "
      "ground_z=%.3f support_z=%.3f ceiling_z=%.3f headroom=%.3f "
      "headroom_known=%.3f ground_candidate=%d ground_verified=%d "
      "support_known=%d support_verified=%d empty_support=%d "
      "clearance_verified=%d footprint_eligible=%d inside_footprint=%d "
      "inside_near_field=%d envelope_body=(%.3f,%.3f) "
      "continuous_support=%d support_ref=%.3f "
      "support_error=%.3f support_tolerance=%.3f confidence=%.3f "
      "planning_frame='%s' "
      "rog_frame='%s'.",
      failureReasonName(diagnostic.reason), diagnostic.trajectory_time,
      diagnostic.trajectory_duration,
      diagnostic.footprint_sample ? "footprint" : "center",
      diagnostic.center.x(), diagnostic.center.y(), diagnostic.center.z(),
      diagnostic.query_point.x(), diagnostic.query_point.y(),
      diagnostic.query_point.z(), diagnostic.footprint_offset.x(),
      diagnostic.footprint_offset.y(), diagnostic.yaw, diagnostic.distance,
      diagnostic.safe_distance, cost, query_status, diagnostic.snapshot_age,
      diagnostic.snapshot_commit_age, diagnostic.snapshot_processing_age_ms,
      cost_source, cost_cause, dynamic_cost, cell_type, raw_type, candidate_type,
      base_type, pending_type, static_cast<unsigned>(projection.pending_count),
      projection.hole_filled ? 1 : 0, projection.obstacle_hold_remaining,
      raw_reason, candidate_reason,
      projection.prior_occupied ? 1 : 0,
      projection.prior_known_free ? 1 : 0,
      projection.occupied_z_min, projection.occupied_z_max,
      projection.height_delta, projection.vertical_occupancy_ratio,
      projection.ground_z, projection.ground_support_z,
      projection.ceiling_z, projection.headroom,
      projection.headroom_known_ratio,
      projection.ground_candidate ? 1 : 0,
      projection.ground_verified ? 1 : 0,
      projection.ground_support_known ? 1 : 0,
      projection.ground_support_verified ? 1 : 0,
      projection.empty_support_verified ? 1 : 0,
      projection.clearance_verified ? 1 : 0,
      projection.footprint_clear_eligible ? 1 : 0,
      projection.inside_current_footprint ? 1 : 0,
      projection.inside_surveyed_near_field ? 1 : 0,
      projection.envelope_body_x, projection.envelope_body_y,
      projection.continuous_ground_support ? 1 : 0,
      projection.reference_ground_z, projection.support_match_error,
      projection.support_match_tolerance, projection.confidence,
      diagnostic.planning_frame.c_str(),
      diagnostic.rog_frame.c_str());
}

void TrajectorySafetyChecker::clearTrajectoryFailure() const {
  FailureDiagnostic clear;
  clear.safe_distance = safe_dist_;
  clear.planning_frame = planning_frame_;
  clear.rog_frame = rog_frame_;
  std::lock_guard<std::mutex> lock(failure_mutex_);
  last_failure_ = std::move(clear);
}

TrajectorySafetyChecker::FailureDiagnostic
TrajectorySafetyChecker::lastFailureDiagnostic() const {
  std::lock_guard<std::mutex> lock(failure_mutex_);
  return last_failure_;
}

double TrajectorySafetyChecker::getDistance(const Eigen::Vector3d &pos) const {
  const auto dynamic_query = querySnapshot();
  if (!dynamic_query) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double dist = 0.0;
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query->query(pos);
  dist = query.distance;
  grad = query.gradient;
  (void)grad;
  return query.ok && queryIsFresh(query)
             ? dist
             : std::numeric_limits<double>::quiet_NaN();
}

bool TrajectorySafetyChecker::projectOutOfObstacle(Eigen::Vector3d &pos,
                                                   double margin) const {
  const auto dynamic_query = querySnapshot();
  if (!dynamic_query) {
    return false;
  }
  double esdf_dist = 0.0;
  Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
  const auto query = dynamic_query->query(pos);
  if (!query.ok || !queryIsFresh(query)) {
    return false;
  }
  esdf_dist = query.distance;
  esdf_grad = query.gradient;
  if (esdf_dist < 0.0 && esdf_grad.norm() > 1e-6) {
    pos += (margin - esdf_dist) * esdf_grad.normalized();
    return true;
  }
  return false;
}

} // namespace minco_planner
