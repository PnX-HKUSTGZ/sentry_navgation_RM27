#include "minco_core/components/local_path_processor.hpp"

namespace minco_planner {

namespace {

bool isLineFree(const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  if (!map || map->resolution() <= 0.0) {
    return true;
  }
  const double dist = (p2 - p1).norm();
  const int steps = static_cast<int>(std::ceil(dist / map->resolution()));
  if (steps <= 0) {
    return true;
  }
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector3d p = p1 + (p2 - p1) * t;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!map->worldToMap(p.x(), p.y(), mx, my) || !map->isFree(mx, my)) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool shouldOptimizeYawForSeed(
  bool yaw_optimization_enabled, const LocalPathSeed & seed) noexcept
{
  return yaw_optimization_enabled && !seed.observed_prefix_clipped;
}

void LocalPathProcessor::configure(
  double lookahead_dist,
  double max_vel,
  double max_acc,
  double traj_goal_tolerance,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
{
  lookahead_dist_ = lookahead_dist;
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
  logger_ = logger;
  clock_ = std::move(clock);
}

void LocalPathProcessor::updateLimits(double max_vel, double max_acc, double traj_goal_tolerance)
{
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
}

LocalPathSeed LocalPathProcessor::buildSeed(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const PlannerModeContext & mode_context,
  const FootprintSafetyCheck & footprint_is_safe) const
{
  LocalPathSeed seed;
  if (global_path.empty()) {
    return seed;
  }

  Eigen::Vector3d global_goal(global_path.back().pose.position.x, global_path.back().pose.position.y, 0.0);
  Eigen::Vector3d cur_pos(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);

  seed.dense_path = extractLocalPath(global_path, cur_pos);
  const bool clip_required =
    mode_context.mode() == PlannerMode::EXPLORATION || mode_context.clipSeedByRogBoundary();
  const bool clip_ok = clipLocalPathByRogBoundary(seed.dense_path, mode_context);
  if (clip_required && (!clip_ok || seed.dense_path.size() < 2U)) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] Local seed path is outside ROGMap boundary or too short after clipping.");
    return seed;
  }
  if (seed.dense_path.size() < 2U) {
    return seed;
  }

  double initial_yaw = 0.0;
  if (!utils::quaternionToYawChecked(current_pose.pose.orientation, initial_yaw)) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] Current pose has an invalid quaternion; reject local replan seed.");
    return seed;
  }
  if (!clipToObservedSafePrefix(
      seed.dense_path, mode_context, initial_yaw, footprint_is_safe,
      seed.observed_prefix_clipped))
  {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] No safely observed local seed prefix; reject local replan seed.");
    return seed;
  }
  if (seed.observed_prefix_clipped) {
    RCLCPP_INFO_THROTTLE(logger_,
      *clock_,
      1000,
      "[MincoPlanner] Clipped local seed to observed-safe prefix at (%.3f, %.3f); "
      "the local terminal state will stop.",
      seed.dense_path.back().x(), seed.dense_path.back().y());
  }

  seed.local_end_is_goal = (global_goal - seed.dense_path.back()).head<2>().norm() <= traj_goal_tolerance_;
  seed.stop_at_local_end = seed.local_end_is_goal || seed.observed_prefix_clipped;
  seed.sparse_waypoints = utils::getSparseWaypoints(seed.dense_path,
    max_vel_,
    max_acc_,
    seed.stop_at_local_end,
    [&mode_context](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
      return isLineFree(mode_context.sparsifyQuery(), a, b);
    });
  seed.valid = seed.sparse_waypoints.size() >= 2U;
  return seed;
}

bool LocalPathProcessor::clipToObservedSafePrefix(
  std::vector<Eigen::Vector3d> & path,
  const PlannerModeContext & mode_context,
  double initial_yaw,
  const FootprintSafetyCheck & footprint_is_safe,
  bool & clipped) const
{
  clipped = false;
  const auto query = mode_context.dynamicQuery();
  if (path.size() < 2U || !query || !footprint_is_safe) {
    path.clear();
    return false;
  }

  const double resolution = query->resolution();
  if (!std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(initial_yaw)) {
    path.clear();
    return false;
  }

  // Half-cell translation steps prevent the rectangular footprint from
  // jumping over a one-cell UNKNOWN or occupied band between seed vertices.
  const double sample_step = std::max(1e-3, 0.5 * resolution);
  std::vector<Eigen::Vector3d> safe_prefix;
  safe_prefix.reserve(path.size() + 1U);

  if (!footprint_is_safe(path.front(), initial_yaw)) {
    path.clear();
    return false;
  }
  safe_prefix.push_back(path.front());

  for (size_t i = 1U; i < path.size(); ++i) {
    const Eigen::Vector3d segment = path[i] - path[i - 1U];
    const double length = segment.head<2>().norm();
    if (!std::isfinite(length)) {
      path.clear();
      return false;
    }
    if (length <= 1e-9) {
      continue;
    }

    // This chassis is omnidirectional: path tangent and body heading are
    // independent. Seed clipping only establishes a translation-safe prefix,
    // so keep the measured physical yaw throughout instead of introducing an
    // instantaneous turn at a global-path vertex. The optimized, continuous
    // yaw trajectory is sampled again by the final publication gate.
    const double yaw = initial_yaw;
    if (!footprint_is_safe(path[i - 1U], yaw)) {
      clipped = true;
      path.swap(safe_prefix);
      return path.size() >= 2U;
    }

    const int samples = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
    Eigen::Vector3d last_safe = path[i - 1U];
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const Eigen::Vector3d position = path[i - 1U] + ratio * segment;
      if (!footprint_is_safe(position, yaw)) {
        clipped = true;
        if ((last_safe - safe_prefix.back()).head<2>().norm() > 1e-9) {
          safe_prefix.push_back(last_safe);
        }
        path.swap(safe_prefix);
        return path.size() >= 2U;
      }
      last_safe = position;
    }
    safe_prefix.push_back(path[i]);
  }

  path.swap(safe_prefix);
  return path.size() >= 2U;
}

std::vector<Eigen::Vector3d> LocalPathProcessor::extractLocalPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path, const Eigen::Vector3d & cur_pos) const
{
  std::vector<Eigen::Vector3d> local_segment;
  if (global_path.empty() || !cur_pos.allFinite()) {
    return local_segment;
  }

  size_t start_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_path.size(); ++i) {
    const auto & pt = global_path[i].pose.position;
    double dist_sq =
      (cur_pos.x() - pt.x) * (cur_pos.x() - pt.x) + (cur_pos.y() - pt.y) * (cur_pos.y() - pt.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  // A rolling global path is not regenerated for every local replan. Always
  // anchor the seed at the measured robot pose so the optimized trajectory does
  // not begin in a grid cell the robot has already left.
  Eigen::Vector3d current = cur_pos;
  current.z() = 0.0;
  local_segment.push_back(current);

  size_t first_forward_idx = start_idx + 1U;
  Eigen::Vector2d tangent = Eigen::Vector2d::Zero();
  if (start_idx + 1U < global_path.size()) {
    const auto & here = global_path[start_idx].pose.position;
    const auto & next = global_path[start_idx + 1U].pose.position;
    tangent = Eigen::Vector2d(next.x - here.x, next.y - here.y);
  } else if (start_idx > 0U) {
    const auto & previous = global_path[start_idx - 1U].pose.position;
    const auto & here = global_path[start_idx].pose.position;
    tangent = Eigen::Vector2d(here.x - previous.x, here.y - previous.y);
  }
  const auto & nearest = global_path[start_idx].pose.position;
  const Eigen::Vector2d to_nearest(nearest.x - current.x(), nearest.y - current.y());
  if (tangent.squaredNorm() > 1e-12 && to_nearest.dot(tangent) > 1e-9) {
    first_forward_idx = start_idx;
  }

  double accum_dist = 0.0;
  for (size_t i = first_forward_idx; i < global_path.size(); ++i) {
    const auto & point = global_path[i].pose.position;
    const Eigen::Vector3d candidate(point.x, point.y, 0.0);
    const double dist = (candidate - local_segment.back()).head<2>().norm();
    if (!std::isfinite(dist)) {
      return {};
    }
    if (dist <= 1e-9) {
      continue;
    }
    accum_dist += dist;
    local_segment.push_back(candidate);

    if (accum_dist >= lookahead_dist_) {
      break;
    }
  }

  return local_segment;
}

bool LocalPathProcessor::clipLocalPathByRogBoundary(
  std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const
{
  if (path.empty()) {
    return false;
  }

  const bool enable_clip =
    mode_context.mode() == PlannerMode::EXPLORATION || mode_context.clipSeedByRogBoundary();
  if (!enable_clip) {
    return path.size() >= 2U;
  }

  const auto query = mode_context.dynamicQuery();
  if (!query) {
    return false;
  }

  const double margin = mode_context.mode() == PlannerMode::PRIORMAP
                          ? mode_context.rogBoundaryMargin()
                          : mode_context.explorationBoundaryMargin();
  const int margin_cells =
    std::max(0, static_cast<int>(std::ceil(margin / std::max(1e-6, query->resolution()))));
  const int max_x = static_cast<int>(query->sizeX());
  const int max_y = static_cast<int>(query->sizeY());
  if (max_x <= 0 || max_y <= 0) {
    return false;
  }

  const double sample_step = mode_context.mode() == PlannerMode::PRIORMAP
                               ? mode_context.rogBoundarySampleStep()
                               : mode_context.explorationBoundarySampleStep();
  const double step = std::max(query->resolution(), std::max(1e-3, sample_step));

  auto inside_boundary = [&query, margin_cells, max_x, max_y](const Eigen::Vector3d & p) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!query->worldToMap(p.x(), p.y(), mx, my)) {
      return false;
    }
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);
    return ix >= margin_cells && iy >= margin_cells && ix < max_x - margin_cells &&
           iy < max_y - margin_cells;
  };

  std::vector<Eigen::Vector3d> clipped;
  clipped.reserve(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    if (!inside_boundary(path[i])) {
      if (i == 0U) {
        path.clear();
        RCLCPP_WARN_THROTTLE(logger_,
          *clock_,
          2000,
          "[MincoPlanner] Local seed path starts outside ROGMap boundary; reject local replan seed.");
        return false;
      }
      break;
    }
    if (i > 0U) {
      const Eigen::Vector3d delta = path[i] - path[i - 1U];
      const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / step)));
      for (int s = 1; s <= samples; ++s) {
        const double ratio = static_cast<double>(s) / static_cast<double>(samples);
        if (!inside_boundary(path[i - 1U] + ratio * delta)) {
          path.swap(clipped);
          return path.size() >= 2U;
        }
      }
    }
    clipped.push_back(path[i]);
  }

  path.swap(clipped);
  return path.size() >= 2U;
}

}  // namespace minco_planner
