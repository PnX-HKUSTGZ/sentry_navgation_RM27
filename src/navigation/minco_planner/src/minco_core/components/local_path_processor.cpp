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

struct CostProfile
{
  bool valid{false};
  uint8_t peak{nav2_costmap_2d::FREE_SPACE};
  double mean{0.0};
};

bool appendCostSample(
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const Eigen::Vector3d & point,
  uint64_t & cost_sum,
  size_t & sample_count,
  uint8_t & peak)
{
  unsigned int mx = 0U;
  unsigned int my = 0U;
  if (!map || !map->worldToMap(point.x(), point.y(), mx, my)) {
    return false;
  }
  const uint8_t cost = map->value(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION ||
    cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return false;
  }
  cost_sum += cost;
  ++sample_count;
  peak = std::max(peak, cost);
  return true;
}

CostProfile samplePolylineCosts(
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const std::vector<Eigen::Vector3d> & points,
  size_t begin,
  size_t end)
{
  CostProfile profile;
  if (!map || begin >= points.size() || end >= points.size() || begin > end) {
    return profile;
  }

  const double sample_step = std::max(1e-3, 0.5 * map->resolution());
  uint64_t cost_sum = 0U;
  size_t sample_count = 0U;
  uint8_t peak = nav2_costmap_2d::FREE_SPACE;
  for (size_t index = begin; index <= end; ++index) {
    const Eigen::Vector3d & start = points[index];
    const Eigen::Vector3d & finish = index < end ? points[index + 1U] : points[index];
    const double length = (finish - start).head<2>().norm();
    const int samples = index < end ?
      std::max(1, static_cast<int>(std::ceil(length / sample_step))) : 0;
    for (int sample = 0; sample <= samples; ++sample) {
      if (index > begin && sample == 0) {
        continue;
      }
      const double ratio = samples > 0 ?
        static_cast<double>(sample) / static_cast<double>(samples) : 0.0;
      if (!appendCostSample(
          map, start + ratio * (finish - start), cost_sum, sample_count, peak))
      {
        return profile;
      }
    }
  }

  profile.valid = sample_count > 0U;
  profile.peak = peak;
  profile.mean = profile.valid ?
    static_cast<double>(cost_sum) / static_cast<double>(sample_count) : 0.0;
  return profile;
}

bool shortcutRespectsCostEnvelope(
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const std::vector<Eigen::Vector3d> & reference_path,
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & b,
  double peak_slack,
  double mean_slack)
{
  size_t begin = reference_path.size();
  size_t end = reference_path.size();
  for (size_t index = 0U; index < reference_path.size(); ++index) {
    if (begin == reference_path.size() &&
      (reference_path[index] - a).head<2>().squaredNorm() <= 1e-12)
    {
      begin = index;
      continue;
    }
    if (begin != reference_path.size() && index > begin &&
      (reference_path[index] - b).head<2>().squaredNorm() <= 1e-12)
    {
      end = index;
      break;
    }
  }
  if (begin == reference_path.size() || end == reference_path.size()) {
    return true;
  }

  const CostProfile reference = samplePolylineCosts(map, reference_path, begin, end);
  const std::vector<Eigen::Vector3d> shortcut{a, b};
  const CostProfile candidate = samplePolylineCosts(map, shortcut, 0U, 1U);
  if (!reference.valid || !candidate.valid) {
    return false;
  }

  return static_cast<double>(candidate.peak) <=
           static_cast<double>(reference.peak) + peak_slack &&
         candidate.mean <= reference.mean + mean_slack;
}

}  // namespace

bool shouldOptimizeYawForSeed(
  bool yaw_optimization_enabled, const LocalPathSeed & seed) noexcept
{
  return yaw_optimization_enabled && seed.local_end_is_goal && !seed.observed_prefix_clipped;
}

void LocalPathProcessor::configure(
  double lookahead_dist,
  double max_vel,
  double max_acc,
  double traj_goal_tolerance,
  double shortcut_peak_cost_slack,
  double shortcut_mean_cost_slack,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
{
  lookahead_dist_ = lookahead_dist;
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
  shortcut_peak_cost_slack_ = shortcut_peak_cost_slack;
  shortcut_mean_cost_slack_ = shortcut_mean_cost_slack;
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
  const double shortcut_sample_step =
    std::max(1e-3, 0.5 * mode_context.dynamicQuery()->resolution());
  std::vector<Eigen::Vector3d> sparse_input = seed.dense_path;
  size_t connector_end = 0U;
  const auto global_query = mode_context.sparsifyQuery();
  unsigned int start_x = 0U, start_y = 0U;
  if (global_query && global_query->worldToMap(cur_pos.x(), cur_pos.y(), start_x, start_y) &&
      !global_query->isFree(start_x, start_y)) {
    // A sub-cell body pose can be safe while the cell-centred footprint mask
    // is lethal. Retain the already footprint-checked short connector to the
    // snapped global start; do not ask that same centre mask to veto it again.
    double connector_length = 0.0;
    for (size_t i = 1U; i < sparse_input.size(); ++i) {
      connector_length += (sparse_input[i] - sparse_input[i - 1U]).head<2>().norm();
      if (connector_length > 2.0 * global_query->resolution() + 1.0e-9) {break;}
      unsigned int x = 0U, y = 0U;
      if (global_query->worldToMap(sparse_input[i].x(), sparse_input[i].y(), x, y) &&
          global_query->isFree(x, y)) {
        connector_end = i;
        break;
      }
    }
    if (connector_end > 0U) {
      sparse_input.erase(sparse_input.begin(), sparse_input.begin() + connector_end);
    }
  }
  seed.sparse_waypoints = utils::getSparseWaypoints(sparse_input,
    max_vel_,
    max_acc_,
    seed.stop_at_local_end,
    [this, &mode_context, &footprint_is_safe, &seed, initial_yaw, shortcut_sample_step](
      const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
      if (!isLineFree(mode_context.sparsifyQuery(), a, b)) {
        return false;
      }
      // SMAC deliberately routes through the lower-cost side of Nav2's
      // inflation field. Do not let waypoint sparsification erase that choice
      // merely because a geometrically shorter segment is not yet lethal.
      if (!shortcutRespectsCostEnvelope(
          mode_context.sparsifyQuery(), seed.dense_path, a, b,
          shortcut_peak_cost_slack_, shortcut_mean_cost_slack_))
      {
        return false;
      }
      // A static line-of-sight shortcut must not erase a dynamic detour or cut
      // the rectangular footprint across an unobserved corner.
      const int samples = std::max(1, static_cast<int>(std::ceil(
        (b - a).head<2>().norm() / shortcut_sample_step)));
      for (int sample = 0; sample <= samples; ++sample) {
        const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
        if (!footprint_is_safe(a + ratio * (b - a), initial_yaw)) {
          return false;
        }
      }
      return true;
    });
  if (connector_end > 0U && !seed.sparse_waypoints.empty()) {
    seed.sparse_waypoints.insert(seed.sparse_waypoints.begin(),
      seed.dense_path.begin(), seed.dense_path.begin() + connector_end);
  }

  // MINCO is allowed to move its intermediate control points.  A long sparse
  // segment at a tunnel entrance can therefore bow toward a wall even though
  // its polyline endpoints are safe.  Keep the same validated guide geometry
  // at a bounded spacing; the polynomial remains smooth, while a narrow
  // doorway or a ramp corner cannot be cut by one oversized piece.
  constexpr double kMaxGuideSpacing = 0.25;
  bool guide_has_turn = false;
  for (size_t index = 1U; index + 1U < seed.sparse_waypoints.size(); ++index) {
    const Eigen::Vector2d incoming =
      (seed.sparse_waypoints[index] - seed.sparse_waypoints[index - 1U]).head<2>();
    const Eigen::Vector2d outgoing =
      (seed.sparse_waypoints[index + 1U] - seed.sparse_waypoints[index]).head<2>();
    if (incoming.norm() > 1.0e-6 && outgoing.norm() > 1.0e-6 &&
        std::abs(incoming.normalized().dot(outgoing.normalized())) < 0.995) {
      guide_has_turn = true;
      break;
    }
  }
  if (guide_has_turn) {
    std::vector<Eigen::Vector3d> densified;
    densified.reserve(seed.sparse_waypoints.size() * 2U);
    densified.push_back(seed.sparse_waypoints.front());
    for (size_t index = 1U; index < seed.sparse_waypoints.size(); ++index) {
      const Eigen::Vector3d delta =
        seed.sparse_waypoints[index] - seed.sparse_waypoints[index - 1U];
      const double length = delta.head<2>().norm();
      const int pieces = std::max(1, static_cast<int>(
        std::ceil(length / kMaxGuideSpacing)));
      for (int piece = 1; piece <= pieces; ++piece) {
        const double ratio = static_cast<double>(piece) /
          static_cast<double>(pieces);
        const Eigen::Vector3d point =
          seed.sparse_waypoints[index - 1U] + ratio * delta;
        if (!footprint_is_safe(point, initial_yaw)) {
          densified.clear();
          break;
        }
        densified.push_back(point);
      }
      if (densified.empty()) {
        break;
      }
    }
    if (densified.size() == seed.sparse_waypoints.size() ||
        (!densified.empty() && densified.back().isApprox(
          seed.sparse_waypoints.back(), 1.0e-9))) {
      seed.sparse_waypoints = std::move(densified);
    }
  }
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

  // Choose the closest point on the polyline rather than the closest stored
  // vertex. A sparse corner or nearby branch vertex can be closer in
  // Euclidean distance than the segment the robot is actually approaching.
  size_t nearest_segment = 0U;
  double nearest_segment_t = 0.0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0U; i + 1U < global_path.size(); ++i) {
    const auto & from = global_path[i].pose.position;
    const auto & to = global_path[i + 1U].pose.position;
    const Eigen::Vector2d segment(to.x - from.x, to.y - from.y);
    const double segment_sq = segment.squaredNorm();
    const Eigen::Vector2d offset(cur_pos.x() - from.x, cur_pos.y() - from.y);
    const double ratio = segment_sq > 1.0e-12
      ? std::clamp(offset.dot(segment) / segment_sq, 0.0, 1.0)
      : 0.0;
    const Eigen::Vector2d closest =
      Eigen::Vector2d(from.x, from.y) + ratio * segment;
    const double dist_sq =
      (cur_pos.x() - closest.x()) * (cur_pos.x() - closest.x()) +
      (cur_pos.y() - closest.y()) * (cur_pos.y() - closest.y());
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      nearest_segment = i;
      nearest_segment_t = ratio;
    }
  }

  // A rolling global path is not regenerated for every local replan. Always
  // anchor the seed at the measured robot pose so the optimized trajectory does
  // not begin in a grid cell the robot has already left.
  Eigen::Vector3d current = cur_pos;
  current.z() = 0.0;
  local_segment.push_back(current);

  // Retain the segment's starting vertex only when the projection is at its
  // beginning; otherwise continue at the segment endpoint. This keeps the
  // existing rolling-horizon behavior while avoiding branch-vertex jumps.
  const size_t first_forward_idx = nearest_segment +
    (nearest_segment_t <= 1.0e-6 ? 0U : 1U);

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
