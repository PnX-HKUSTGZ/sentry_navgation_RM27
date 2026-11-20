#include "traj_opt/minco_optimizer.hpp"

#include <algorithm>
#include "nav2_costmap_2d/cost_values.hpp"

#define POS_IDX 0
#define VEL_IDX 1
#define ACC_IDX 2
#define ATT_IDX 3
namespace minco_planner {
using Mat63f = Eigen::Matrix<double, 6, 3>;

void MincoOptimizer::InflationGrid::capture(
  const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  cells.clear();
  if (!map) {
    return;
  }
  width = map->sizeX();
  height = map->sizeY();
  resolution = map->resolution();
  if (width < 2U || height < 2U || !std::isfinite(resolution) || resolution <= 0.0) {
    return;
  }
  map->mapToWorld(0U, 0U, first_center.x(), first_center.y());
  if (!first_center.allFinite() || !map->copyValues(cells) ||
      cells.size() != static_cast<size_t>(width) * height) {
    cells.clear();
  }
}

bool MincoOptimizer::InflationGrid::traversable(const Eigen::Vector2d & point) const
{
  const Eigen::Vector2d index = (point - first_center) / resolution +
    Eigen::Vector2d::Constant(0.5);
  if (!index.allFinite() || index.x() < 0.0 || index.y() < 0.0 ||
      index.x() >= width || index.y() >= height) {
    return false;
  }
  return cells[static_cast<size_t>(index.y()) * width + static_cast<size_t>(index.x())] <
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

double MincoOptimizer::InflationGrid::sample(
  const Eigen::Vector2d & point, Eigen::Vector2d & gradient) const
{
  const Eigen::Vector2d index = (point - first_center) / resolution;
  gradient.setZero();
  if (!index.allFinite() || index.x() < -0.5 || index.y() < -0.5 ||
      index.x() >= width - 0.5 || index.y() >= height - 0.5) {
    return 1.0;
  }
  const double x = std::clamp(index.x(), 0.0, width - 1.0);
  const double y = std::clamp(index.y(), 0.0, height - 1.0);
  const auto ix = std::min(static_cast<unsigned int>(x), width - 2U);
  const auto iy = std::min(static_cast<unsigned int>(y), height - 2U);
  const auto value = [this](unsigned int col, unsigned int row) {
    constexpr double max_cost = nav2_costmap_2d::MAX_NON_OBSTACLE;
    return std::min(max_cost, static_cast<double>(cells[static_cast<size_t>(row) * width + col])) /
      max_cost;
  };
  const double u = x - ix;
  const double v = y - iy;
  const double c00 = value(ix, iy);
  const double c10 = value(ix + 1U, iy);
  const double c01 = value(ix, iy + 1U);
  const double c11 = value(ix + 1U, iy + 1U);
  const double lower = (1.0 - u) * c00 + u * c10;
  const double upper = (1.0 - u) * c01 + u * c11;
  if (index.x() == x) {
    gradient.x() = ((1.0 - v) * (c10 - c00) + v * (c11 - c01)) / resolution;
  }
  if (index.y() == y) {
    gradient.y() = (upper - lower) / resolution;
  }
  return (1.0 - v) * lower + v * upper;
}

void MincoOptimizer::prepareInflationCost()
{
  auto & vars = opt_vars_;
  vars.inflation_cost_weight = cfg_.inflation_cost_weight;
  const bool uses_guide_width = cfg_.guide_lateral_max_tolerance > cfg_.guide_lateral_tolerance;
  vars.inflation.capture(
    cfg_.inflation_cost_weight > 0.0 || uses_guide_width ? inflation_map_ : nullptr);
  vars.guide_lateral_tolerance = cfg_.guide_lateral_tolerance;
  vars.guide_side_offsets.setZero(2, vars.piece_num);
  if (vars.inflation.empty() || !uses_guide_width) {
    return;
  }

  // Check the two sides independently so a wall on one side does not consume
  // room on the other side. The final trajectory is checked on live maps.
  const double step = 0.5 * vars.inflation.resolution;
  const double max_offset = std::min(cfg_.guide_lateral_max_tolerance,
    vars.inflation.resolution * std::max(vars.inflation.width, vars.inflation.height));
  const size_t rings = static_cast<size_t>(std::ceil(max_offset / step));
  for (int i = 0; i < vars.piece_num; ++i) {
    const Eigen::Vector2d a = vars.waypoint_attractor.col(i).head<2>();
    const Eigen::Vector2d edge = vars.waypoint_attractor.col(i + 1).head<2>() - a;
    const double length = edge.norm();
    if (length <= 1.0e-6) {
      continue;
    }
    const Eigen::Vector2d normal(-edge.y() / length, edge.x() / length);
    const int intervals = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int side = 0; side < 2; ++side) {
      const double direction = side == 0 ? -1.0 : 1.0;
      for (size_t ring = 0; ring <= rings; ++ring) {
        const double offset = std::min(step * ring, max_offset);
        const Eigen::Vector2d shifted_start = a + direction * offset * normal;
        bool clear = true;
        for (int j = 0; j <= intervals; ++j) {
          const Eigen::Vector2d point = shifted_start + (static_cast<double>(j) / intervals) * edge;
          if (!vars.inflation.traversable(point)) {
            clear = false;
            break;
          }
        }
        if (!clear) {
          break;
        }
        vars.guide_side_offsets(side, i) = std::max(0.0, offset - vars.guide_lateral_tolerance);
      }
    }
  }
}

double MincoOptimizer::optimize(const std::vector<Eigen::Vector3d> & waypoints,
  const Eigen::Matrix3d & start_state,
  const Eigen::Matrix3d & end_state,
  const VecDf & local_magnitudes,
  geometry_utils::Trajectory & out_traj,
  double hard_velocity_limit)
{
  // 1. Setup the optimization problem
  out_traj.clear();
  last_iteration_count_ = 0;
  last_return_code_ = 0;
  last_objective_total_ = std::numeric_limits<double>::quiet_NaN();
  last_query_failure_count_ = 0;
  last_time_allocation_iterations_ = 0;
  last_peak_velocity_ = std::numeric_limits<double>::quiet_NaN();
  last_peak_acceleration_ = std::numeric_limits<double>::quiet_NaN();
  last_total_duration_ = std::numeric_limits<double>::quiet_NaN();
  last_failure_reason_ = FailureReason::NONE;
  opt_vars_.query_failure_count = 0;

  if (!std::isfinite(hard_velocity_limit) || hard_velocity_limit <= 0.0 ||
      hard_velocity_limit > cfg_.max_vel * (1.0 + 1.0e-6)) {
    last_return_code_ = lbfgs::LBFGSERR_INVALIDPARAMETERS;
    last_failure_reason_ = FailureReason::INVALID_INPUT;
    return INFINITY;
  }

  if (!setupProblemAndCheck(waypoints, start_state, end_state)) {
    cout << YELLOW << " -- [TrajOpt] Error in setup problem, force return." << RESET << endl;
    last_return_code_ = lbfgs::LBFGSERR_INVALIDPARAMETERS;
    last_failure_reason_ = FailureReason::INVALID_INPUT;
    return INFINITY;
  }
  if (std::max(start_state.col(1).norm(), end_state.col(1).norm()) >
          hard_velocity_limit * 1.001 ||
      std::max(start_state.col(2).norm(), end_state.col(2).norm()) > cfg_.max_acc * 1.001) {
    last_peak_velocity_ = std::max(start_state.col(1).norm(), end_state.col(1).norm());
    last_peak_acceleration_ = std::max(start_state.col(2).norm(), end_state.col(2).norm());
    last_failure_reason_ = FailureReason::DYNAMIC_FEASIBILITY;
    return INFINITY;
  }
  prepareInflationCost();

  // 2. Pack variables x = [tau, xi]
  // tau: unconstrained time variables
  // xi: stacked intermediate control points
  // [Theory] We optimize an unconstrained vector and map tau -> positive segment times.
  VecDf x(opt_vars_.dim_t + opt_vars_.dim_p);
  Eigen::Map<VecDf> tau(x.data(), opt_vars_.dim_t);
  Eigen::Map<VecDf> xi(x.data() + opt_vars_.dim_t, opt_vars_.dim_p);
  opt_vars_.local_magnitudes = local_magnitudes;
  opt_vars_.penalty_log.resize(6);  // Energy, pos, vel, acc, attract, time_barrier
  opt_vars_.penalty_log.setZero();

  if (opt_vars_.times.minCoeff() < 1e-3) {
    cout << YELLOW << " -- [TrajOpt] Error, the init times have zero, force return." << RESET << endl;
    cout << " -- Head PVA: " << endl;
    cout << opt_vars_.headPVA << endl;
    cout << " -- Tail PVA: " << endl;
    cout << opt_vars_.tailPVA << endl;
    cout << " -- Times: " << endl;
    cout << opt_vars_.times.transpose() << endl;
    last_failure_reason_ = FailureReason::INVALID_INPUT;
    return INFINITY;
  }

  gcopter::backwardMapTToTau(opt_vars_.times, tau);
  const auto guide_knots = opt_vars_.waypoint_attractor.middleCols(1, opt_vars_.piece_num - 1);
  const Mat3Df offsets = (opt_vars_.points - guide_knots) / opt_vars_.position_scale;
  xi = Eigen::Map<const VecDf>(offsets.data(), offsets.size());

  opt_vars_.iter_num = 0;
  double minCostFunctional = 0.0;

  // 3. Configure L-BFGS
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 3;
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.delta = cfg_.opt_accuracy;  // Gradient tolerance
  lbfgs_params.max_iterations = 256;

  // Restore per-solve weights after any previous penalty continuation.
  opt_vars_.penaltyWeights = cfg_.penaltyWeights;
  opt_vars_.magnitudeBounds = cfg_.magnitudeBounds;
  const double boundary_speed = std::max(start_state.col(1).norm(), end_state.col(1).norm());
  opt_vars_.continuation_velocity_limit = std::max(boundary_speed, 0.95 * hard_velocity_limit);
  opt_vars_.continuation_velocity_weight = 0.0;
  // A time stretch with fixed nonzero boundary derivatives is not a pure
  // rescaling: it can increase overshoot and produce a very long crawl.
  // Re-solve the original optimized variables with stronger dynamic penalties
  // before accepting such a result or asking the planner for a stop endpoint.
  geometry_utils::Trajectory feasible_fallback;
  VecDf fallback_times;
  Mat3Df fallback_points;
  double fallback_cost = 0.0;
  for (int attempt = 0; attempt < 3; ++attempt) {
    // 4. Run the optimizer
    // [Theory] L-BFGS is a limited-memory quasi-Newton method.
    int ret = lbfgs::lbfgs_optimize(x, minCostFunctional, &MincoOptimizer::costFunctional, nullptr,
                                    nullptr, &this->opt_vars_, lbfgs_params);
    last_iteration_count_ = opt_vars_.iter_num;
    last_return_code_ = ret;
    last_objective_total_ = minCostFunctional;
    last_query_failure_count_ = opt_vars_.query_failure_count;

    if (cfg_.print_optimizer_log) {
      cout << " -- [MincoOpt] Opt finish, with iter num: " << opt_vars_.iter_num << "\n";
      cout << "\tEnergy: " << opt_vars_.penalty_log(0) << endl;
      cout << "\tPos: " << opt_vars_.penalty_log(1) << endl;
      cout << "\tVel: " << opt_vars_.penalty_log(2) << endl;
      cout << "\tAcc: " << opt_vars_.penalty_log(3) << endl;
      cout << "\tAttract: " << opt_vars_.penalty_log(4) << endl;
      cout << "\tTime Barrier: " << opt_vars_.penalty_log(5) << endl;
      cout << "\tOptimized Time: " << opt_vars_.times.norm() << endl;
    }

    const bool line_search_stalled = ret == lbfgs::LBFGSERR_MINIMUMSTEP ||
      ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH;
    if (line_search_stalled && x.allFinite()) {
      // L-BFGS restores x to its last accepted iterate on line-search failure.
      // Recompute that iterate's objective before the independent hard checks.
      VecDf gradient(x.size());
      minCostFunctional = costFunctional(&opt_vars_, x, gradient);
      last_objective_total_ = minCostFunctional;
    }
    if (std::isfinite(minCostFunctional) &&
        (ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMITERATION || line_search_stalled)) {
      // 5. Decode the solution and build the output trajectory
      gcopter::forwardMapTauToT(tau, opt_vars_.times);
      opt_vars_.points = guide_knots +
        opt_vars_.position_scale * Eigen::Map<const Mat3Df>(xi.data(), 3, opt_vars_.piece_num - 1);

      // opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA,
      // opt_vars_.piece_num);
      opt_vars_.minco_solver_->setParameters(opt_vars_.points, opt_vars_.times);
      opt_vars_.minco_solver_->getTrajectory(out_traj);

      // Velocity and acceleration are soft integral penalties in the L-BFGS
      // objective. Enforce the configured limits on the continuous polynomial by
      // stretching all piece times uniformly and rebuilding MINCO with the same
      // points and boundary PVA. This keeps the geometric seed and boundary
      // conditions while removing narrow peaks that quadrature can miss.
      const double optimized_duration = out_traj.getTotalDuration();
      if (cfg_.print_optimizer_log) {
        std::cout << " -- [MincoOpt] Dynamic input attempt=" << attempt
                  << " duration=" << optimized_duration
                  << " peak_v=" << out_traj.getMaxVelRate()
                  << " peak_a=" << out_traj.getMaxAccRate() << std::endl;
      }
      const bool feasible = enforceDynamicFeasibility(out_traj, hard_velocity_limit);
      const bool moving_boundary =
          start_state.col(1).norm() > 1.0e-3 || end_state.col(1).norm() > 1.0e-3 ||
          start_state.col(2).norm() > 1.0e-3 || end_state.col(2).norm() > 1.0e-3;
      const bool excessive_stretch =
          moving_boundary && out_traj.getTotalDuration() > 1.15 * optimized_duration;
      if (attempt < 2 && moving_boundary &&
          (!feasible || excessive_stretch)) {
        if (feasible && (feasible_fallback.empty() ||
            out_traj.getTotalDuration() < feasible_fallback.getTotalDuration())) {
          feasible_fallback = out_traj;
          fallback_times = opt_vars_.times;
          fallback_points = opt_vars_.points;
          fallback_cost = minCostFunctional;
        }
        opt_vars_.penaltyWeights(ACC_IDX) *= 10.0;
        // Do not strengthen the speed penalty against an unreachable soft
        // turn/clearance target at the measured start. Re-solve against the
        // executable limit, with headroom for peaks between quadrature nodes.
        opt_vars_.continuation_velocity_weight =
          10.0 * std::max(opt_vars_.continuation_velocity_weight, cfg_.penaltyWeights(VEL_IDX));
        opt_vars_.magnitudeBounds(2) = 0.95 * cfg_.max_acc;
        // A short relative-objective plateau is not convergence after changing
        // penalty scales, especially with many tightly guided pieces.
        lbfgs_params.delta = std::min(cfg_.opt_accuracy, 1.0e-6);
        lbfgs_params.past = 10;
        continue;
      }
      if (!feasible) {
        out_traj.clear();
        last_failure_reason_ = last_total_duration_ > cfg_.max_trajectory_duration ?
          FailureReason::DURATION_LIMIT : FailureReason::DYNAMIC_FEASIBILITY;
      }
    } else {
      out_traj.clear();
      last_failure_reason_ = FailureReason::OPTIMIZATION;
    }
    break;
  }

  if (!feasible_fallback.empty() &&
      (out_traj.empty() || feasible_fallback.getTotalDuration() < out_traj.getTotalDuration())) {
    out_traj = std::move(feasible_fallback);
    opt_vars_.times = std::move(fallback_times);
    opt_vars_.points = std::move(fallback_points);
    minCostFunctional = fallback_cost;
    last_peak_velocity_ = out_traj.getMaxVelRate();
    last_peak_acceleration_ = out_traj.getMaxAccRate();
    last_failure_reason_ = FailureReason::NONE;
  }
  if (out_traj.empty()) {
    last_objective_total_ = std::numeric_limits<double>::infinity();
    return last_objective_total_;
  }

  last_total_duration_ = out_traj.getTotalDuration();
  last_objective_total_ = minCostFunctional;
  opt_vars_.init_ts = opt_vars_.times;
  opt_vars_.init_ps.clear();
  opt_vars_.init_ps.reserve(static_cast<size_t>(opt_vars_.points.cols()));
  for (int i = 0; i < opt_vars_.points.cols(); ++i) {
    opt_vars_.init_ps.emplace_back(opt_vars_.points.col(i));
  }
  opt_vars_.default_init = false;
  return minCostFunctional;
}

void MincoOptimizer::validateConfig(const Config & cfg)
{
  if (!std::isfinite(cfg.obstacle_slowdown_distance) || cfg.obstacle_slowdown_distance < 0.0 ||
    !std::isfinite(cfg.obstacle_max_velocity) || cfg.obstacle_max_velocity <= 0.0) {
    throw std::invalid_argument("Obstacle approach speed configuration is invalid");
  }
  if (!std::isfinite(cfg.terminal_tangent_lookahead) || cfg.terminal_tangent_lookahead <= 0.0) {
    throw std::invalid_argument("Terminal tangent lookahead must be finite and positive");
  }
  if (!std::isfinite(cfg.guide_lateral_weight) || cfg.guide_lateral_weight < 0.0 ||
    !std::isfinite(cfg.guide_lateral_tolerance) || cfg.guide_lateral_tolerance < 0.0 ||
    !std::isfinite(cfg.guide_lateral_max_tolerance) || cfg.guide_lateral_max_tolerance < 0.0 ||
    !std::isfinite(cfg.inflation_cost_weight) || cfg.inflation_cost_weight < 0.0) {
    throw std::invalid_argument("MincoOptimizer guide penalty must be finite and nonnegative");
  }
  if (cfg.time_allocation_iters <= 0) {
    throw std::invalid_argument("MincoOptimizer time_allocation_iters must be greater than zero");
  }
  if (!(std::isfinite(cfg.max_vel) && cfg.max_vel > 0.0 && std::isfinite(cfg.max_acc) &&
      cfg.max_acc > 0.0)) {
    throw std::invalid_argument("MincoOptimizer velocity and acceleration limits must be finite and positive");
  }
  if (!std::isfinite(cfg.terminal_velocity_ratio) ||
      cfg.terminal_velocity_ratio <= 0.0 || cfg.terminal_velocity_ratio > 1.0) {
    throw std::invalid_argument(
      "MincoOptimizer terminal_velocity_ratio must be finite and in (0, 1]");
  }
  if (!std::isfinite(cfg.max_trajectory_duration) ||
      cfg.max_trajectory_duration <= 0.0) {
    throw std::invalid_argument(
      "MincoOptimizer max_trajectory_duration must be finite and positive");
  }
}

bool MincoOptimizer::enforceDynamicFeasibility(
  geometry_utils::Trajectory & trajectory, double velocity_limit)
{
  constexpr double kRelativeTolerance = 1.0e-3;
  // Polynomial-extrema root finding is conservative near a repeated root and
  // can plateau about 1-1.5% above the analytic limit after time scaling. Keep
  // the strict target for every retiming attempt, then accept only this bounded
  // numerical residue after all attempts are exhausted. Gross violations and
  // non-finite extrema still fail closed here, and the complete position/yaw
  // trajectory is validated again before publish.
  constexpr double kFinalNumericalTolerance = 2.0e-2;
  constexpr double kScaleMargin = 1.02;
  constexpr double kMaxScalePerIteration = 4.0;
  constexpr double kMaxPieceDuration = 1.0e3;

  const double acceleration_limit = cfg_.max_acc;
  const double boundary_velocity =
    std::max(opt_vars_.headPVA.col(1).norm(), opt_vars_.tailPVA.col(1).norm());
  const double boundary_acceleration =
    std::max(opt_vars_.headPVA.col(2).norm(), opt_vars_.tailPVA.col(2).norm());
  last_peak_velocity_ = boundary_velocity;
  last_peak_acceleration_ = boundary_acceleration;
  if (!std::isfinite(boundary_velocity) || !std::isfinite(boundary_acceleration) ||
      boundary_velocity > velocity_limit * (1.0 + kRelativeTolerance) ||
      boundary_acceleration > acceleration_limit * (1.0 + kRelativeTolerance)) {
    return false;
  }

  const bool moving_boundary = boundary_velocity > 1.0e-3 || boundary_acceleration > 1.0e-3;
  double previous_scale_requirement = std::numeric_limits<double>::infinity();
  for (int iteration = 0; iteration <= cfg_.time_allocation_iters; ++iteration) {
    last_total_duration_ = trajectory.getTotalDuration();
    // Fixed nonzero boundary velocities do not decrease under uniform time
    // scaling. Stop rebuilding once the duration budget is exhausted; another
    // penalty solve or a freshly checked cached tail is the bounded fallback.
    if (!std::isfinite(last_total_duration_) ||
        last_total_duration_ > cfg_.max_trajectory_duration) {
      return false;
    }
    last_peak_velocity_ = trajectory.getMaxVelRate();
    last_peak_acceleration_ = trajectory.getMaxAccRate();
    if (!std::isfinite(last_peak_velocity_) || !std::isfinite(last_peak_acceleration_)) {
      return false;
    }

    const double velocity_ratio = last_peak_velocity_ / velocity_limit;
    const double acceleration_ratio = last_peak_acceleration_ / acceleration_limit;
    if (velocity_ratio <= 1.0 + kRelativeTolerance &&
        acceleration_ratio <= 1.0 + kRelativeTolerance) {
      return true;
    }
    const bool boundary_at_limit = boundary_velocity >= 0.95 * velocity_limit ||
      boundary_acceleration >= 0.95 * acceleration_limit;
    if (boundary_at_limit && velocity_ratio <= 1.0 + kFinalNumericalTolerance &&
        acceleration_ratio <= 1.0 + kFinalNumericalTolerance) {
      // The fixed boundary cannot be slowed by stretching. Use the same final
      // numerical tolerance here before repeated rebuilding distorts the path.
      return true;
    }
    if (iteration == cfg_.time_allocation_iters) {
      return velocity_ratio <= 1.0 + kFinalNumericalTolerance &&
             acceleration_ratio <= 1.0 + kFinalNumericalTolerance;
    }

    const double scale_from_velocity = velocity_ratio;
    const double scale_from_acceleration = std::sqrt(acceleration_ratio);
    const double scale_requirement = std::max(scale_from_velocity, scale_from_acceleration);
    if (moving_boundary && scale_requirement >= previous_scale_requirement) {
      // With fixed boundary PVA, stretching can increase the next peak.
      // Let the solver redistribute piece times instead of stretching again.
      return velocity_ratio <= 1.0 + kFinalNumericalTolerance &&
             acceleration_ratio <= 1.0 + kFinalNumericalTolerance;
    }
    previous_scale_requirement = scale_requirement;
    const double time_scale = std::clamp(
      scale_requirement * kScaleMargin,
      1.0 + kRelativeTolerance,
      moving_boundary ? 1.15 : kMaxScalePerIteration);
    if (!std::isfinite(time_scale) || time_scale <= 1.0) {
      return false;
    }

    opt_vars_.times *= time_scale;
    if (!opt_vars_.times.allFinite() || opt_vars_.times.minCoeff() <= 0.0 ||
        opt_vars_.times.maxCoeff() > kMaxPieceDuration) {
      return false;
    }

    opt_vars_.minco_solver_->setParameters(opt_vars_.points, opt_vars_.times);
    opt_vars_.minco_solver_->getTrajectory(trajectory);
    ++last_time_allocation_iterations_;
  }

  return false;
}

double MincoOptimizer::costFunctional(void * ptr, const VecDf & x, VecDf & g)
{
  OptVars & opt_vars_ = *(static_cast<OptVars *>(ptr));
  const auto & dim_t = opt_vars_.dim_t;
  const auto & dim_p = opt_vars_.dim_p;
  const auto & rho = opt_vars_.rho;
  const auto & minco_solver_ = opt_vars_.minco_solver_;

  opt_vars_.iter_num++;

  // 1. Unpack variables and gradient buffers
  const Eigen::Map<const VecDf> tau(x.data(), dim_t);
  const Eigen::Map<const VecDf> xi(x.data() + dim_t, dim_p);
  Eigen::Map<VecDf> grad_tau(g.data(), dim_t);
  Eigen::Map<VecDf> grad_points(g.data() + dim_t, dim_p);

  // 2. Map tau -> segment times
  VecDf times;
  gcopter::forwardMapTauToT(tau, times);

  Mat3Df points;
  if (dim_p > 0) {
    points = opt_vars_.waypoint_attractor.middleCols(1, dim_p / 3) +
      opt_vars_.position_scale * Eigen::Map<const Mat3Df>(xi.data(), 3, dim_p / 3);
  } else {
    points.resize(3, 0);
  }

  // 3. Update MINCO polynomial parameters
  minco_solver_->setParameters(points, times);

  // 4. Energy term (MINCO internal objective)
  double cost = 0.0;
  MatD3f partialGradByCoeffs(6 * times.size(), 3);
  VecDf partialGradByTimes(times.size());
  partialGradByCoeffs.setZero();
  partialGradByTimes.setZero();
  minco_solver_->getEnergy(cost);
  minco_solver_->getEnergyPartialGradByCoeffs(partialGradByCoeffs);
  minco_solver_->getEnergyPartialGradByTimes(partialGradByTimes);
  opt_vars_.penalty_log(0) = cost;

  // 5. Penalty terms (ESDF / vel / acc / attract)
  constraintsFunctional(opt_vars_, times, minco_solver_->getCoeffs(),
    cost, partialGradByTimes, partialGradByCoeffs);

  // 6. Propagate gradients to points and times
  Mat3Df gradByPoints;
  VecDf gradByTimes;
  minco_solver_->propogateGrad(partialGradByCoeffs, partialGradByTimes, gradByPoints, gradByTimes);

  // 7. Time regularization
  cost += rho * times.sum();
  gradByTimes.array() += rho;

  // 7.5 Kinematic Time Barrier
  computeTimeBarrier(opt_vars_, times, cost, gradByTimes, opt_vars_.penalty_log);

  // 8. Backprop time gradient (T -> tau)
  gcopter::propagateGradientTToTau(tau, gradByTimes, grad_tau);

  if (dim_p > 0) {
    grad_points = opt_vars_.position_scale *
      Eigen::Map<VecDf>(gradByPoints.data(), gradByPoints.size());
  }

  return cost;
}

void MincoOptimizer::computeTimeBarrier(const OptVars & opt_vars,
  const VecDf & times,
  double & cost,
  VecDf & gradByTimes,
  VecDf & penalty_log)
{
  const int N = static_cast<int>(times.size());
  const auto & w_barrier = opt_vars.penaltyWeights(4);  // Time barrier weight
  const bool has_init_ps = (opt_vars.init_ps.size() == static_cast<size_t>(std::max(0, N - 1)));

  for (int i = 0; i < N; ++i) {
    const double t_i = times(i);
    const double local_vmax = opt_vars.local_magnitudes(i);
    const double vmax_safe = std::max(1e-3, local_vmax);
    Eigen::Vector3d p_start = opt_vars.headPVA.col(0);
    Eigen::Vector3d p_end = opt_vars.tailPVA.col(0);

    if (i > 0) {
      p_start =
        has_init_ps ? opt_vars.init_ps[static_cast<size_t>(i - 1)] : opt_vars.waypoint_attractor.col(i);
    }
    if (i < N - 1) {
      p_end =
        has_init_ps ? opt_vars.init_ps[static_cast<size_t>(i)] : opt_vars.waypoint_attractor.col(i + 1);
    }

    const double dist = (p_end - p_start).norm();
    const double t_min = dist / vmax_safe;

    if (t_i < t_min) {
      const double violation = t_min - t_i;
      const double violation2 = violation * violation;
      cost += w_barrier * violation2 * violation;
      gradByTimes(i) += -3.0 * w_barrier * violation2;
      penalty_log(5) = violation;
    }
  }
}

void MincoOptimizer::constraintsFunctional(OptVars & vars, const VecDf & T,
  const MatD3f & coeffs,
  double & cost,
  VecDf & partialGradByTimes,
  MatD3f & partialGradByCoeffs)
{
  const auto & waypoint_attractor = vars.waypoint_attractor;
  const auto & map = vars.map;
  const auto smooth_eps = vars.smooth_eps;
  const int integral_res = vars.integral_res;
  const double safe_dist = vars.magnitudeBounds[0];
  const double amax = vars.magnitudeBounds[2];
  const double amaxSqr = amax * amax;
  const double weightPos = vars.penaltyWeights[0];
  const double weightVel = vars.penaltyWeights[1];
  const double weightAcc = vars.penaltyWeights[2];
  const double weightAtt = vars.penaltyWeights[3];
  const double guide_lateral_weight = vars.guide_lateral_weight;
  const bool use_inflation = vars.inflation_cost_weight > 0.0 && !vars.inflation.empty();
  const auto piece_num = T.size();
  const double initial_speed = vars.headPVA.col(1).norm();
  VecDf speed_limit_time_gradient = VecDf::Zero(piece_num);
  double elapsed_time = 0.0;

  // 2. Setup piece-wise sampling
  const double integralFrac = 1.0 / integral_res;
  VecDf max_pena(4);
  max_pena.setZero();

  // 3. Loop over pieces and samples
  for (int i = 0; i < piece_num; i++) {
    const Mat63f & c = coeffs.block<6, 3>(i * 6, 0);
    const auto & step = T(i) * integralFrac;
    const double local_vmax = vars.local_magnitudes(i);
    const Vec3f guide_start = waypoint_attractor.col(i);
    const Vec3f guide_edge = waypoint_attractor.col(i + 1) - guide_start;
    const double guide_edge_squared = guide_edge.squaredNorm();
    Vec3f guide_normal(-guide_edge.y(), guide_edge.x(), 0.0);
    if (guide_normal.squaredNorm() > 1.0e-12) {
      guide_normal.normalize();
    }
    for (int j = 0; j <= integral_res; j++) {
      double s1 = j * step;
      double s2 = s1 * s1;
      double s3 = s2 * s1;
      double s4 = s2 * s2;
      double s5 = s4 * s1;
      Vec6f beta0, beta1, beta2, beta3, beta4;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      // beta4 << 0.0, 0.0, 0.0, 0., 0.0, 120.0;

      const Vec3f pos = c.transpose() * beta0;
      const Vec3f vel = c.transpose() * beta1;
      const Vec3f acc = c.transpose() * beta2;
      const Vec3f jer = c.transpose() * beta3;

      double tmp_cost{0.0};
      double speed_limit_sample_gradient = 0.0;
      Vec3f gradPos{0.0, 0.0, 0.0}, gradVel{0.0, 0.0, 0.0}, gradAcc{0.0, 0.0, 0.0};

      // For position cost
      if (weightPos > 0.0 && map) {
        double esdf_dist = -safe_dist;
        Eigen::Vector3d esdf_grad = Eigen::Vector3d::Zero();
        const auto query = map->query(pos.cast<double>());
        if (query.ok) {
          esdf_dist = query.distance;
          esdf_grad = query.gradient;
        } else {
          ++vars.query_failure_count;
        }

        const double violaPos = safe_dist - esdf_dist;
        double violaPosPena, violaPosPenaD;
        if (gcopter::smoothedL1(violaPos, smooth_eps, violaPosPena, violaPosPenaD)) {
          // d(violaPos)/dpos = -grad(dist)
          gradPos += (-weightPos * violaPosPenaD) * esdf_grad.cast<double>();
          tmp_cost += weightPos * violaPosPena;
          if (violaPos > max_pena(POS_IDX))
            max_pena(POS_IDX) = violaPos;
        }
      }

      if (use_inflation) {
        Eigen::Vector2d cost_gradient;
        const double weighted_cost =
          vars.inflation_cost_weight * vars.inflation.sample(pos.head<2>(), cost_gradient);
        // Integrate cost over distance so it affects where the robot travels.
        // Regularize the speed norm at stops for a finite analytic gradient.
        constexpr double speed_epsilon = 1.0e-4;
        const double speed_norm = std::sqrt(vel.squaredNorm() + speed_epsilon * speed_epsilon);
        const double distance_rate = speed_norm - speed_epsilon;
        gradPos.head<2>() += vars.inflation_cost_weight * distance_rate * cost_gradient;
        gradVel += (weighted_cost / speed_norm) * vel;
        tmp_cost += weighted_cost * distance_rate;
      }

      // For velocity cost
      const double speed_squared = vel.squaredNorm();
      // The measured boundary can enter faster than the next piece's soft
      // target. Penalize speeding above the reachable braking profile.
      const double braking_speed = initial_speed - amax * (elapsed_time + s1);
      const double preferred_speed = std::max(local_vmax, braking_speed);
      const double violaVel = speed_squared - preferred_speed * preferred_speed;
      double violaVelPena, violaVelPenaD;
      if (weightVel > 0 && gcopter::smoothedL1(violaVel, smooth_eps, violaVelPena, violaVelPenaD)) {
        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
        tmp_cost += weightVel * violaVelPena;
        if (braking_speed > local_vmax) {
          speed_limit_sample_gradient = 2.0 * weightVel * violaVelPenaD * amax * braking_speed;
        }
        if (violaVel > max_pena(VEL_IDX))
          max_pena(VEL_IDX) = violaVel;
      }
      const double continuation_violation =
        speed_squared - vars.continuation_velocity_limit * vars.continuation_velocity_limit;
      if (vars.continuation_velocity_weight > 0.0 &&
          gcopter::smoothedL1(continuation_violation, smooth_eps, violaVelPena, violaVelPenaD)) {
        gradVel += 2.0 * vars.continuation_velocity_weight * violaVelPenaD * vel;
        tmp_cost += vars.continuation_velocity_weight * violaVelPena;
      }

      // For acceleration cost
      const auto & violaAcc = acc.squaredNorm() - amaxSqr;
      double violaAccPena, violaAccPenaD;
      if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smooth_eps, violaAccPena, violaAccPenaD)) {
        gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
        tmp_cost += weightAcc * violaAccPena;
        if (violaAcc > max_pena(ACC_IDX))
          max_pena(ACC_IDX) = violaAcc;
      }

      // For attract point cost
      // Penalize lateral bowing along the whole guide segment. Knot-only
      // attraction leaves the middle of a polynomial free to cut a narrow gate.
      if (guide_lateral_weight > 0.0) {
        const double fraction = guide_edge_squared > 1.0e-12 ?
          std::clamp((pos - guide_start).dot(guide_edge) / guide_edge_squared, 0.0, 1.0) : 0.0;
        Vec3f lateral = pos - (guide_start + fraction * guide_edge);
        const double allowed_lateral = std::clamp(lateral.dot(guide_normal),
          -vars.guide_side_offsets(0, i), vars.guide_side_offsets(1, i));
        lateral -= allowed_lateral * guide_normal;
        const double deviation = lateral.norm();
        if (deviation > vars.guide_lateral_tolerance && deviation > 1.0e-9) {
          const double excess = deviation - vars.guide_lateral_tolerance;
          gradPos += (2.0 * guide_lateral_weight * excess / deviation) * lateral;
          tmp_cost += guide_lateral_weight * excess * excess;
        }
      }

      if (weightAtt > 0.0) {
        const auto is_end = ((j == integral_res) && (i != piece_num - 1));
        const auto idx = is_end ? i + 1 : i;
        if (is_end) {
          Vec3f p_a = pos - waypoint_attractor.col(idx);
          const auto & violaAtt = p_a.squaredNorm() - 0.1 * 0.1;  // Dead zone: 0.1m
          double violaAttPena, violaAttPenaD;
          if (violaAtt > max_pena(ATT_IDX))
            max_pena(ATT_IDX) = violaAtt;
          if (gcopter::smoothedL1(violaAtt, smooth_eps, violaAttPena, violaAttPenaD)) {
            Vec3f gradAtt = weightAtt * violaAttPenaD * 2.0 * p_a;
            partialGradByCoeffs.block<6, 3>(i * 6, 0) += beta0 * gradAtt.transpose();
            cost += weightAtt * violaAttPena;
            if (is_end) {
              partialGradByTimes(i) += gradAtt.dot(vel);
            }
          }
        }
      }

      const auto node = (j == 0 || j == integral_res) ? 0.5 : 1.0;
      const double alpha = j * integralFrac;
      partialGradByCoeffs.block<6, 3>(i * 6, 0) +=
        (beta0 * gradPos.transpose() + beta1 * gradVel.transpose() + beta2 * gradAcc.transpose()) * node *
        step;
      partialGradByTimes(i) +=
        (gradPos.dot(vel) + gradVel.dot(acc) + gradAcc.dot(jer) +
        speed_limit_sample_gradient) * alpha * node * step +
        node * integralFrac * tmp_cost;
      speed_limit_time_gradient(i) += node * step * speed_limit_sample_gradient;
      cost += node * step * tmp_cost;
    }
    elapsed_time += T(i);
  }

  // Every earlier piece shifts the absolute time of the subsequent speed samples.
  double later_speed_limit_gradient = 0.0;
  for (Eigen::Index i = piece_num - 1; i >= 0; --i) {
    partialGradByTimes(i) += later_speed_limit_gradient;
    later_speed_limit_gradient += speed_limit_time_gradient(i);
  }

  // 4. Save penalty logs
  vars.penalty_log(1) = max_pena(POS_IDX);
  vars.penalty_log(2) = max_pena(VEL_IDX);
  vars.penalty_log(3) = max_pena(ACC_IDX);
  vars.penalty_log(4) = max_pena(ATT_IDX);
}

bool MincoOptimizer::setupProblemAndCheck(const std::vector<Eigen::Vector3d> & waypoints,
  const Eigen::Matrix3d & start_state,
  const Eigen::Matrix3d & end_state)
{
  // 1. Decide the number of polynomial pieces
  int N = waypoints.size() - 1;
  if (N <= 0) {
    return false;
  }

  // 2. Allocate optimization context
  opt_vars_.piece_num = N;
  opt_vars_.headPVA = start_state;
  opt_vars_.tailPVA = end_state;
  opt_vars_.times.resize(N);
  opt_vars_.points.resize(3, N - 1);
  opt_vars_.waypoint_attractor.resize(3, N + 1);
  for (int i = 0; i < N - 1; ++i) {
    opt_vars_.waypoint_attractor.col(i + 1) = waypoints[i + 1];
  }
  opt_vars_.waypoint_attractor.col(0) = opt_vars_.headPVA.col(0);
  opt_vars_.waypoint_attractor.rightCols(1) = opt_vars_.tailPVA.col(0);
  // Express each knot as an offset in units of the typical guide spacing.
  // Metre-sized knot steps dominate duration updates on a densely sampled guide.
  opt_vars_.position_scale = std::clamp(
    (opt_vars_.waypoint_attractor.rightCols(N) - opt_vars_.waypoint_attractor.leftCols(N))
    .colwise().norm().mean(), 0.10, 0.50);

  // 3. Set variable dimensions
  opt_vars_.dim_t = N;
  opt_vars_.dim_p = 3 * (N - 1);

  // 4. Allocate gradient buffers
  opt_vars_.gradByPoints.resize(3, N - 1);
  opt_vars_.gradByTimes.resize(N);
  opt_vars_.partialGradByCoeffs.resize(6 * N, 3);
  opt_vars_.partialGradByTimes.resize(N);

  // 5. Initialize time and point guesses
  DefaultInit();

  // Apply warm-start (shifted hot start) if provided and sizes match.
  if (!opt_vars_.default_init) {
    const bool ts_ok = (opt_vars_.init_ts.size() == opt_vars_.times.size());
    const bool ps_ok = (opt_vars_.init_ps.size() == static_cast<size_t>(opt_vars_.points.cols()));
    if (ts_ok && ps_ok) {
      opt_vars_.times = opt_vars_.init_ts;
      for (int i = 0; i < opt_vars_.points.cols(); ++i) {
        opt_vars_.points.col(i) = opt_vars_.init_ps[static_cast<size_t>(i)];
      }
    } else {
      // Size mismatch: fallback to cold init.
      opt_vars_.default_init = true;
    }
  }

  // 6. Set boundary conditions for MINCO
  opt_vars_.minco_solver_->setConditions(opt_vars_.headPVA, opt_vars_.tailPVA, N);
  return true;
}

void MincoOptimizer::DefaultInit()
{
  // 1. Extract segment distance computation logic.
  const auto computeDis = [this]() -> VecDf {
    return (opt_vars_.waypoint_attractor.leftCols(opt_vars_.piece_num) -
            opt_vars_.waypoint_attractor.rightCols(opt_vars_.piece_num))
      .colwise()
      .norm()
      .transpose();
  };
  const VecDf dis = computeDis();

  // 2. Allocate initial times with kinematic-aware startup / stopping compensation.
  const double max_vel = cfg_.max_vel;
  const double max_acc = cfg_.max_acc;
  const double cruise_speed = std::max(1e-3, max_vel);
  const double v_tail = opt_vars_.tailPVA.col(1).norm();
  const bool is_stopping = (v_tail < 0.1);

  opt_vars_.times.resize(opt_vars_.piece_num);
  for (int i = 0; i < opt_vars_.piece_num; ++i) {
    const double d = std::max(0.0, dis(i));
    double t = 0.1;
    if (i == 0) {
      // First segment: conservative startup time to avoid speed spike.
      const double t_acc = std::sqrt(std::max(0.0, 2.0 * d / max_acc));
      const double t_cons = d / std::max(1e-3, max_vel * 0.5);
      t = std::max(t_acc, t_cons);
    } else if (i == opt_vars_.piece_num - 1 && is_stopping) {
      // Last segment: add braking compensation for stop behavior.
      const double t_cruise = d / cruise_speed;
      const double t_brake = std::sqrt(std::max(0.0, 2.0 * d / max_acc));
      t = std::max(t_cruise, t_brake);
    } else {
      // Middle segments: keep original uniform-speed allocation.
      t = d / cruise_speed;
    }

    opt_vars_.times(i) = t;
  }
  opt_vars_.times = opt_vars_.times.cwiseMax(0.1);

  // 3. Initialize intermediate points from guide waypoints (unchanged).
  opt_vars_.points = opt_vars_.waypoint_attractor.block(0, 1, 3, opt_vars_.piece_num - 1);
}

void MincoOptimizer::setInitPsAndTs(const vec_Vec3f & init_ps, const VecDf & init_ts)
{
  // Store warm-start guesses. They will be applied after problem dimensions
  // are known (inside setupProblemAndCheck()).
  if (init_ts.size() == 0) {
    opt_vars_.default_init = true;
    opt_vars_.init_ps.clear();
    opt_vars_.init_ts.resize(0);
    return;
  }

  opt_vars_.default_init = false;
  opt_vars_.init_ps = init_ps;
  opt_vars_.init_ts = init_ts;
}
}  // namespace minco_planner
