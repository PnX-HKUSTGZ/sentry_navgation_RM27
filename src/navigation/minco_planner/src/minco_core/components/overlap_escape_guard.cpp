#include "minco_core/components/overlap_escape_guard.hpp"
#include "minco_core/components/footprint_geometry.hpp"

#include <boost/geometry.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace minco_planner {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

Polygon polygon(const std::vector<Eigen::Vector2d> & points) {
  Polygon result;
  for (const auto & point : points) {
    bg::append(result.outer(), Point(point.x(), point.y()));
  }
  bg::correct(result);
  return result;
}

double overlap(const Polygon & a, const Polygon & b) {
  std::vector<Polygon> intersections;
  bg::intersection(a, b, intersections);
  double area = 0.0;
  for (const auto & intersection : intersections) {
    area += std::abs(bg::area(intersection));
  }
  return area;
}
}  // namespace

OverlapEscapeGuard::OverlapEscapeGuard(Config config) : config_(std::move(config)) {
  if (!config_.static_overlap.valid() || config_.footprint.size() < 3U || !std::isfinite(config_.max_speed) ||
    config_.max_speed <= 0.0 || !std::isfinite(config_.max_duration) ||
    config_.max_duration <= 0.0 || !std::isfinite(config_.max_distance) ||
    config_.max_distance <= 0.0 || !std::isfinite(config_.max_penetration) ||
    config_.max_penetration <= 0.0 || !std::isfinite(config_.max_overlap_ratio) ||
    config_.max_overlap_ratio <= 0.0 || config_.max_overlap_ratio >= 1.0 ||
    !std::isfinite(config_.min_overlap_ratio) || config_.min_overlap_ratio < 0.0 ||
    config_.min_overlap_ratio >= config_.max_overlap_ratio ||
    !std::isfinite(config_.tracking_tolerance) || config_.tracking_tolerance <= 0.0 ||
    !std::isfinite(config_.map_timeout) || config_.map_timeout <= 0.0 ||
    !std::isfinite(config_.future_tolerance) || config_.future_tolerance < 0.0)
  {
    throw std::invalid_argument("Invalid bounded overlap escape configuration");
  }
  for (const auto & point : config_.footprint) {
    if (!point.allFinite()) {
      throw std::invalid_argument("Nonfinite escape footprint");
    }
  }
  const auto body = polygon(config_.footprint);
  if (!bg::is_valid(body) || std::abs(bg::area(body)) < 1.0e-8) {
    throw std::invalid_argument("Invalid escape footprint polygon");
  }
}

std::shared_ptr<OverlapEscapeGuard::Context> OverlapEscapeGuard::begin(
  const Eigen::Vector3d & start, double yaw, const Eigen::Vector2d & velocity,
  const std::shared_ptr<rog_map::MapQueryInterface> & query, double now, std::string * reason,
  std::shared_ptr<rog_map::MapQueryInterface> static_query) const
{
  if (!start.allFinite() || !std::isfinite(yaw) || !velocity.allFinite() ||
    velocity.norm() <= 1.0e-6 || velocity.norm() > config_.max_speed + 1.0e-9)
  {
    if (reason) { *reason = "INVALID_ESCAPE_STATE"; }
    return nullptr;
  }
  const auto started = std::chrono::steady_clock::now();
  const double requested_duration =
    std::min(config_.max_duration, config_.max_distance / velocity.norm());
  std::vector<double> durations{requested_duration};
  // A full retreat can leave a narrow gate through the opposite wall.  Keep
  // trying shorter horizons until the vehicle has just cleared the overlap.
  // Dynamic-only escapes keep the configured horizon and existing behavior.
  if (static_query && requested_duration > 0.05) {
    const double step = std::max(0.05, requested_duration / 20.0);
    for (double candidate = requested_duration - step;
      candidate >= 0.05 - 1.0e-9; candidate -= step)
    {
      durations.push_back(std::max(0.05, candidate));
    }
  }

  std::string last_reason;
  for (const double duration : durations) {
    auto context = std::make_shared<Context>();
    context->start = start;
    context->yaw = yaw;
    context->velocity = velocity;
    context->duration = duration;
    context->distance = velocity.norm() * duration;
    context->started = started;
    context->static_query = static_query;
    if (!validate(*context, start, yaw, duration, query, now, true, &last_reason,
        static_cast<bool>(context->static_query))) {
      continue;
    }
    if (context->static_query) {
      context->static_context = std::make_shared<Context>();
      context->static_context->start = start;
      context->static_context->yaw = yaw;
      context->static_context->velocity = velocity;
      context->static_context->duration = duration;
      context->static_context->distance = velocity.norm() * duration;
      context->static_context->started = started;
      if (!validate(*context->static_context, start, yaw, duration,
          context->static_query, now, true, &last_reason, false, true)) {
        continue;
      }
      if (context->last_overlap_area + context->static_context->last_overlap_area >
        config_.max_overlap_ratio * std::abs(bg::area(polygon(config_.footprint)))) {
        last_reason = "COMBINED_OVERLAP_AREA_LIMIT";
        continue;
      }
    }
    if (reason) {*reason = "NONE";}
    return context;
  }
  if (reason) {*reason = last_reason.empty() ? "ESCAPE_HORIZON_UNSAFE" : last_reason;}
  return nullptr;
}

bool OverlapEscapeGuard::check(Context & context, const Eigen::Vector3d & actual,
  double yaw, const std::shared_ptr<rog_map::MapQueryInterface> & query, double now) const
{
  std::lock_guard<std::mutex> lock(context.mutex);
  if (!actual.allFinite() || !std::isfinite(yaw) ||
    std::abs(std::remainder(yaw - context.yaw, 2.0 * std::acos(-1.0))) > 0.03 ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() - context.started).count() >
    config_.max_duration + 0.50)
  {
    return false;
  }
  const Eigen::Vector2d direction = context.velocity.normalized();
  const Eigen::Vector2d offset = (actual - context.start).head<2>();
  const double progress = offset.dot(direction);
  const double length = context.distance > 0.0 ? context.distance :
    std::min(config_.max_distance, context.velocity.norm() * config_.max_duration);
  if (progress < -0.01 || progress > length + config_.tracking_tolerance ||
    (offset - progress * direction).norm() > config_.tracking_tolerance)
  {
    return false;
  }
  const double remaining = std::max(0.0, length - progress) / context.velocity.norm();
  if (!validate(context, actual, yaw, remaining, query, now, false, nullptr,
      static_cast<bool>(context.static_query))) {return false;}
  return !context.static_query || (context.static_context &&
    validate(*context.static_context, actual, yaw, remaining, context.static_query,
      now, false, nullptr, false, true));
}

bool OverlapEscapeGuard::validate(Context & context, const Eigen::Vector3d & start,
  double yaw, double duration, const std::shared_ptr<rog_map::MapQueryInterface> & query,
  double now, bool initialize, std::string * reason, bool dynamic_only, bool static_grid) const
{
  const auto reject = [reason](const char * value) {
    if (reason) { *reason = value; }
    return false;
  };
  if (!query || !std::isfinite(now) || now <= 0.0 || query->sizeX() < 2U || query->sizeY() < 2U) {
    return reject("INVALID_MAP_OR_TIME");
  }
  double x0, y0, x1, y1, x2, y2;
  query->mapToWorld(0U, 0U, x0, y0);
  query->mapToWorld(1U, 0U, x1, y1);
  query->mapToWorld(0U, 1U, x2, y2);
  const Eigen::Vector2d origin(x0, y0), axis_x(x1 - x0, y1 - y0), axis_y(x2 - x0, y2 - y0);
  Eigen::Matrix2d basis;
  basis.col(0) = axis_x;
  basis.col(1) = axis_y;
  if (!basis.allFinite() || std::abs(basis.determinant()) < 1.0e-10) {
    return reject("INVALID_GRID_TRANSFORM");
  }
  const double resolution = std::min(axis_x.norm(), axis_y.norm());
  const Eigen::Rotation2Dd rotation(yaw);
  const Eigen::Matrix2d inverse_basis = basis.inverse();
  std::vector<Eigen::Vector2d> offsets;
  Eigen::Vector2d lower = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d upper = -lower;
  for (const auto & vertex : config_.footprint) {
    offsets.push_back(rotation * vertex);
    for (double t : {0.0, duration}) {
      const Eigen::Vector2d cell = inverse_basis *
        (start.head<2>() + t * context.velocity + offsets.back() - origin);
      lower = lower.cwiseMin(cell);
      upper = upper.cwiseMax(cell);
    }
  }
  if (lower.x() < -0.5 || lower.y() < -0.5 ||
    upper.x() > query->sizeX() - 0.5 || upper.y() > query->sizeY() - 0.5)
  {
    return reject("FOOTPRINT_OUT_OF_MAP");
  }
  std::vector<Eigen::Vector3d> centers;
  std::vector<Polygon> cells;
  for (int y = std::max(0, static_cast<int>(std::ceil(lower.y() - 0.5)));
    y <= std::min(static_cast<int>(query->sizeY()) - 1, static_cast<int>(std::floor(upper.y() + 0.5))); ++y)
  {
    for (int x = std::max(0, static_cast<int>(std::ceil(lower.x() - 0.5)));
      x <= std::min(static_cast<int>(query->sizeX()) - 1, static_cast<int>(std::floor(upper.x() + 0.5))); ++x)
    {
      const Eigen::Vector2d center = origin + x * axis_x + y * axis_y;
      centers.emplace_back(center.x(), center.y(), start.z());
      cells.push_back(polygon({center - 0.5 * axis_x - 0.5 * axis_y,
        center + 0.5 * axis_x - 0.5 * axis_y, center + 0.5 * axis_x + 0.5 * axis_y,
        center - 0.5 * axis_x + 0.5 * axis_y}));
    }
  }
  const auto results = query->queryBatch(centers);
  if (results.size() != centers.size()) {
    return reject("INCOMPLETE_QUERY");
  }
  std::vector<double> previous_areas(centers.size(), 0.0);
  double initial_total = 0.0;
  std::vector<double> actual_areas(context.allowed_cells.size(), 0.0);
  const double step = std::min(0.05, 0.25 * resolution / context.velocity.norm());
  const int steps = std::max(1, static_cast<int>(std::ceil(duration / step)));
  for (int sample = 0; sample <= steps; ++sample) {
    const double t = duration * sample / steps;
    std::vector<Eigen::Vector2d> vertices;
    for (const auto & offset : offsets) {
      vertices.push_back(start.head<2>() + t * context.velocity + offset);
    }
    const Polygon body = polygon(vertices);
    std::vector<double> current_areas(centers.size(), 0.0);
    double total = 0.0;
    for (size_t index = 0; index < cells.size(); ++index) {
      if (!footprintIntersectsCell(vertices, centers[index].head<2>(), axis_x, axis_y)) {
        continue;
      }
      const double area = overlap(body, cells[index]);
      if (area <= 1.0e-9 && !static_grid) {
        continue;
      }
      const auto result = dynamic_only ? dynamicEvidenceOnly(results[index]) : results[index];
      const double age = now - result.snapshot_stamp;
      if (!result.ok || !result.projected_cost_valid || !std::isfinite(result.distance) ||
        (!static_grid && (!std::isfinite(age) || result.snapshot_stamp <= 0.0 ||
        age > config_.map_timeout || age < -config_.future_tolerance)))
      {
        return reject("INVALID_OR_STALE_QUERY");
      }
      if (result.projected_cost == 255U && (static_grid || !config_.allow_unknown_motion)) {
        return reject("UNKNOWN_FOOTPRINT");
      }
      if (result.projected_cost != 254U && result.projected_cost != 253U) {
        continue;
      }
      double penetration = -result.distance;
      if (static_grid) {
        unsigned int mx, my;
        if (!query->worldToMap(centers[index].x(), centers[index].y(), mx, my)) {
          return reject("STATIC_OUT_OF_MAP");
        }
        penetration = std::numeric_limits<double>::infinity();
        const int radius = static_cast<int>(std::ceil(config_.max_penetration / resolution)) + 1;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const int x = static_cast<int>(mx) + dx, y = static_cast<int>(my) + dy;
            if (x < 0 || y < 0 || x >= static_cast<int>(query->sizeX()) ||
              y >= static_cast<int>(query->sizeY()) || !query->isFree(x, y)) {continue;}
            const double distance = std::hypot(std::max(0.0, std::abs(dx)-0.5) * axis_x.norm(),
              std::max(0.0, std::abs(dy)-0.5) * axis_y.norm());
            penetration = std::min(penetration, distance);
          }
        }
      }
      if (penetration > config_.max_penetration) {
        return reject("PENETRATION_LIMIT");
      }
      if (sample == 0 && initialize) {
        context.allowed_cells.push_back(centers[index].head<2>());
        context.last_cell_areas.push_back(area);
        actual_areas.push_back(area);
      }
      const auto original = std::find_if(context.allowed_cells.begin(), context.allowed_cells.end(),
        [&](const Eigen::Vector2d & cell) {return (cell - centers[index].head<2>()).norm() < 1.0e-6;});
      if (original == context.allowed_cells.end()) {
        return reject("NEW_OBSTACLE_CELL");
      }
      if (sample > 0 && area > previous_areas[index] + 1.0e-7) {
        return reject("PREDICTED_OVERLAP_INCREASE");
      }
      const auto original_index = static_cast<size_t>(original - context.allowed_cells.begin());
      if (sample == 0) {
        if (!initialize && area > context.last_cell_areas[original_index] + 1.0e-6) {
          return reject("ACTUAL_OVERLAP_INCREASE");
        }
        actual_areas[original_index] = area;
      }
      current_areas[index] = area;
      total += area;
    }
    if (sample == 0) {
      initial_total = total;
      if (total > config_.max_overlap_ratio * std::abs(bg::area(body))) {
        return reject("OVERLAP_AREA_LIMIT");
      }
      if (!initialize && total > context.last_overlap_area + 1.0e-6) {
        return reject("ACTUAL_OVERLAP_INCREASE");
      }
    }
    if (sample == steps && total > 1.0e-7 && !static_grid) {
      return reject("ENDPOINT_STILL_OVERLAPPED");
    }
    if (static_grid && sample == steps) {
      const auto end = checkStaticFootprint(query, config_.footprint,
        start + Eigen::Vector3d(t * context.velocity.x(), t * context.velocity.y(), 0.0), yaw,
        config_.static_overlap);
      if (!end.valid || end.cost >= 253U) {return reject("STATIC_ENDPOINT_BLOCKED");}
    }
    previous_areas = std::move(current_areas);
  }
  context.last_overlap_area = initial_total;
  context.last_cell_areas = std::move(actual_areas);
  if (reason) { *reason = "NONE"; }
  return true;
}
}  // namespace minco_planner
