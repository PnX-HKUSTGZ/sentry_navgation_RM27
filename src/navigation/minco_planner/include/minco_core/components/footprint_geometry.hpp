#pragma once

#include "rog_map/map_query_interface.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <memory>

namespace minco_planner {

struct StaticOverlapPolicy {
  double max_ratio{0.0};
  double max_depth{0.0};
  bool enabled() const {return max_ratio > 0.0 && max_depth > 0.0;}
  bool valid() const {
    return std::isfinite(max_ratio) && max_ratio >= 0.0 && max_ratio < 1.0 &&
      std::isfinite(max_depth) && max_depth >= 0.0 && (max_ratio == 0.0 || max_depth > 0.0);
  }
};

double polygonArea(const std::vector<Eigen::Vector2d> & polygon);
double footprintCellOverlapArea(const std::vector<Eigen::Vector2d> & body,
  const Eigen::Vector2d & center, const Eigen::Vector2d & axis_x,
  const Eigen::Vector2d & axis_y);
std::vector<Eigen::Vector2d> insetFootprint(
  const std::vector<Eigen::Vector2d> & body, double depth);

// Shared separating-axis test against the original, complete grid square.
inline bool footprintIntersectsCell(const std::vector<Eigen::Vector2d> & body,
  const Eigen::Vector2d & center, const Eigen::Vector2d & axis_x,
  const Eigen::Vector2d & axis_y)
{
  if (body.size() < 3U) {return false;}
  const auto separated = [&](const Eigen::Vector2d & axis) {
    double lower = std::numeric_limits<double>::infinity(), upper = -lower;
    for (const auto & vertex : body) {
      lower = std::min(lower, axis.dot(vertex));
      upper = std::max(upper, axis.dot(vertex));
    }
    const double c = axis.dot(center);
    const double radius = 0.5 * (std::abs(axis.dot(axis_x)) + std::abs(axis.dot(axis_y)));
    return c + radius < lower - 1.0e-9 || c - radius > upper + 1.0e-9;
  };
  if (separated({-axis_x.y(), axis_x.x()}) || separated({-axis_y.y(), axis_y.x()})) {
    return false;
  }
  for (size_t i = 0; i < body.size(); ++i) {
    const Eigen::Vector2d edge = body[(i + 1U) % body.size()] - body[i];
    if (separated({-edge.y(), edge.x()})) {return false;}
  }
  return true;
}

struct StaticFootprintResult {
  bool valid{false};
  uint8_t cost{255U};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  double overlap_area{0.0};
  double overlap_ratio{0.0};
  bool depth_exceeded{false};
};

inline StaticFootprintResult checkStaticFootprint(
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const std::vector<Eigen::Vector2d> & footprint, const Eigen::Vector3d & position, double yaw,
  const StaticOverlapPolicy & policy = {})
{
  StaticFootprintResult result;
  if (!policy.valid() || !map || footprint.size() < 3U || !position.allFinite() || !std::isfinite(yaw) ||
    map->sizeX() < 2U || map->sizeY() < 2U) {return result;}
  double x0, y0, x1, y1, x2, y2;
  map->mapToWorld(0, 0, x0, y0);
  map->mapToWorld(1, 0, x1, y1);
  map->mapToWorld(0, 1, x2, y2);
  const Eigen::Vector2d origin(x0, y0), ax(x1-x0, y1-y0), ay(x2-x0, y2-y0);
  Eigen::Matrix2d basis;
  basis.col(0) = ax; basis.col(1) = ay;
  if (!basis.allFinite() || std::abs(basis.determinant()) < 1.0e-10) {return result;}
  const Eigen::Matrix2d inverse = basis.inverse();
  const Eigen::Rotation2Dd rotation(yaw);
  std::vector<Eigen::Vector2d> body;
  Eigen::Vector2d lower = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d upper = -lower;
  for (const auto & vertex : footprint) {
    if (!vertex.allFinite()) {return result;}
    body.push_back(position.head<2>() + rotation * vertex);
    const Eigen::Vector2d cell = inverse * (body.back() - origin);
    lower = lower.cwiseMin(cell); upper = upper.cwiseMax(cell);
  }
  if (lower.x() < -0.5 || lower.y() < -0.5 || upper.x() > map->sizeX()-0.5 ||
    upper.y() > map->sizeY()-0.5) {return result;}
  std::vector<Eigen::Vector3d> centers;
  for (int y = std::max(0, static_cast<int>(std::ceil(lower.y()-0.5)));
    y <= std::min(static_cast<int>(map->sizeY())-1, static_cast<int>(std::floor(upper.y()+0.5))); ++y) {
    for (int x = std::max(0, static_cast<int>(std::ceil(lower.x()-0.5)));
      x <= std::min(static_cast<int>(map->sizeX())-1, static_cast<int>(std::floor(upper.x()+0.5))); ++x) {
      const Eigen::Vector2d center = origin + x*ax + y*ay;
      if (footprintIntersectsCell(body, center, ax, ay)) {
        centers.emplace_back(center.x(), center.y(), position.z());
      }
    }
  }
  const auto values = map->queryBatch(centers);
  if (values.size() != centers.size() || values.empty()) {return result;}
  result.valid = true; result.cost = 0;
  std::vector<Eigen::Vector2d> occupied;
  for (size_t i = 0; i < values.size(); ++i) {
    if (!values[i].ok || !values[i].projected_cost_valid) {result.valid = false; return result;}
    if (values[i].projected_cost >= 253U) {
      result.cost = values[i].projected_cost; result.point = centers[i];
      // Tolerance applies only to original known-occupied PGM squares.
      if (!policy.enabled() || result.cost != 254U) {return result;}
      occupied.push_back(centers[i].head<2>());
      result.overlap_area += footprintCellOverlapArea(body, occupied.back(), ax, ay);
    }
  }
  if (!occupied.empty()) {
    const double body_area = polygonArea(body);
    if (body_area <= 1.0e-10) {result.valid = false; return result;}
    result.overlap_ratio = result.overlap_area / body_area;
    const auto core = insetFootprint(body, policy.max_depth);
    if (core.size() < 3U) {result.valid = false; return result;}
    for (const auto & center : occupied) {
      if (footprintIntersectsCell(core, center, ax, ay)) {
        result.depth_exceeded = true;
        result.point.head<2>() = center;
        break;
      }
    }
    if (!result.depth_exceeded && result.overlap_ratio <= policy.max_ratio + 1.0e-9) {
      result.cost = 0U;
    }
  }
  return result;
}

// A missing provenance record must never turn a fused obstacle into free space.
inline rog_map::QueryResult dynamicEvidenceOnly(rog_map::QueryResult result)
{
  if (!result.projection.valid) {
    result.ok = false;
    result.status = rog_map::QueryStatus::SNAPSHOT_INVALID;
    result.projected_cost_valid = true;
    result.projected_cost = 255U;
    return result;
  }
  result.projected_cost = result.projection.dynamic_cost;
  result.projected_cost_valid = true;
  result.projection.cost_source = result.projected_cost == 255U ?
    rog_map::ProjectedCostSource::UNKNOWN : rog_map::ProjectedCostSource::DYNAMIC_PROJECTION;
  if (result.projection.cost_cause == rog_map::ProjectedCostCause::PRIOR_MAP) {
    result.projection.cost_cause = rog_map::ProjectedCostCause::UNKNOWN;
  }
  return result;
}
}  // namespace minco_planner
