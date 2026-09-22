#include "minco_core/components/map_query_adapters.hpp"

#include "tf2/time.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace minco_planner
{

namespace
{

Eigen::Vector3d transformPoint(
  const geometry_msgs::msg::TransformStamped & tf,
  const Eigen::Vector3d & p)
{
  const auto & q_msg = tf.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  const Eigen::Vector3d t(
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  return q * p + t;
}

Eigen::Vector3d rotateVector(
  const geometry_msgs::msg::TransformStamped & tf,
  const Eigen::Vector3d & v)
{
  const auto & q_msg = tf.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  return q * v;
}

bool isLethalCost(const unsigned char cost)
{
  return cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
         cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

}  // namespace

Nav2CostmapQuery::Nav2CostmapQuery(nav2_costmap_2d::Costmap2D * costmap)
: costmap_(costmap)
{
}

bool Nav2CostmapQuery::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!costmap_) {
    return false;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return costmap_->worldToMap(wx, wy, mx, my);
}

void Nav2CostmapQuery::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  if (!costmap_) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  costmap_->mapToWorld(mx, my, wx, wy);
}

unsigned int Nav2CostmapQuery::sizeX() const
{
  if (!costmap_) {
    return 0U;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return costmap_->getSizeInCellsX();
}
unsigned int Nav2CostmapQuery::sizeY() const
{
  if (!costmap_) {
    return 0U;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return costmap_->getSizeInCellsY();
}
double Nav2CostmapQuery::resolution() const
{
  if (!costmap_) {
    return 0.0;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return costmap_->getResolution();
}
double Nav2CostmapQuery::originX() const
{
  if (!costmap_) {
    return 0.0;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return costmap_->getOriginX();
}
double Nav2CostmapQuery::originY() const
{
  if (!costmap_) {
    return 0.0;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return costmap_->getOriginY();
}

uint8_t Nav2CostmapQuery::value(unsigned int mx, unsigned int my) const
{
  if (!costmap_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY()) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  return costmap_->getCost(mx, my);
}

const unsigned char * Nav2CostmapQuery::values() const
{
  if (!costmap_) {
    return nullptr;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return costmap_->getCharMap();
}

bool Nav2CostmapQuery::copyValues(std::vector<unsigned char> & out) const
{
  if (!costmap_) {
    out.clear();
    return false;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  const size_t count = static_cast<size_t>(costmap_->getSizeInCellsX()) *
    static_cast<size_t>(costmap_->getSizeInCellsY());
  const auto * data = costmap_->getCharMap();
  if (!data || count == 0U) {
    out.clear();
    return false;
  }
  out.assign(data, data + count);
  return true;
}

bool Nav2CostmapQuery::isValid(unsigned int mx, unsigned int my) const
{
  if (!costmap_) {
    return false;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  return mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY();
}

bool Nav2CostmapQuery::isFree(unsigned int mx, unsigned int my) const
{
  if (!costmap_) {
    return false;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY()) {
    return false;
  }
  const auto cost = costmap_->getCost(mx, my);
  return cost != nav2_costmap_2d::NO_INFORMATION &&
         cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool Nav2CostmapQuery::evaluate(
  const Eigen::Vector3d & pos, double & dist,
  Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

rog_map::QueryResult Nav2CostmapQuery::query(const Eigen::Vector3d & pos) const
{
  const auto results = queryBatch({pos});
  return results.empty() ? rog_map::QueryResult{} : results.front();
}

std::vector<rog_map::QueryResult> Nav2CostmapQuery::queryBatch(
  const std::vector<Eigen::Vector3d> & positions) const
{
  std::vector<rog_map::QueryResult> results(positions.size());
  if (!costmap_) {
    for (auto & result : results) {
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
    }
    return results;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  for (size_t index = 0; index < positions.size(); ++index) {
    const auto & pos = positions[index];
    auto & result = results[index];
    if (!pos.allFinite()) {
      result.status = rog_map::QueryStatus::NONFINITE_INPUT;
      continue;
    }
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(pos.x(), pos.y(), mx, my)) {
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
      continue;
    }
    const auto cost = costmap_->getCost(mx, my);
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.projected_cost_valid = true;
    result.projected_cost = cost;
    result.distance = isLethalCost(cost) ? -1.0 : costmap_->getResolution();
    result.gradient = Eigen::Vector3d::Zero();
  }
  return results;
}

StaticObstacleClearanceQuery::StaticObstacleClearanceQuery(
  std::shared_ptr<rog_map::MapQueryInterface> base,
  double clearance_radius)
: StaticObstacleClearanceQuery(base, base, clearance_radius)
{
}

StaticObstacleClearanceQuery::StaticObstacleClearanceQuery(
  std::shared_ptr<rog_map::MapQueryInterface> base,
  std::shared_ptr<rog_map::MapQueryInterface> static_source,
  double clearance_radius)
: base_(std::move(base)),
  static_source_(std::move(static_source)),
  clearance_radius_(clearance_radius)
{
  if (!base_ || !static_source_) {
    throw std::invalid_argument(
            "StaticObstacleClearanceQuery requires base and static-source queries");
  }
  buildOverlay();
}

bool StaticObstacleClearanceQuery::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return base_ && base_->worldToMap(wx, wy, mx, my);
}

void StaticObstacleClearanceQuery::mapToWorld(
  unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  if (!base_) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  base_->mapToWorld(mx, my, wx, wy);
}

unsigned int StaticObstacleClearanceQuery::sizeX() const
{
  return base_ ? base_->sizeX() : 0U;
}

unsigned int StaticObstacleClearanceQuery::sizeY() const
{
  return base_ ? base_->sizeY() : 0U;
}

double StaticObstacleClearanceQuery::resolution() const
{
  return base_ ? base_->resolution() : 0.0;
}

double StaticObstacleClearanceQuery::originX() const
{
  return base_ ? base_->originX() : 0.0;
}

double StaticObstacleClearanceQuery::originY() const
{
  return base_ ? base_->originY() : 0.0;
}

uint8_t StaticObstacleClearanceQuery::combinedValue(
  unsigned int mx, unsigned int my, uint8_t base_cost) const
{
  const unsigned int nx = sizeX();
  if (mx >= nx || my >= sizeY()) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  const size_t index = static_cast<size_t>(my) * static_cast<size_t>(nx) + mx;
  if (index >= overlay_costs_.size()) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  // NO_INFORMATION (255) must stay unknown instead of being silently turned
  // into known occupied space by the static clearance overlay.
  return std::max(base_cost, overlay_costs_[index]);
}

uint8_t StaticObstacleClearanceQuery::value(unsigned int mx, unsigned int my) const
{
  if (!base_ || !base_->isValid(mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  return combinedValue(mx, my, base_->value(mx, my));
}

const unsigned char * StaticObstacleClearanceQuery::values() const
{
  std::lock_guard<std::mutex> lock(merged_values_mutex_);
  if (!copyValues(merged_values_)) {
    return nullptr;
  }
  return merged_values_.data();
}

bool StaticObstacleClearanceQuery::copyValues(std::vector<unsigned char> & out) const
{
  if (!base_ || !base_->copyValues(out) || out.size() != overlay_costs_.size()) {
    out.clear();
    return false;
  }
  for (size_t index = 0; index < out.size(); ++index) {
    out[index] = std::max(out[index], overlay_costs_[index]);
  }
  return true;
}

bool StaticObstacleClearanceQuery::isValid(unsigned int mx, unsigned int my) const
{
  return base_ && base_->isValid(mx, my) &&
         static_cast<size_t>(my) * static_cast<size_t>(sizeX()) + mx < overlay_costs_.size();
}

bool StaticObstacleClearanceQuery::isFree(unsigned int mx, unsigned int my) const
{
  if (!isValid(mx, my)) {
    return false;
  }
  const uint8_t cost = value(mx, my);
  return cost != nav2_costmap_2d::NO_INFORMATION &&
         cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool StaticObstacleClearanceQuery::evaluate(
  const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

rog_map::QueryResult StaticObstacleClearanceQuery::query(
  const Eigen::Vector3d & pos) const
{
  const auto results = queryBatch({pos});
  return results.empty() ? rog_map::QueryResult{} : results.front();
}

std::vector<rog_map::QueryResult> StaticObstacleClearanceQuery::queryBatch(
  const std::vector<Eigen::Vector3d> & positions) const
{
  if (!base_) {
    return std::vector<rog_map::QueryResult>(positions.size());
  }
  auto results = base_->queryBatch(positions);
  if (results.size() != positions.size()) {
    return std::vector<rog_map::QueryResult>(positions.size());
  }

  for (size_t index = 0; index < positions.size(); ++index) {
    auto & result = results[index];
    if (!result.ok) {
      continue;
    }
    unsigned int mx = 0U;
    unsigned int my = 0U;
    if (!base_->worldToMap(positions[index].x(), positions[index].y(), mx, my)) {
      result.ok = false;
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
      continue;
    }
    const uint8_t base_cost = result.projected_cost_valid ?
      result.projected_cost : base_->value(mx, my);
    const uint8_t cost = combinedValue(mx, my, base_cost);
    result.projected_cost_valid = true;
    result.projected_cost = cost;
    if (isLethalCost(cost)) {
      result.distance = -1.0;
      result.gradient = Eigen::Vector3d::Zero();
    }
  }
  return results;
}

void StaticObstacleClearanceQuery::buildOverlay()
{
  if (!std::isfinite(clearance_radius_) || clearance_radius_ < 0.0) {
    throw std::invalid_argument(
            "StaticObstacleClearanceQuery clearance radius must be finite and nonnegative");
  }

  const unsigned int nx = sizeX();
  const unsigned int ny = sizeY();
  const double map_resolution = resolution();
  if (nx == 0U || ny == 0U || !std::isfinite(map_resolution) || map_resolution <= 0.0) {
    throw std::invalid_argument("StaticObstacleClearanceQuery base geometry is invalid");
  }
  if (static_source_->sizeX() != nx || static_source_->sizeY() != ny ||
    std::abs(static_source_->resolution() - map_resolution) > 1.0e-9 ||
    std::abs(static_source_->originX() - originX()) > 1.0e-9 ||
    std::abs(static_source_->originY() - originY()) > 1.0e-9)
  {
    throw std::invalid_argument(
            "StaticObstacleClearanceQuery static-source geometry does not match the base query");
  }

  std::vector<unsigned char> static_values;
  const size_t cell_count = static_cast<size_t>(nx) * static_cast<size_t>(ny);
  if (!static_source_->copyValues(static_values) || static_values.size() != cell_count) {
    throw std::invalid_argument(
            "StaticObstacleClearanceQuery could not snapshot the static source map");
  }

  overlay_costs_.assign(cell_count, nav2_costmap_2d::FREE_SPACE);
  hardened_cell_count_ = 0U;
  unknown_boundary_guard_cell_count_ = 0U;
  std::vector<uint8_t> unknown_near_lethal(cell_count, 0U);
  const int clearance_cells = static_cast<int>(
    std::ceil(clearance_radius_ / map_resolution));
  for (unsigned int my = 0U; my < ny; ++my) {
    for (unsigned int mx = 0U; mx < nx; ++mx) {
      const size_t source_index = static_cast<size_t>(my) * static_cast<size_t>(nx) + mx;
      // 253 is generated by Nav2 inflation and is already non-traversable, but
      // must not recursively expand the footprint mask. Only exact obstacle
      // cells are allowed to create this clearance overlay.
      if (static_values[source_index] != nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
        for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
          if (map_resolution * std::hypot(dx, dy) > clearance_radius_ + 1.0e-9) {
            continue;
          }
          const int target_x = static_cast<int>(mx) + dx;
          const int target_y = static_cast<int>(my) + dy;
          if (target_x < 0 || target_y < 0 || target_x >= static_cast<int>(nx) ||
            target_y >= static_cast<int>(ny))
          {
            continue;
          }
          const size_t target_index = static_cast<size_t>(target_y) *
            static_cast<size_t>(nx) + static_cast<size_t>(target_x);
          overlay_costs_[target_index] = nav2_costmap_2d::LETHAL_OBSTACLE;
          if (static_values[target_index] == nav2_costmap_2d::NO_INFORMATION) {
            unknown_near_lethal[target_index] = 1U;
          }
        }
      }
    }
  }

  // Unknown pixels at the anti-aliased edge of a static obstacle are still
  // intentionally unknown. Guard only the known-free side of that edge with a
  // high, but traversable, cost so global search prefers the middle of the
  // surveyed corridor instead of entering the unknown boundary. The second
  // dilation is bounded by the same footprint radius and never changes an
  // unknown cell into free space.
  constexpr uint8_t kUnknownBoundaryGuardCost =
    nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1U;
  for (unsigned int my = 0U; my < ny; ++my) {
    for (unsigned int mx = 0U; mx < nx; ++mx) {
      const size_t source_index = static_cast<size_t>(my) *
        static_cast<size_t>(nx) + mx;
      if (unknown_near_lethal[source_index] == 0U) {
        continue;
      }
      for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
        for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
          if (map_resolution * std::hypot(dx, dy) > clearance_radius_ + 1.0e-9) {
            continue;
          }
          const int target_x = static_cast<int>(mx) + dx;
          const int target_y = static_cast<int>(my) + dy;
          if (target_x < 0 || target_y < 0 || target_x >= static_cast<int>(nx) ||
            target_y >= static_cast<int>(ny))
          {
            continue;
          }
          const size_t target_index = static_cast<size_t>(target_y) *
            static_cast<size_t>(nx) + static_cast<size_t>(target_x);
          const uint8_t base_cost = static_values[target_index];
          if (base_cost == nav2_costmap_2d::NO_INFORMATION ||
            base_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
          {
            continue;
          }
          if (overlay_costs_[target_index] < kUnknownBoundaryGuardCost) {
            overlay_costs_[target_index] = kUnknownBoundaryGuardCost;
          }
        }
      }
    }
  }

  for (size_t index = 0; index < cell_count; ++index) {
    if (overlay_costs_[index] == nav2_costmap_2d::LETHAL_OBSTACLE &&
      static_values[index] != nav2_costmap_2d::LETHAL_OBSTACLE &&
      static_values[index] != nav2_costmap_2d::NO_INFORMATION)
    {
      ++hardened_cell_count_;
    }
    if (overlay_costs_[index] == kUnknownBoundaryGuardCost &&
      static_values[index] < kUnknownBoundaryGuardCost &&
      static_values[index] != nav2_costmap_2d::NO_INFORMATION &&
      static_values[index] != nav2_costmap_2d::LETHAL_OBSTACLE &&
      overlay_costs_[index] < nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      ++unknown_boundary_guard_cell_count_;
    }
  }
}

SurveyedGroundEdgeQuery::SurveyedGroundEdgeQuery(
  std::shared_ptr<rog_map::MapQueryInterface> base,
  const rog_map::PriorMapData & prior_map,
  const SurveyedGroundEdgeConfig & config)
: base_(std::move(base))
{
  if (!base_) {
    throw std::invalid_argument("SurveyedGroundEdgeQuery requires a base query");
  }
  buildOverlay(prior_map, config);
}

bool SurveyedGroundEdgeQuery::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return base_ && base_->worldToMap(wx, wy, mx, my);
}

void SurveyedGroundEdgeQuery::mapToWorld(
  unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  if (!base_) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  base_->mapToWorld(mx, my, wx, wy);
}

unsigned int SurveyedGroundEdgeQuery::sizeX() const
{
  return base_ ? base_->sizeX() : 0U;
}

unsigned int SurveyedGroundEdgeQuery::sizeY() const
{
  return base_ ? base_->sizeY() : 0U;
}

double SurveyedGroundEdgeQuery::resolution() const
{
  return base_ ? base_->resolution() : 0.0;
}

double SurveyedGroundEdgeQuery::originX() const
{
  return base_ ? base_->originX() : 0.0;
}

double SurveyedGroundEdgeQuery::originY() const
{
  return base_ ? base_->originY() : 0.0;
}

uint8_t SurveyedGroundEdgeQuery::combinedValue(
  unsigned int mx, unsigned int my, uint8_t base_cost) const
{
  const unsigned int nx = sizeX();
  if (mx >= nx || my >= sizeY()) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  const size_t index = static_cast<size_t>(my) * static_cast<size_t>(nx) + mx;
  if (index >= overlay_costs_.size()) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  return std::max(base_cost, overlay_costs_[index]);
}

uint8_t SurveyedGroundEdgeQuery::value(unsigned int mx, unsigned int my) const
{
  if (!base_ || !base_->isValid(mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  return combinedValue(mx, my, base_->value(mx, my));
}

const unsigned char * SurveyedGroundEdgeQuery::values() const
{
  std::lock_guard<std::mutex> lock(merged_values_mutex_);
  if (!copyValues(merged_values_)) {
    return nullptr;
  }
  return merged_values_.data();
}

bool SurveyedGroundEdgeQuery::copyValues(std::vector<unsigned char> & out) const
{
  if (!base_ || !base_->copyValues(out) || out.size() != overlay_costs_.size()) {
    out.clear();
    return false;
  }
  for (size_t index = 0; index < out.size(); ++index) {
    out[index] = std::max(out[index], overlay_costs_[index]);
  }
  return true;
}

bool SurveyedGroundEdgeQuery::isValid(unsigned int mx, unsigned int my) const
{
  return base_ && base_->isValid(mx, my) &&
         static_cast<size_t>(my) * static_cast<size_t>(sizeX()) + mx < overlay_costs_.size();
}

bool SurveyedGroundEdgeQuery::isFree(unsigned int mx, unsigned int my) const
{
  if (!isValid(mx, my)) {
    return false;
  }
  const uint8_t cost = value(mx, my);
  return cost != nav2_costmap_2d::NO_INFORMATION &&
         cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool SurveyedGroundEdgeQuery::evaluate(
  const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

rog_map::QueryResult SurveyedGroundEdgeQuery::query(const Eigen::Vector3d & pos) const
{
  const auto results = queryBatch({pos});
  return results.empty() ? rog_map::QueryResult{} : results.front();
}

std::vector<rog_map::QueryResult> SurveyedGroundEdgeQuery::queryBatch(
  const std::vector<Eigen::Vector3d> & positions) const
{
  if (!base_) {
    return std::vector<rog_map::QueryResult>(positions.size());
  }
  auto results = base_->queryBatch(positions);
  if (results.size() != positions.size()) {
    return std::vector<rog_map::QueryResult>(positions.size());
  }

  for (size_t index = 0; index < positions.size(); ++index) {
    auto & result = results[index];
    if (!result.ok) {
      continue;
    }
    unsigned int mx = 0U;
    unsigned int my = 0U;
    if (!base_->worldToMap(positions[index].x(), positions[index].y(), mx, my)) {
      result.ok = false;
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
      continue;
    }
    const uint8_t base_cost = result.projected_cost_valid ?
      result.projected_cost : base_->value(mx, my);
    const uint8_t cost = combinedValue(mx, my, base_cost);
    result.projected_cost_valid = true;
    result.projected_cost = cost;
    if (isLethalCost(cost)) {
      result.distance = -1.0;
      result.gradient = Eigen::Vector3d::Zero();
    }
  }
  return results;
}

void SurveyedGroundEdgeQuery::buildOverlay(
  const rog_map::PriorMapData & prior_map,
  const SurveyedGroundEdgeConfig & config)
{
  if (!prior_map.loaded || !prior_map.ground_elevation_loaded) {
    throw std::invalid_argument(
            "SurveyedGroundEdgeQuery requires a loaded ground-elevation prior");
  }
  if (!std::isfinite(config.max_step) || config.max_step < 0.0 ||
    !std::isfinite(config.max_slope_deg) || config.max_slope_deg < 0.0 ||
    config.max_slope_deg >= 90.0 || !std::isfinite(config.clearance_radius) ||
    config.clearance_radius < 0.0 || config.clearance_cost == 0U ||
    config.clearance_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    throw std::invalid_argument("SurveyedGroundEdgeQuery configuration is invalid");
  }

  const unsigned int nx = sizeX();
  const unsigned int ny = sizeY();
  const double map_resolution = resolution();
  if (nx == 0U || ny == 0U || !std::isfinite(map_resolution) || map_resolution <= 0.0) {
    throw std::invalid_argument("SurveyedGroundEdgeQuery base geometry is invalid");
  }
  const size_t cell_count = static_cast<size_t>(nx) * static_cast<size_t>(ny);
  overlay_costs_.assign(cell_count, nav2_costmap_2d::FREE_SPACE);
  std::vector<unsigned char> base_values;
  if (!base_->copyValues(base_values) || base_values.size() != cell_count) {
    throw std::invalid_argument("SurveyedGroundEdgeQuery could not snapshot the base map");
  }
  std::vector<uint8_t> support_known(cell_count, 0U);
  std::vector<double> support_height(cell_count, 0.0);

  const double map_origin_x = originX();
  const double map_origin_y = originY();
  for (unsigned int my = 0U; my < ny; ++my) {
    for (unsigned int mx = 0U; mx < nx; ++mx) {
      const size_t index = static_cast<size_t>(my) * static_cast<size_t>(nx) + mx;
      const double wx = map_origin_x + (static_cast<double>(mx) + 0.5) * map_resolution;
      const double wy = map_origin_y + (static_cast<double>(my) + 0.5) * map_resolution;
      support_known[index] = rog_map::priorMapGroundSupportAtMapPoint(
        prior_map, wx, wy, support_height[index]) ? 1U : 0U;
      const bool base_traversable =
        base_values[index] != nav2_costmap_2d::NO_INFORMATION &&
        base_values[index] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      if (support_known[index] == 0U && base_traversable) {
        overlay_costs_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
        ++unsupported_cell_count_;
      }
    }
  }

  constexpr double kPi = 3.14159265358979323846;
  const double max_slope = std::tan(config.max_slope_deg * kPi / 180.0);
  std::vector<uint8_t> edge_mask(cell_count, 0U);
  for (unsigned int my = 0U; my < ny; ++my) {
    for (unsigned int mx = 0U; mx < nx; ++mx) {
      const size_t index = static_cast<size_t>(my) * static_cast<size_t>(nx) + mx;
      const bool base_traversable =
        base_values[index] != nav2_costmap_2d::NO_INFORMATION &&
        base_values[index] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      if (support_known[index] == 0U || !base_traversable) {
        continue;
      }
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int neighbor_x = static_cast<int>(mx) + dx;
          const int neighbor_y = static_cast<int>(my) + dy;
          if (neighbor_x < 0 || neighbor_y < 0 ||
            neighbor_x >= static_cast<int>(nx) || neighbor_y >= static_cast<int>(ny))
          {
            continue;
          }
          const size_t neighbor_index = static_cast<size_t>(neighbor_y) *
            static_cast<size_t>(nx) + static_cast<size_t>(neighbor_x);
          const bool neighbor_traversable =
            base_values[neighbor_index] != nav2_costmap_2d::NO_INFORMATION &&
            base_values[neighbor_index] < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
          if (!neighbor_traversable) {
            continue;
          }
          if (support_known[neighbor_index] == 0U) {
            // The unsupported cell itself is already lethal. Mark the last
            // supported cell as an edge so the configured footprint clearance
            // is also reserved on the drivable side of the boundary.
            edge_mask[index] = 1U;
            continue;
          }
          // Height discontinuities need each supported pair only once. The
          // supported-to-unsupported case above intentionally checks all eight
          // directions because the unsupported cell cannot mark the pair later.
          if (dx < 0 || (dx == 0 && dy < 0)) {
            continue;
          }
          const double planar_distance = map_resolution * std::hypot(dx, dy);
          const double allowed_delta = std::max(config.max_step, max_slope * planar_distance);
          if (std::abs(support_height[index] - support_height[neighbor_index]) >
            allowed_delta + 1.0e-6)
          {
            edge_mask[index] = 1U;
            edge_mask[neighbor_index] = 1U;
          }
        }
      }
    }
  }

  const int clearance_cells = static_cast<int>(
    std::ceil(config.clearance_radius / map_resolution));
  for (unsigned int my = 0U; my < ny; ++my) {
    for (unsigned int mx = 0U; mx < nx; ++mx) {
      const size_t index = static_cast<size_t>(my) * static_cast<size_t>(nx) + mx;
      if (edge_mask[index] == 0U) {
        continue;
      }
      overlay_costs_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
      ++edge_cell_count_;
      for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
        for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
          if (map_resolution * std::hypot(dx, dy) > config.clearance_radius + 1.0e-9) {
            continue;
          }
          const int clearance_x = static_cast<int>(mx) + dx;
          const int clearance_y = static_cast<int>(my) + dy;
          if (clearance_x < 0 || clearance_y < 0 ||
            clearance_x >= static_cast<int>(nx) || clearance_y >= static_cast<int>(ny))
          {
            continue;
          }
          const size_t clearance_index = static_cast<size_t>(clearance_y) *
            static_cast<size_t>(nx) + static_cast<size_t>(clearance_x);
          if (edge_mask[clearance_index] == 0U) {
            const double edge_distance = map_resolution * std::hypot(dx, dy);
            const uint8_t clearance_value =
              edge_distance <= config.lethal_clearance_radius + 1.0e-9
              ? nav2_costmap_2d::LETHAL_OBSTACLE
              : config.clearance_cost;
            overlay_costs_[clearance_index] =
              std::max(overlay_costs_[clearance_index], clearance_value);
          }
        }
      }
    }
  }
  clearance_cell_count_ = static_cast<size_t>(std::count(
    overlay_costs_.begin(), overlay_costs_.end(), config.clearance_cost));
}

FrameAwareRogQuery::FrameAwareRogQuery(
  std::shared_ptr<rog_map::MapQueryInterface> raw,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::string planning_frame,
  std::string rog_frame,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
: raw_(std::move(raw)), tf_(std::move(tf)), planning_frame_(std::move(planning_frame)),
  rog_frame_(std::move(rog_frame)), logger_(logger), clock_(std::move(clock))
{
}

bool FrameAwareRogQuery::worldToMap(
  double wx, double wy, unsigned int & mx,
  unsigned int & my) const
{
  if (!raw_) {
    return false;
  }
  Eigen::Vector3d p(wx, wy, 0.0);
  if (!transformPlanningToRog(p, p)) {
    return false;
  }
  return raw_->worldToMap(p.x(), p.y(), mx, my);
}

void FrameAwareRogQuery::mapToWorld(
  unsigned int mx, unsigned int my, double & wx,
  double & wy) const
{
  wx = 0.0;
  wy = 0.0;
  if (!raw_) {
    return;
  }
  double rwx = 0.0;
  double rwy = 0.0;
  raw_->mapToWorld(mx, my, rwx, rwy);
  Eigen::Vector3d p(rwx, rwy, 0.0);
  if (!transformRogToPlanning(p, p)) {
    return;
  }
  wx = p.x();
  wy = p.y();
}

unsigned int FrameAwareRogQuery::sizeX() const
{
  return raw_ ? raw_->sizeX() : 0U;
}
unsigned int FrameAwareRogQuery::sizeY() const
{
  return raw_ ? raw_->sizeY() : 0U;
}
double FrameAwareRogQuery::resolution() const
{
  return raw_ ? raw_->resolution() : 0.0;
}
double FrameAwareRogQuery::originX() const
{
  return raw_ ? raw_->originX() : 0.0;
}
double FrameAwareRogQuery::originY() const
{
  return raw_ ? raw_->originY() : 0.0;
}

uint8_t FrameAwareRogQuery::value(unsigned int mx, unsigned int my) const
{
  return raw_ ? raw_->value(mx, my) : nav2_costmap_2d::LETHAL_OBSTACLE;
}

const unsigned char * FrameAwareRogQuery::values() const
{
  return raw_ ? raw_->values() : nullptr;
}

bool FrameAwareRogQuery::copyValues(std::vector<unsigned char> & out) const
{
  return raw_ && raw_->copyValues(out);
}

bool FrameAwareRogQuery::isValid(unsigned int mx, unsigned int my) const
{
  return raw_ && raw_->isValid(mx, my);
}

bool FrameAwareRogQuery::isFree(unsigned int mx, unsigned int my) const
{
  return raw_ && raw_->isFree(mx, my);
}

bool FrameAwareRogQuery::evaluate(
  const Eigen::Vector3d & pos, double & dist,
  Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

rog_map::QueryResult FrameAwareRogQuery::query(const Eigen::Vector3d & pos) const
{
  const auto results = queryBatch({pos});
  return results.empty() ? rog_map::QueryResult{} : results.front();
}

std::vector<rog_map::QueryResult> FrameAwareRogQuery::queryBatch(
  const std::vector<Eigen::Vector3d> & positions) const
{
  std::vector<rog_map::QueryResult> failures(positions.size());
  if (!raw_) {
    return failures;
  }

  if (planning_frame_ == rog_frame_) {
    return raw_->queryBatch(positions);
  }
  if (!tf_) {
    for (auto & failure : failures) {
      failure.status = rog_map::QueryStatus::TF_FAILED;
    }
    return failures;
  }

  geometry_msgs::msg::TransformStamped rog_from_planning;
  try {
    rog_from_planning =
      tf_->lookupTransform(rog_frame_, planning_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "[MincoPlanner] TF %s -> %s failed for ROGMap batch query: %s",
      planning_frame_.c_str(), rog_frame_.c_str(), ex.what());
    for (auto & failure : failures) {
      failure.status = rog_map::QueryStatus::TF_FAILED;
    }
    return failures;
  }

  const auto & q_msg = rog_from_planning.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  const Eigen::Vector3d translation(
    rog_from_planning.transform.translation.x,
    rog_from_planning.transform.translation.y,
    rog_from_planning.transform.translation.z);
  if (!q.coeffs().allFinite() || !translation.allFinite() || q.norm() < 1e-9) {
    for (auto & failure : failures) {
      failure.status = rog_map::QueryStatus::TF_FAILED;
    }
    return failures;
  }
  const Eigen::Quaterniond normalized_q = q.normalized();

  std::vector<Eigen::Vector3d> rog_positions;
  rog_positions.reserve(positions.size());
  for (const auto & position : positions) {
    rog_positions.push_back(normalized_q * position + translation);
  }
  auto results = raw_->queryBatch(rog_positions);
  if (results.size() != positions.size()) {
    return failures;
  }

  const Eigen::Quaterniond planning_from_rog = normalized_q.conjugate();
  for (auto & result : results) {
    if (!result.ok) {
      continue;
    }
    result.gradient = planning_from_rog * result.gradient;
    if (!std::isfinite(result.distance) || !result.gradient.allFinite()) {
      result.ok = false;
      result.status = rog_map::QueryStatus::NONFINITE_OUTPUT;
    }
  }
  return results;
}

bool FrameAwareRogQuery::transformPlanningToRog(
  const Eigen::Vector3d & in,
  Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(rog_frame_, planning_frame_, tf2::TimePointZero);
    out = transformPoint(tf, in);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] TF %s -> %s failed for ROGMap dynamic query: %s",
      planning_frame_.c_str(),
      rog_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool FrameAwareRogQuery::transformRogToPlanning(
  const Eigen::Vector3d & in,
  Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(planning_frame_, rog_frame_, tf2::TimePointZero);
    out = transformPoint(tf, in);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] TF %s -> %s failed for ROGMap dynamic query: %s",
      rog_frame_.c_str(),
      planning_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool FrameAwareRogQuery::rotateRogToPlanning(
  const Eigen::Vector3d & in,
  Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(planning_frame_, rog_frame_, tf2::TimePointZero);
    out = rotateVector(tf, in);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace minco_planner
