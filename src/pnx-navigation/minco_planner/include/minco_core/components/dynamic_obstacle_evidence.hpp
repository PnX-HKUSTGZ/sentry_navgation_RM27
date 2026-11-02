#pragma once

#include <cmath>
#include <cstdint>

#include "rog_map/map_query_interface.hpp"
#include "rog_map/projection_layer.hpp"

namespace minco_planner {
namespace dynamic_obstacle {

inline bool hasMeasuredOccupiedEvidence(const rog_map::QueryResult &result) {
  // Preserve distance-only behavior for non-ROG/custom query adapters.
  if (!result.projection.valid) {
    return true;
  }

  const auto source = result.projection.cost_source;
  if (source != rog_map::ProjectedCostSource::DYNAMIC_PROJECTION &&
      source != rog_map::ProjectedCostSource::DYNAMIC_AND_PRIOR) {
    return false;
  }
  if (result.projection.cost_cause ==
          rog_map::ProjectedCostCause::UNKNOWN_AS_OCCUPIED ||
      result.projection.cost_cause ==
          rog_map::ProjectedCostCause::MASK_DENOISE_TO_UNKNOWN ||
      result.projection.cost_cause ==
          rog_map::ProjectedCostCause::MASK_HOLE_FILL) {
    return false;
  }

  // A fail-closed UNKNOWN column can have raw/candidate/base type OCCUPIED even
  // though no obstacle return exists. Only a finite occupied vertical span and
  // an obstacle-producing geometric classification are allowed to alter global
  // topology. Local trajectory safety remains fail-closed for all other cases.
  const auto &projection = result.projection;
  if (!std::isfinite(projection.occupied_z_min) ||
      !std::isfinite(projection.occupied_z_max) ||
      projection.occupied_z_min > projection.occupied_z_max) {
    return false;
  }

  return rog_map::isMeasuredObstacleReason(
             static_cast<rog_map::ProjectionClassReason>(
                 projection.raw_reason)) ||
         rog_map::isMeasuredObstacleReason(
             static_cast<rog_map::ProjectionClassReason>(
                 projection.candidate_reason));
}

} // namespace dynamic_obstacle
} // namespace minco_planner
