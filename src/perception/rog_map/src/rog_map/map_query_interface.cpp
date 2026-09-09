#include <rog_map/map_query_interface.hpp>
#include <rog_map/projection_layer.hpp>

#include <cmath>

namespace rog_map {

const char *queryStatusName(QueryStatus status) {
  switch (status) {
  case QueryStatus::OK:
    return "OK";
  case QueryStatus::OUT_OF_MAP:
    return "OUT_OF_MAP";
  case QueryStatus::SNAPSHOT_INVALID:
    return "SNAPSHOT_INVALID";
  case QueryStatus::FIELD_UNINITIALIZED:
    return "FIELD_UNINITIALIZED";
  case QueryStatus::INTERPOLATION_FAILED:
    return "INTERPOLATION_FAILED";
  case QueryStatus::TF_FAILED:
    return "TF_FAILED";
  case QueryStatus::NONFINITE_INPUT:
    return "NONFINITE_INPUT";
  case QueryStatus::NONFINITE_OUTPUT:
    return "NONFINITE_OUTPUT";
  }
  return "UNKNOWN";
}

const char *projectedCostSourceName(ProjectedCostSource source) {
  switch (source) {
  case ProjectedCostSource::UNKNOWN:
    return "UNKNOWN";
  case ProjectedCostSource::DYNAMIC_PROJECTION:
    return "DYNAMIC_PROJECTION";
  case ProjectedCostSource::PRIOR_MAP:
    return "PRIOR_MAP";
  case ProjectedCostSource::DYNAMIC_AND_PRIOR:
    return "DYNAMIC_AND_PRIOR";
  }
  return "UNKNOWN";
}

const char *projectedCostCauseName(ProjectedCostCause cause) {
  switch (cause) {
  case ProjectedCostCause::UNKNOWN:
    return "UNKNOWN";
  case ProjectedCostCause::RAW_OCCUPIED:
    return "RAW_OCCUPIED";
  case ProjectedCostCause::UNKNOWN_AS_OCCUPIED:
    return "UNKNOWN_AS_OCCUPIED";
  case ProjectedCostCause::OBSTACLE_HOLD:
    return "OBSTACLE_HOLD";
  case ProjectedCostCause::HYSTERESIS:
    return "HYSTERESIS";
  case ProjectedCostCause::MASK_HOLE_FILL:
    return "MASK_HOLE_FILL";
  case ProjectedCostCause::MASK_DENOISE_TO_UNKNOWN:
    return "MASK_DENOISE_TO_UNKNOWN";
  case ProjectedCostCause::PRIOR_MAP:
    return "PRIOR_MAP";
  case ProjectedCostCause::RETAINED_OR_FILTERED:
    return "RETAINED_OR_FILTERED";
  }
  return "UNKNOWN";
}

const char *projectionCellTypeName(uint8_t type) {
  switch (static_cast<CellType>(type)) {
  case CellType::UNKNOWN:
    return "UNKNOWN";
  case CellType::FREE:
    return "FREE";
  case CellType::PASSABLE:
    return "PASSABLE";
  case CellType::OCCUPIED:
    return "OCCUPIED";
  }
  return "INVALID";
}

const char *projectionClassReasonName(uint8_t reason) {
  switch (static_cast<ProjectionClassReason>(reason)) {
  case ProjectionClassReason::INSUFFICIENT_OBSERVATION:
    return "INSUFFICIENT_OBSERVATION";
  case ProjectionClassReason::EMPTY_COLUMN:
    return "EMPTY_COLUMN";
  case ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR:
    return "ROBOT_FOOTPRINT_CLEAR";
  case ProjectionClassReason::THIN_SURFACE:
    return "THIN_SURFACE";
  case ProjectionClassReason::SOLID_VERTICAL_WALL:
    return "SOLID_VERTICAL_WALL";
  case ProjectionClassReason::HOLLOW_TUNNEL:
    return "HOLLOW_TUNNEL";
  case ProjectionClassReason::AMBIGUOUS_OCCUPIED:
    return "AMBIGUOUS_OCCUPIED";
  case ProjectionClassReason::GROUND_UNVERIFIED:
    return "GROUND_UNVERIFIED";
  case ProjectionClassReason::HEADROOM_UNVERIFIED:
    return "HEADROOM_UNVERIFIED";
  case ProjectionClassReason::HEADROOM_BLOCKED:
    return "HEADROOM_BLOCKED";
  case ProjectionClassReason::OVERHEAD_CLEARANCE_OK:
    return "OVERHEAD_CLEARANCE_OK";
  case ProjectionClassReason::CLEARANCE_OK:
    return "CLEARANCE_OK";
  case ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK:
    return "GROUND_BRIDGE_CLEARANCE_OK";
  case ProjectionClassReason::CLEARANCE_DROPOUT_HOLD:
    return "CLEARANCE_DROPOUT_HOLD";
  case ProjectionClassReason::CLEARANCE_BOUNDED_HOLE_FILL:
    return "CLEARANCE_BOUNDED_HOLE_FILL";
  case ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR:
    return "SURVEYED_NEAR_FIELD_CLEAR";
  }
  return "INVALID";
}

QueryResult MapQueryInterface::query(const Eigen::Vector3d &pos) const {
  QueryResult result;
  if (!pos.allFinite()) {
    result.status = QueryStatus::NONFINITE_INPUT;
    return result;
  }

  double dist = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  if (!evaluate(pos, dist, grad)) {
    result.status = QueryStatus::INTERPOLATION_FAILED;
    return result;
  }
  if (!std::isfinite(dist) || !grad.allFinite()) {
    result.status = QueryStatus::NONFINITE_OUTPUT;
    return result;
  }

  result.ok = true;
  result.status = QueryStatus::OK;
  result.distance = dist;
  result.gradient = grad;
  return result;
}

std::vector<QueryResult> MapQueryInterface::queryBatch(
    const std::vector<Eigen::Vector3d> &positions) const {
  std::vector<QueryResult> results;
  results.reserve(positions.size());
  for (const auto &position : positions) {
    results.push_back(query(position));
  }
  return results;
}

} // namespace rog_map
