#include <rog_map/query_adapter.hpp>
#include <rog_map/projection_layer.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rog_map {

namespace {

bool interpolationCoordinate(double world, double origin, double resolution,
                             int cell_count, int &lower, double &fraction) {
  const double extent = static_cast<double>(cell_count) * resolution;
  if (cell_count <= 1 || resolution <= 0.0 || world < origin ||
      world >= origin + extent) {
    return false;
  }

  // Grid origins describe the lower cell edge, while EDT samples live at cell
  // centers. Clamp the half-cell boundary bands to the nearest center so every
  // point accepted by worldToMap() also has a well-defined field value.
  const double centered = (world - origin) / resolution - 0.5;
  if (centered <= 0.0) {
    lower = 0;
    fraction = 0.0;
    return true;
  }
  if (centered >= static_cast<double>(cell_count - 1)) {
    lower = cell_count - 2;
    fraction = 1.0;
    return true;
  }

  lower = static_cast<int>(std::floor(centered));
  fraction = centered - static_cast<double>(lower);
  return true;
}

bool sampleBilinear(const MapSnapshot &snap, int ix, int iy, double fx,
                    double fy, double &dist, Eigen::Vector3d &grad) {
  const size_t width = static_cast<size_t>(snap.width);
  const size_t idx00 =
      static_cast<size_t>(iy) * width + static_cast<size_t>(ix);
  const size_t idx10 =
      static_cast<size_t>(iy) * width + static_cast<size_t>(ix + 1);
  const size_t idx01 =
      static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix);
  const size_t idx11 =
      static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix + 1);

  const double d00 = snap.distances[idx00];
  const double d10 = snap.distances[idx10];
  const double d01 = snap.distances[idx01];
  const double d11 = snap.distances[idx11];
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) ||
      !std::isfinite(d11)) {
    return false;
  }

  const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
  const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
  dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;
  if (!std::isfinite(dist)) {
    return false;
  }

  grad.x() = ((1.0 - fy) * (d10 - d00) + fy * (d11 - d01)) / snap.resolution;
  grad.y() = ((1.0 - fx) * (d01 - d00) + fx * (d11 - d10)) / snap.resolution;
  grad.z() = 0.0;
  if (!grad.allFinite()) {
    grad.setZero();
  }
  return true;
}

bool sampleQuadratic(const MapSnapshot &snap, int ix, int iy, double fx,
                     double fy, double &dist, Eigen::Vector3d &grad) {
  if (ix < 1 || iy < 1 || ix + 1 >= snap.width || iy + 1 >= snap.height) {
    return false;
  }
  const size_t width = static_cast<size_t>(snap.width);
  const auto sample = [&snap, width](int x, int y) {
    return snap
        .distances[static_cast<size_t>(y) * width + static_cast<size_t>(x)];
  };
  const double wx[3] = {0.5 * fx * (fx - 1.0), 1.0 - fx * fx,
                        0.5 * fx * (fx + 1.0)};
  const double wy[3] = {0.5 * fy * (fy - 1.0), 1.0 - fy * fy,
                        0.5 * fy * (fy + 1.0)};
  const double dwx[3] = {fx - 0.5, -2.0 * fx, fx + 0.5};
  const double dwy[3] = {fy - 0.5, -2.0 * fy, fy + 0.5};

  dist = 0.0;
  double dd_dx_pix = 0.0;
  double dd_dy_pix = 0.0;
  for (int dy = 0; dy < 3; ++dy) {
    for (int dx = 0; dx < 3; ++dx) {
      const double d = sample(ix + dx - 1, iy + dy - 1);
      if (!std::isfinite(d)) {
        return false;
      }
      dist += wx[dx] * wy[dy] * d;
      dd_dx_pix += dwx[dx] * wy[dy] * d;
      dd_dy_pix += wx[dx] * dwy[dy] * d;
    }
  }
  if (!std::isfinite(dist)) {
    return false;
  }
  grad.x() = dd_dx_pix / snap.resolution;
  grad.y() = dd_dy_pix / snap.resolution;
  grad.z() = 0.0;
  if (!grad.allFinite()) {
    grad.setZero();
  }
  return true;
}

void clampResult(const MapSnapshot &snap, double &dist, Eigen::Vector3d &grad) {
  if (!std::isfinite(dist)) {
    dist = snap.field_max_distance;
    grad.setZero();
    return;
  }
  if (snap.field_clamp_distance) {
    const double unclamped = dist;
    dist = std::clamp(dist, snap.field_min_distance, snap.field_max_distance);
    if (unclamped != dist) {
      grad.setZero();
    }
  }
  if (!grad.allFinite()) {
    grad.setZero();
  }
}

QueryResult makeResult(const MapSnapshot &snap, QueryStatus status) {
  QueryResult result;
  result.status = status;
  result.projection_sequence = snap.projection_sequence;
  result.mask_sequence = snap.mask_sequence;
  result.field_sequence = snap.field_sequence;
  result.snapshot_sequence =
      snap.snapshot_sequence != 0U ? snap.snapshot_sequence : snap.sequence;
  result.snapshot_stamp = snap.stamp;
  result.snapshot_commit_stamp = snap.commit_stamp;
  result.snapshot_processing_age_ms =
      snap.commit_stamp > 0.0 && snap.stamp > 0.0 &&
              snap.commit_stamp >= snap.stamp
          ? (snap.commit_stamp - snap.stamp) * 1000.0
          : std::numeric_limits<double>::quiet_NaN();
  result.field_age_ms = snap.field_stamp > 0.0 && snap.stamp >= snap.field_stamp
                            ? (snap.stamp - snap.field_stamp) * 1000.0
                            : std::numeric_limits<double>::quiet_NaN();
  return result;
}

QueryResult querySnapshotValue(
    const std::shared_ptr<const MapSnapshot> &snap,
    const Eigen::Vector3d &pos) {
  QueryResult invalid;
  if (!pos.allFinite()) {
    invalid.status = QueryStatus::NONFINITE_INPUT;
    return invalid;
  }
  if (!snap || snap->width <= 1 || snap->height <= 1 ||
      snap->resolution <= 0.0 ||
      snap->values.size() != static_cast<size_t>(snap->width) *
                                  static_cast<size_t>(snap->height)) {
    return invalid;
  }

  const int cell_x = static_cast<int>(
      std::floor((pos.x() - snap->origin_x) / snap->resolution));
  const int cell_y = static_cast<int>(
      std::floor((pos.y() - snap->origin_y) / snap->resolution));
  if (cell_x < 0 || cell_y < 0 || cell_x >= snap->width ||
      cell_y >= snap->height) {
    return makeResult(*snap, QueryStatus::OUT_OF_MAP);
  }

  QueryResult result = makeResult(*snap, QueryStatus::OK);
  result.projected_cost_valid = true;
  const size_t cell_index =
      static_cast<size_t>(cell_y) * static_cast<size_t>(snap->width) +
      static_cast<size_t>(cell_x);
  result.projected_cost = snap->values[cell_index];

  const size_t cell_count = static_cast<size_t>(snap->width) *
                            static_cast<size_t>(snap->height);
  const bool diagnostic_shape_valid =
      snap->dynamic_values.size() == cell_count &&
      snap->prior_occupied.size() == cell_count &&
      snap->types.size() == cell_count &&
      snap->raw_reasons.size() == cell_count &&
      snap->candidate_reasons.size() == cell_count;
  if (diagnostic_shape_valid) {
    auto & diagnostic = result.projection;
    diagnostic.valid = true;
    diagnostic.dynamic_cost = snap->dynamic_values[cell_index];
    diagnostic.cell_type = snap->types[cell_index];
    diagnostic.raw_reason = snap->raw_reasons[cell_index];
    diagnostic.candidate_reason = snap->candidate_reasons[cell_index];
    diagnostic.prior_occupied = snap->prior_occupied[cell_index] != 0U;
    const auto byte_at = [cell_count, cell_index](
        const std::vector<uint8_t> & values, uint8_t fallback) {
        return values.size() == cell_count ? values[cell_index] : fallback;
      };
    diagnostic.raw_type = byte_at(snap->raw_types, diagnostic.cell_type);
    diagnostic.candidate_type =
        byte_at(snap->candidate_types, diagnostic.raw_type);
    diagnostic.base_type = byte_at(snap->base_types, diagnostic.cell_type);
    diagnostic.pending_type =
        byte_at(snap->pending_types, diagnostic.base_type);
    diagnostic.pending_count = byte_at(snap->pending_counts, 0U);
    diagnostic.hole_filled = byte_at(snap->hole_filled, 0U) != 0U;
    const auto flag_at = [cell_count, cell_index](
        const std::vector<uint8_t> & values) {
        return values.size() == cell_count && values[cell_index] != 0U;
      };
    const auto float_at = [cell_count, cell_index](
        const std::vector<float> & values, float fallback) {
        return values.size() == cell_count ? values[cell_index] : fallback;
      };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    diagnostic.prior_known_free = flag_at(snap->prior_known_free);
    diagnostic.ground_candidate = flag_at(snap->ground_candidate);
    diagnostic.ground_verified = flag_at(snap->ground_verified);
    diagnostic.ground_support_known = flag_at(snap->ground_support_known);
    diagnostic.ground_support_verified =
        flag_at(snap->ground_support_verified);
    diagnostic.empty_support_verified = flag_at(snap->empty_support_verified);
    diagnostic.clearance_verified = flag_at(snap->clearance_verified);
    diagnostic.footprint_clear_eligible =
        flag_at(snap->footprint_clear_eligible);
    diagnostic.inside_current_footprint =
        flag_at(snap->inside_current_footprint);
    diagnostic.inside_surveyed_near_field =
        flag_at(snap->inside_surveyed_near_field);
    diagnostic.continuous_ground_support =
        flag_at(snap->continuous_ground_support);
    diagnostic.confidence = float_at(snap->confidence, 0.0F);
    diagnostic.occupied_z_min = float_at(snap->occupied_z_min, nan);
    diagnostic.occupied_z_max = float_at(snap->occupied_z_max, nan);
    diagnostic.height_delta = float_at(snap->height_deltas, nan);
    diagnostic.vertical_occupancy_ratio =
        float_at(snap->vertical_occupancy_ratio, nan);
    diagnostic.ground_z = float_at(snap->ground_z, nan);
    diagnostic.ground_support_z = float_at(snap->ground_support_z, nan);
    diagnostic.ceiling_z = float_at(snap->ceiling_z, nan);
    diagnostic.headroom = float_at(snap->headroom, nan);
    diagnostic.headroom_known_ratio =
        float_at(snap->headroom_known_ratio, nan);
    diagnostic.reference_ground_z = float_at(snap->reference_ground_z, nan);
    diagnostic.support_match_tolerance =
        float_at(snap->support_match_tolerance, nan);
    diagnostic.support_match_error = float_at(snap->support_match_error, nan);
    diagnostic.envelope_body_x = float_at(snap->envelope_body_x, nan);
    diagnostic.envelope_body_y = float_at(snap->envelope_body_y, nan);
    diagnostic.obstacle_hold_remaining =
        float_at(snap->obstacle_hold_remaining, 0.0F);

    const bool dynamic_lethal = diagnostic.dynamic_cost == 254U;
    if (diagnostic.prior_occupied && dynamic_lethal) {
      diagnostic.cost_source = ProjectedCostSource::DYNAMIC_AND_PRIOR;
    } else if (diagnostic.prior_occupied) {
      diagnostic.cost_source = ProjectedCostSource::PRIOR_MAP;
    } else if (dynamic_lethal) {
      diagnostic.cost_source = ProjectedCostSource::DYNAMIC_PROJECTION;
    }
    if (!dynamic_lethal && diagnostic.prior_occupied) {
      diagnostic.cost_cause = ProjectedCostCause::PRIOR_MAP;
    } else if (dynamic_lethal) {
      const auto final_type = static_cast<CellType>(diagnostic.cell_type);
      const auto raw_type = static_cast<CellType>(diagnostic.raw_type);
      const auto base_type = static_cast<CellType>(diagnostic.base_type);
      if (diagnostic.hole_filled) {
        diagnostic.cost_cause = ProjectedCostCause::MASK_HOLE_FILL;
      } else if (final_type == CellType::UNKNOWN &&
                 base_type == CellType::OCCUPIED) {
        // denoiseMask() is the only projection stage that changes an
        // OCCUPIED base cell into a final UNKNOWN cell.  In fail-closed mode
        // that UNKNOWN still encodes to 254, so distinguish it from a column
        // that was genuinely unknown before mask filtering.
        diagnostic.cost_cause =
          ProjectedCostCause::MASK_DENOISE_TO_UNKNOWN;
      } else if (final_type == CellType::UNKNOWN ||
                 base_type == CellType::UNKNOWN) {
        diagnostic.cost_cause = ProjectedCostCause::UNKNOWN_AS_OCCUPIED;
      } else if (raw_type == CellType::OCCUPIED) {
        diagnostic.cost_cause = ProjectedCostCause::RAW_OCCUPIED;
      } else if (diagnostic.obstacle_hold_remaining > 0.0F) {
        diagnostic.cost_cause = ProjectedCostCause::OBSTACLE_HOLD;
      } else if (base_type == CellType::OCCUPIED &&
                 diagnostic.pending_count > 0U) {
        diagnostic.cost_cause = ProjectedCostCause::HYSTERESIS;
      } else {
        diagnostic.cost_cause = ProjectedCostCause::RETAINED_OR_FILTERED;
      }
    }
  }
  if (snap->distances.size() !=
      static_cast<size_t>(snap->width) * static_cast<size_t>(snap->height)) {
    result.status = QueryStatus::FIELD_UNINITIALIZED;
    return result;
  }

  int ix = 0;
  int iy = 0;
  double fx = 0.0;
  double fy = 0.0;
  if (!interpolationCoordinate(pos.x(), snap->origin_x, snap->resolution,
                               snap->width, ix, fx) ||
      !interpolationCoordinate(pos.y(), snap->origin_y, snap->resolution,
                               snap->height, iy, fy)) {
    result.status = QueryStatus::OUT_OF_MAP;
    result.projected_cost_valid = false;
    return result;
  }

  double dist = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  bool ok = false;
  if (snap->interpolation == InterpolationMode::QUADRATIC) {
    ok = sampleQuadratic(*snap, ix, iy, fx, fy, dist, grad);
  }
  if (!ok) {
    ok = sampleBilinear(*snap, ix, iy, fx, fy, dist, grad);
  }
  if (!ok) {
    result.status = QueryStatus::INTERPOLATION_FAILED;
    return result;
  }

  clampResult(*snap, dist, grad);
  if (!std::isfinite(dist) || !grad.allFinite()) {
    result.status = QueryStatus::NONFINITE_OUTPUT;
    return result;
  }

  result.ok = true;
  result.distance = dist;
  result.gradient = grad;
  return result;
}

} // namespace

void QueryAdapter::update(const std::shared_ptr<const MapSnapshot> &snapshot,
                          const std::shared_ptr<DynamicLayer> &field) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = snapshot;
  field_ = field;
}

std::shared_ptr<const MapSnapshot> QueryAdapter::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

QueryCounters QueryAdapter::queryCounters() const {
  std::lock_guard<std::mutex> lock(counters_mutex_);
  return counters_;
}

void recordQueryStatus(QueryCounters &counters, QueryStatus status) {
  if (status == QueryStatus::OK) {
    ++counters.ok;
    return;
  }
  ++counters.failed;
  switch (status) {
  case QueryStatus::OK:
    break;
  case QueryStatus::OUT_OF_MAP:
    ++counters.out_of_map;
    break;
  case QueryStatus::SNAPSHOT_INVALID:
    ++counters.snapshot_invalid;
    break;
  case QueryStatus::FIELD_UNINITIALIZED:
    ++counters.field_uninitialized;
    break;
  case QueryStatus::INTERPOLATION_FAILED:
    ++counters.interpolation_failed;
    break;
  case QueryStatus::TF_FAILED:
    ++counters.tf_failed;
    break;
  case QueryStatus::NONFINITE_INPUT:
    ++counters.nonfinite_input;
    break;
  case QueryStatus::NONFINITE_OUTPUT:
    ++counters.nonfinite_output;
    break;
  }
}

QueryResult QueryAdapter::recorded(QueryResult result) const {
  std::lock_guard<std::mutex> lock(counters_mutex_);
  recordQueryStatus(counters_, result.status);
  return result;
}

bool QueryAdapter::worldToMap(double wx, double wy, unsigned int &mx,
                              unsigned int &my) const {
  const auto snap = snapshot();
  if (!snap || snap->width <= 0 || snap->height <= 0 ||
      snap->resolution <= 0.0) {
    return false;
  }
  if (wx < snap->origin_x || wy < snap->origin_y) {
    return false;
  }
  const int ix =
      static_cast<int>(std::floor((wx - snap->origin_x) / snap->resolution));
  const int iy =
      static_cast<int>(std::floor((wy - snap->origin_y) / snap->resolution));
  if (ix < 0 || iy < 0 || ix >= snap->width || iy >= snap->height) {
    return false;
  }
  mx = static_cast<unsigned int>(ix);
  my = static_cast<unsigned int>(iy);
  return true;
}

void QueryAdapter::mapToWorld(unsigned int mx, unsigned int my, double &wx,
                              double &wy) const {
  const auto snap = snapshot();
  if (!snap || snap->resolution <= 0.0) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  wx = snap->origin_x + (static_cast<double>(mx) + 0.5) * snap->resolution;
  wy = snap->origin_y + (static_cast<double>(my) + 0.5) * snap->resolution;
}

unsigned int QueryAdapter::sizeX() const {
  const auto snap = snapshot();
  return snap ? static_cast<unsigned int>(snap->width) : 0U;
}

unsigned int QueryAdapter::sizeY() const {
  const auto snap = snapshot();
  return snap ? static_cast<unsigned int>(snap->height) : 0U;
}

double QueryAdapter::resolution() const {
  const auto snap = snapshot();
  return snap ? snap->resolution : 0.0;
}

double QueryAdapter::originX() const {
  const auto snap = snapshot();
  return snap ? snap->origin_x : 0.0;
}

double QueryAdapter::originY() const {
  const auto snap = snapshot();
  return snap ? snap->origin_y : 0.0;
}

uint8_t QueryAdapter::value(unsigned int mx, unsigned int my) const {
  const auto snap = snapshot();
  if (!snap || mx >= static_cast<unsigned int>(snap->width) ||
      my >= static_cast<unsigned int>(snap->height)) {
    return 254U;
  }
  const size_t idx =
      static_cast<size_t>(my) * static_cast<size_t>(snap->width) +
      static_cast<size_t>(mx);
  return snap->values[idx];
}

const unsigned char *QueryAdapter::values() const {
  const auto snap = snapshot();
  if (!snap || snap->values.empty()) {
    return nullptr;
  }
  return snap->values.data();
}

bool QueryAdapter::copyValues(std::vector<unsigned char> &out) const {
  const auto snap = snapshot();
  if (!snap || snap->values.empty()) {
    out.clear();
    return false;
  }
  out.assign(snap->values.begin(), snap->values.end());
  return true;
}

bool QueryAdapter::isValid(unsigned int mx, unsigned int my) const {
  const auto snap = snapshot();
  return snap && mx < static_cast<unsigned int>(snap->width) &&
         my < static_cast<unsigned int>(snap->height);
}

bool QueryAdapter::isFree(unsigned int mx, unsigned int my) const {
  const auto snap = snapshot();
  if (!snap || mx >= static_cast<unsigned int>(snap->width) ||
      my >= static_cast<unsigned int>(snap->height)) {
    return false;
  }
  const size_t idx =
      static_cast<size_t>(my) * static_cast<size_t>(snap->width) +
      static_cast<size_t>(mx);
  return snap->values[idx] < 253U;
}

QueryResult QueryAdapter::query(const Eigen::Vector3d &pos) const {
  // Distance and projected cost are read from one immutable snapshot.
  return recorded(querySnapshotValue(snapshot(), pos));
}

std::vector<QueryResult> QueryAdapter::queryBatch(
    const std::vector<Eigen::Vector3d> &positions) const {
  const auto snap = snapshot();
  std::vector<QueryResult> results;
  results.reserve(positions.size());
  for (const auto &position : positions) {
    results.push_back(querySnapshotValue(snap, position));
  }
  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    for (const auto &result : results) {
      recordQueryStatus(counters_, result.status);
    }
  }
  return results;
}

bool QueryAdapter::evaluate(const Eigen::Vector3d &pos, double &dist,
                            Eigen::Vector3d &grad) const {
  const QueryResult result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

} // namespace rog_map
