#include <rog_map/projection_layer.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace rog_map {

namespace {

double elapsedMs(const std::chrono::steady_clock::time_point &start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

struct VerticalRun {
  int begin{0};
  int end{0};
};

std::vector<VerticalRun> occupiedRuns(const ColumnStats &stats) {
  std::vector<VerticalRun> runs;
  bool open = false;
  VerticalRun run;
  for (size_t i = 0; i < stats.vertical_states.size(); ++i) {
    if (stats.vertical_states[i] == VerticalVoxelState::OCCUPIED) {
      if (!open) {
        run.begin = static_cast<int>(i);
        run.end = run.begin;
        open = true;
      } else {
        run.end = static_cast<int>(i);
      }
    } else if (open) {
      runs.push_back(run);
      open = false;
    }
  }
  if (open) {
    runs.push_back(run);
  }
  return runs;
}

double centerZ(const ColumnStats &stats, int local_z, double resolution) {
  return stats.scan_z_min_abs + static_cast<double>(local_z) * resolution;
}

double knownRatioInBodyVolume(const ColumnStats &stats, double ground_z,
                              const ProjectionLayerConfig &config,
                              double resolution) {
  const double body_begin = ground_z + config.body_bottom_clearance;
  const double body_end =
      ground_z + config.vehicle_height + config.headroom_margin;
  int total = 0;
  int known = 0;
  for (double z = body_begin + 0.5 * resolution; z < body_end;
       z += resolution) {
    ++total;
    const int local_z =
        static_cast<int>(std::llround((z - stats.scan_z_min_abs) / resolution));
    if (local_z >= 0 &&
        local_z < static_cast<int>(stats.vertical_states.size()) &&
        stats.vertical_states[static_cast<size_t>(local_z)] !=
            VerticalVoxelState::UNKNOWN) {
      ++known;
    }
  }
  return total > 0 ? static_cast<double>(known) / static_cast<double>(total)
                   : 0.0;
}

bool hasTrustedGroundSupport(const ColumnStats &stats) {
  return stats.prior_known_free != 0U && stats.ground_support_known != 0U &&
         std::isfinite(stats.ground_support_z_abs);
}

bool matchesTrustedGroundSupport(const CellData &cell,
                                 const ProjectionLayerConfig &config) {
  return cell.prior_known_free != 0U && cell.ground_support_known != 0U &&
         std::isfinite(cell.ground_z_abs) &&
         std::isfinite(cell.ground_support_z_abs) &&
         std::abs(static_cast<double>(cell.ground_z_abs) -
                  static_cast<double>(cell.ground_support_z_abs)) <=
             config.ground_support_tolerance + 1.0e-6;
}

bool isClearanceProofReason(ProjectionClassReason reason) {
  switch (reason) {
  case ProjectionClassReason::EMPTY_COLUMN:
  case ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR:
  case ProjectionClassReason::OVERHEAD_CLEARANCE_OK:
  case ProjectionClassReason::CLEARANCE_OK:
  case ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK:
  case ProjectionClassReason::CLEARANCE_BOUNDED_HOLE_FILL:
  case ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR:
    return true;
  default:
    return false;
  }
}

bool isTraversableCellType(CellType type) {
  return type == CellType::FREE || type == CellType::PASSABLE;
}

CellType classifyLegacyCell(const ColumnStats &stats, CellData &cell,
                            const ProjectionLayerConfig &config,
                            double resolution) {
  const int min_observed = std::max(1, config.min_observed_voxels);
  cell.confidence =
      static_cast<float>(std::clamp(static_cast<double>(stats.observed_count) /
                                        static_cast<double>(min_observed),
                                    0.0, 1.0));

  if (stats.observed_count < min_observed) {
    cell.candidate_reason = ProjectionClassReason::INSUFFICIENT_OBSERVATION;
    cell.traversable = 0U;
    return CellType::UNKNOWN;
  }

  if (stats.occupied_count == 0) {
    cell.candidate_reason = ProjectionClassReason::EMPTY_COLUMN;
    cell.traversable = 1U;
    return CellType::FREE;
  }

  const bool valid_occupied_span =
      stats.occupied_z_index_min <= stats.occupied_z_index_max &&
      std::isfinite(stats.occupied_z_min_abs) &&
      std::isfinite(stats.occupied_z_max_abs);
  if (!valid_occupied_span) {
    cell.candidate_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
    cell.traversable = 0U;
    return CellType::OCCUPIED;
  }

  const int span_steps =
      stats.occupied_z_index_max - stats.occupied_z_index_min;
  const double height_delta = static_cast<double>(span_steps) * resolution;
  cell.occupied_z_min_abs = static_cast<float>(stats.occupied_z_min_abs);
  cell.occupied_z_max_abs = static_cast<float>(stats.occupied_z_max_abs);
  cell.height_delta = static_cast<float>(height_delta);

  if (height_delta <= config.surface_height_delta_max) {
    cell.vertical_occupancy_ratio = 1.0F;
    cell.candidate_reason = ProjectionClassReason::THIN_SURFACE;
    cell.traversable = 1U;
    return CellType::PASSABLE;
  }

  const double vertical_occupancy_ratio =
      std::clamp((static_cast<double>(stats.occupied_count - 1) * resolution) /
                     height_delta,
                 0.0, 1.0);
  cell.vertical_occupancy_ratio = static_cast<float>(vertical_occupancy_ratio);

  if (height_delta >= config.wall_height_delta_min &&
      vertical_occupancy_ratio >= config.wall_occupancy_ratio_min) {
    cell.candidate_reason = ProjectionClassReason::SOLID_VERTICAL_WALL;
    cell.traversable = 0U;
    return CellType::OCCUPIED;
  }

  if (height_delta >= config.tunnel_height_delta_min &&
      height_delta <= config.tunnel_height_delta_max &&
      vertical_occupancy_ratio <= config.tunnel_occupancy_ratio_max) {
    cell.candidate_reason = ProjectionClassReason::HOLLOW_TUNNEL;
    cell.traversable = 1U;
    return CellType::PASSABLE;
  }

  cell.candidate_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
  cell.traversable = 0U;
  return CellType::OCCUPIED;
}

CellType classifyClearanceCell(const ColumnStats &stats, CellData &cell,
                               const ProjectionLayerConfig &config,
                               double resolution) {
  const int min_observed = std::max(1, config.min_observed_voxels);
  cell.confidence =
      static_cast<float>(std::clamp(static_cast<double>(stats.observed_count) /
                                        static_cast<double>(min_observed),
                                    0.0, 1.0));

  if (stats.observed_count < min_observed) {
    cell.candidate_reason = ProjectionClassReason::INSUFFICIENT_OBSERVATION;
    return CellType::UNKNOWN;
  }
  if (stats.occupied_count == 0) {
    cell.candidate_reason = ProjectionClassReason::EMPTY_COLUMN;
    // A spinning LiDAR cannot observe support inside its downward blind ring.
    // PRIORMAP deployments may explicitly accept an empty column only when the
    // complete body-height interval above the reference ground is sufficiently
    // observed. Unknown columns remain closed and occupied returns still
    // require connected ground plus verified headroom.
    const bool trusted_support = hasTrustedGroundSupport(stats);
    const bool empty_authorized = config.require_ground_support
                                      ? trusted_support
                                      : config.observed_empty_as_free;
    if (empty_authorized) {
      const double support_z = config.require_ground_support
                                   ? stats.ground_support_z_abs
                                   : config.reference_ground_z_abs;
      cell.headroom_known_ratio = static_cast<float>(
          knownRatioInBodyVolume(stats, support_z, config, resolution));
      if (cell.headroom_known_ratio + 1.0e-6F >=
          static_cast<float>(config.min_headroom_known_ratio)) {
        cell.clearance_verified = 1U;
        cell.empty_support_verified = config.require_ground_support ? 1U : 0U;
        cell.traversable = 1U;
        return CellType::FREE;
      }
      cell.candidate_reason = ProjectionClassReason::HEADROOM_UNVERIFIED;
    } else if (config.require_ground_support) {
      cell.candidate_reason = ProjectionClassReason::GROUND_UNVERIFIED;
    }
    return CellType::OCCUPIED;
  }
  if (stats.vertical_states.empty()) {
    cell.candidate_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
    return CellType::OCCUPIED;
  }

  const auto runs = occupiedRuns(stats);
  if (runs.empty()) {
    cell.candidate_reason = ProjectionClassReason::AMBIGUOUS_OCCUPIED;
    return CellType::OCCUPIED;
  }

  const VerticalRun &lowest = runs.front();
  const double lowest_span =
      static_cast<double>(lowest.end - lowest.begin) * resolution;
  cell.occupied_z_min_abs =
      static_cast<float>(centerZ(stats, lowest.begin, resolution));
  cell.occupied_z_max_abs =
      static_cast<float>(centerZ(stats, runs.back().end, resolution));
  cell.height_delta =
      static_cast<float>(centerZ(stats, runs.back().end, resolution) -
                         centerZ(stats, lowest.begin, resolution));
  cell.vertical_occupancy_ratio =
      static_cast<float>(std::clamp(static_cast<double>(stats.occupied_count) /
                                        static_cast<double>(std::max<size_t>(
                                            1U, stats.vertical_states.size())),
                                    0.0, 1.0));

  // A roof can be observed either as a thin horizontal underside or as a
  // thick vertical leading face. Test the conservative lower voxel boundary
  // before treating the lowest run as ground. Only evidence from this column
  // is used: sufficient height, a known-free body band, and explicit
  // groundless/PRIORMAP authorization are all required.
  const double occupied_surface_inset =
      config.headroom_voxel_inset_fraction * resolution;
  const double lowest_bottom =
      centerZ(stats, lowest.begin, resolution) - occupied_surface_inset;
  const bool trusted_support = hasTrustedGroundSupport(stats);
  const double reference_ground_z =
      config.require_ground_support && trusted_support
          ? stats.ground_support_z_abs
          : config.reference_ground_z_abs;
  const double reference_headroom = lowest_bottom - reference_ground_z;
  const double required_headroom =
      config.vehicle_height + config.headroom_margin;
  if (reference_headroom + 1.0e-6 >= required_headroom) {
    cell.ceiling_z_abs = static_cast<float>(lowest_bottom);
    cell.headroom = static_cast<float>(reference_headroom);
    cell.headroom_known_ratio = static_cast<float>(
        knownRatioInBodyVolume(stats, reference_ground_z, config, resolution));
    // A measured overhead return and an empty column carry different evidence.
    // Keep their body-volume thresholds separate so sparse range-image rays at
    // a roof edge do not force deployments to weaken the empty-column gate.
    if (cell.headroom_known_ratio + 1.0e-6F <
        static_cast<float>(config.min_observed_overhead_headroom_known_ratio)) {
      cell.candidate_reason = ProjectionClassReason::HEADROOM_UNVERIFIED;
      return config.clearance_unknown_as_occupied ? CellType::OCCUPIED
                                                  : CellType::UNKNOWN;
    }
    const bool groundless_authorized = config.require_ground_support
                                           ? trusted_support
                                           : config.observed_empty_as_free;
    if (!groundless_authorized) {
      cell.candidate_reason = ProjectionClassReason::GROUND_UNVERIFIED;
      return CellType::OCCUPIED;
    }
    cell.clearance_verified = 1U;
    cell.candidate_reason = ProjectionClassReason::OVERHEAD_CLEARANCE_OK;
    cell.traversable = 1U;
    return CellType::PASSABLE;
  }

  // A driveable surface must start as the lowest thin occupied run. It is only
  // accepted later when it is connected to a seed underneath the robot.
  if (lowest_span > config.surface_height_delta_max) {
    // A thick run whose lower edge lies inside the required body band is a
    // wall or low beam. It must never fall through to ground connectivity.
    const double headroom = reference_headroom;
    cell.ceiling_z_abs = static_cast<float>(lowest_bottom);
    cell.headroom = static_cast<float>(headroom);
    cell.candidate_reason = ProjectionClassReason::HEADROOM_BLOCKED;
    return CellType::OCCUPIED;
  }

  cell.ground_candidate = 1U;
  double ground_z =
      centerZ(stats, lowest.end, resolution) + occupied_surface_inset;
  size_t ceiling_run = 1U;
  while (ceiling_run < runs.size()) {
    const double next_bottom =
        centerZ(stats, runs[ceiling_run].begin, resolution) -
        occupied_surface_inset;
    if (next_bottom - ground_z > config.body_bottom_clearance) {
      break;
    }
    ground_z = centerZ(stats, runs[ceiling_run].end, resolution) +
               occupied_surface_inset;
    ++ceiling_run;
  }
  cell.ground_z_abs = static_cast<float>(ground_z);
  cell.headroom_known_ratio = static_cast<float>(
      knownRatioInBodyVolume(stats, ground_z, config, resolution));

  if (ceiling_run < runs.size()) {
    const double ceiling_z =
        centerZ(stats, runs[ceiling_run].begin, resolution) -
        occupied_surface_inset;
    const double headroom = ceiling_z - ground_z;
    cell.ceiling_z_abs = static_cast<float>(ceiling_z);
    cell.headroom = static_cast<float>(headroom);
    if (headroom < required_headroom) {
      cell.candidate_reason = ProjectionClassReason::HEADROOM_BLOCKED;
      return CellType::OCCUPIED;
    }
  }

  if (cell.headroom_known_ratio + 1.0e-6F <
      static_cast<float>(config.min_headroom_known_ratio)) {
    cell.candidate_reason = ProjectionClassReason::HEADROOM_UNVERIFIED;
    return config.clearance_unknown_as_occupied ? CellType::OCCUPIED
                                                : CellType::UNKNOWN;
  }

  cell.clearance_verified = 1U;
  cell.candidate_reason = ProjectionClassReason::CLEARANCE_OK;
  cell.traversable = 1U;
  return CellType::PASSABLE;
}

CellType classifyCell(const ColumnStats &stats, CellData &cell,
                      const ProjectionLayerConfig &config, double resolution) {
  if (config.clearance_check_en || config.require_ground_support) {
    return classifyClearanceCell(stats, cell, config, resolution);
  }
  return classifyLegacyCell(stats, cell, config, resolution);
}

bool insideRobotEnvelope(double wx, double wy,
                         const ProjectionLayerConfig &config, double length,
                         double width, double raster_resolution = 0.0) {
  const double dx = wx - config.robot_x;
  const double dy = wy - config.robot_y;
  const double cos_yaw = std::cos(config.robot_yaw);
  const double sin_yaw = std::sin(config.robot_yaw);
  const double body_x = cos_yaw * dx + sin_yaw * dy;
  const double body_y = -sin_yaw * dx + cos_yaw * dy;
  // A point on the hard footprint boundary can map to a grid cell whose
  // center lies just outside the continuous rectangle. Include exactly the
  // projected half-cell extent so every intersecting cell is rasterized,
  // including at non-axis-aligned yaw. Eligibility and support/occupancy gates
  // still decide whether such a cell may be cleared.
  const double raster_padding = 0.5 * std::max(0.0, raster_resolution) *
                                (std::abs(cos_yaw) + std::abs(sin_yaw));
  return std::abs(body_x) <= 0.5 * length + raster_padding + 1.0e-6 &&
         std::abs(body_y) <= 0.5 * width + raster_padding + 1.0e-6;
}

} // namespace

ProjectionSlideResult ProjectionLayer::syncSlidingWindow(
    int width, int height, double resolution, const Eigen::Vector2i &min_id,
    const Eigen::Vector2d &origin, const ProjectionLayerConfig &config) {
  if (width <= 0 || height <= 0 || resolution <= 0.0 || width % 2 == 0 ||
      height % 2 == 0) {
    throw std::invalid_argument("ProjectionLayer sliding window requires "
                                "positive odd dimensions and resolution");
  }

  ProjectionSlideResult result;
  current_config_ = config;
  const Vec3i half_size(width / 2, height / 2, 0);
  const Vec3i center_id(min_id.x() + half_size.x(), min_id.y() + half_size.y(),
                        0);
  Vec3f center_pos;

  if (!initialized_) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;
    cell_buffer_.assign(
        static_cast<size_t>(width_) * static_cast<size_t>(height_), CellData{});
    cells_.assign(cell_buffer_.size(), CellData{});
    values_.assign(cell_buffer_.size(), 255U);
    mask_.assign(cell_buffer_.size(),
                 (config.unknown_as_occupied || config.require_ground_support)
                     ? 0U
                     : 1U);
    initSlidingMap(half_size, resolution, true, 0.0, Vec3f::Zero());
    globalIndexToPos(center_id, center_pos);
    updateLocalMapOriginAndBound(center_pos, center_id);
    initialized_ = true;
    view_rebuild_required_ = true;
    resetLocalMap();
    result.full_refresh_required = true;
    result.dirty_columns.resize(cell_buffer_.size());
    for (size_t i = 0; i < result.dirty_columns.size(); ++i) {
      result.dirty_columns[i] = static_cast<int>(i);
    }
    return result;
  }

  if (width_ != width || height_ != height ||
      std::abs(resolution_ - resolution) > 1.0e-9 ||
      (sc_.map_size_i - Vec3i(width, height, 1)).cwiseAbs().maxCoeff() != 0 ||
      cell_buffer_.size() !=
          static_cast<size_t>(width) * static_cast<size_t>(height)) {
    throw std::invalid_argument("ProjectionLayer sliding storage geometry "
                                "cannot change after initialization");
  }

  const Vec3i shift = center_id - local_map_origin_i_;
  result.window_moved = shift.x() != 0 || shift.y() != 0;
  view_rebuild_required_ = result.window_moved;
  result.full_refresh_required =
      std::abs(shift.x()) >= width_ || std::abs(shift.y()) >= height_;
  slide_dirty_hash_ids_.clear();
  globalIndexToPos(center_id, center_pos);
  SlidingMap::mapSliding(center_pos);
  origin_ = origin;

  std::vector<uint8_t> dirty_flags(cell_buffer_.size(), 0U);
  for (const int hash_id : slide_dirty_hash_ids_) {
    if (hash_id < 0 || hash_id >= static_cast<int>(cell_buffer_.size())) {
      continue;
    }
    Vec3i global_id;
    hashIdToGlobalIndex(hash_id, global_id);
    const int x = global_id.x() - local_map_bound_min_i_.x();
    const int y = global_id.y() - local_map_bound_min_i_.y();
    if (x < 0 || y < 0 || x >= width_ || y >= height_) {
      continue;
    }
    const int view_id = y * width_ + x;
    if (!dirty_flags[static_cast<size_t>(view_id)]) {
      dirty_flags[static_cast<size_t>(view_id)] = 1U;
      result.dirty_columns.push_back(view_id);
    }
  }
  if (result.full_refresh_required &&
      result.dirty_columns.size() != cell_buffer_.size()) {
    result.dirty_columns.resize(cell_buffer_.size());
    for (size_t i = 0; i < result.dirty_columns.size(); ++i) {
      result.dirty_columns[i] = static_cast<int>(i);
    }
  }
  return result;
}

void ProjectionLayer::update(int width, int height, double resolution,
                             const Eigen::Vector2d &origin, double now,
                             const ProjectionLayerConfig &config,
                             const ColumnScanner &scanner,
                             ProjectionUpdateStats *stats) {
  updateFull(width, height, resolution, origin, now, config, scanner, stats);
}

void ProjectionLayer::updateFull(int width, int height, double resolution,
                                 const Eigen::Vector2d &origin, double now,
                                 const ProjectionLayerConfig &config,
                                 const ColumnScanner &scanner,
                                 ProjectionUpdateStats *stats) {
  const auto full_start = std::chrono::steady_clock::now();
  if (width <= 0 || height <= 0 || resolution <= 0.0 || !scanner) {
    width_ = 0;
    height_ = 0;
    resolution_ = 0.0;
    cells_.clear();
    values_.clear();
    mask_.clear();
    if (stats) {
      stats->update_full_time_ms = elapsedMs(full_start);
    }
    return;
  }

  if (!initialized_) {
    const Eigen::Vector2i min_id(
        static_cast<int>(std::llround(origin.x() / resolution)),
        static_cast<int>(std::llround(origin.y() / resolution)));
    syncSlidingWindow(width, height, resolution, min_id, origin, config);
  }
  if (!matchesGeometry(width, height, resolution, origin)) {
    throw std::invalid_argument(
        "ProjectionLayer::updateFull called with unsynchronized geometry");
  }
  current_config_ = config;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      stageOneCell(x, y, now, config, scanner);
    }
  }
  resolveGroundConnectivityAndCommit(now, config);

  const auto filter_start = std::chrono::steady_clock::now();
  double view_time_ms = 0.0;
  filterMask(config, nullptr, &view_time_ms);
  view_rebuild_required_ = false;
  if (stats) {
    stats->mask_filter_time_ms +=
        std::max(0.0, elapsedMs(filter_start) - view_time_ms);
    stats->value_mask_time_ms += view_time_ms;
    stats->update_full_time_ms += elapsedMs(full_start);
    collectClassificationStats(cells_, stats);
  }
}

void ProjectionLayer::updateDirty(int width, int height, double resolution,
                                  const Eigen::Vector2d &origin, double now,
                                  const ProjectionLayerConfig &config,
                                  const ColumnScanner &scanner,
                                  const std::vector<int> &dirty_columns,
                                  bool force_full_refresh,
                                  ProjectionUpdateStats *stats) {
  const auto dirty_start = std::chrono::steady_clock::now();
  const bool geometry_changed =
      !matchesGeometry(width, height, resolution, origin);
  if (force_full_refresh || geometry_changed || cells_.empty()) {
    updateFull(width, height, resolution, origin, now, config, scanner, stats);
    return;
  }

  current_config_ = config;
  if (dirty_columns.empty()) {
    if (config.clearance_check_en || config.require_ground_support) {
      resolveGroundConnectivityAndCommit(now, config);
      const auto filter_start = std::chrono::steady_clock::now();
      double view_time_ms = 0.0;
      filterMask(config, nullptr, &view_time_ms);
      view_rebuild_required_ = false;
      if (stats) {
        stats->mask_filter_time_ms +=
            std::max(0.0, elapsedMs(filter_start) - view_time_ms);
        stats->value_mask_time_ms += view_time_ms;
      }
    }
    if (stats) {
      stats->update_dirty_time_ms += elapsedMs(dirty_start);
      collectClassificationStats(cells_, stats);
    }
    return;
  }

  const size_t expected =
      static_cast<size_t>(width_) * static_cast<size_t>(height_);
  std::vector<int> base_dirty_indices;
  base_dirty_indices.reserve(dirty_columns.size());
  for (const int column_id : dirty_columns) {
    if (column_id < 0 || column_id >= static_cast<int>(expected)) {
      continue;
    }
    base_dirty_indices.push_back(column_id);
  }
  std::sort(base_dirty_indices.begin(), base_dirty_indices.end());
  base_dirty_indices.erase(
      std::unique(base_dirty_indices.begin(), base_dirty_indices.end()),
      base_dirty_indices.end());

  for (const int idx_int : base_dirty_indices) {
    stageOneCell(idx_int % width_, idx_int / width_, now, config, scanner);
  }
  resolveGroundConnectivityAndCommit(now, config);

  const auto filter_start = std::chrono::steady_clock::now();
  double view_time_ms = 0.0;
  // Ground connectivity can change beyond a dirty column's morphology radius.
  // Rebuild the 2D view whenever clearance checking is active.
  filterMask(config,
             (view_rebuild_required_ || config.clearance_check_en ||
              config.require_ground_support)
                 ? nullptr
                 : &base_dirty_indices,
             &view_time_ms);
  view_rebuild_required_ = false;
  if (stats) {
    stats->mask_filter_time_ms +=
        std::max(0.0, elapsedMs(filter_start) - view_time_ms);
    stats->value_mask_time_ms += view_time_ms;
    stats->update_dirty_time_ms += elapsedMs(dirty_start);
    collectClassificationStats(cells_, stats);
  }
}

bool ProjectionLayer::matchesGeometry(int width, int height, double resolution,
                                      const Eigen::Vector2d &origin) const {
  return width_ == width && height_ == height &&
         cells_.size() ==
             static_cast<size_t>(width) * static_cast<size_t>(height) &&
         std::abs(resolution_ - resolution) <= 1.0e-9 &&
         (origin_ - origin).norm() <= 1.0e-9;
}

void ProjectionLayer::applyValueAndMask(CellData &cell,
                                        const ProjectionLayerConfig &config) {
  // value is a cost/debug layer; mask is the 2D ESDF source, where 0 is
  // obstacle and 1 is free.
  switch (cell.type) {
  case CellType::UNKNOWN:
    cell.value = (config.unknown_as_occupied || config.require_ground_support)
                     ? 254U
                     : 255U;
    cell.mask =
        (config.unknown_as_occupied || config.require_ground_support) ? 0U : 1U;
    cell.traversable =
        (config.unknown_as_occupied || config.require_ground_support) ? 0U : 1U;
    break;
  case CellType::FREE:
    cell.value = 0U;
    cell.mask = 1U;
    cell.traversable = 1U;
    break;
  case CellType::PASSABLE:
    cell.value = config.passable_as_free ? 0U : config.passable_cost;
    cell.mask = 1U;
    cell.traversable = 1U;
    break;
  case CellType::OCCUPIED:
    cell.value = 254U;
    cell.mask = 0U;
    cell.traversable = 0U;
    break;
  }
}

void ProjectionLayer::collectClassificationStats(
    const std::vector<CellData> &cells, ProjectionUpdateStats *stats) {
  if (!stats) {
    return;
  }
  stats->thin_surface_count = 0.0;
  stats->vertical_wall_count = 0.0;
  stats->hollow_tunnel_count = 0.0;
  stats->ambiguous_occupied_count = 0.0;
  stats->empty_column_count = 0.0;
  stats->clearance_dropout_hold_count = 0.0;
  stats->clearance_hole_fill_count = 0.0;
  stats->insufficient_observation_count = 0.0;
  for (const auto &cell : cells) {
    switch (cell.raw_reason) {
    case ProjectionClassReason::INSUFFICIENT_OBSERVATION:
      stats->insufficient_observation_count += 1.0;
      break;
    case ProjectionClassReason::EMPTY_COLUMN:
      stats->empty_column_count += 1.0;
      break;
    case ProjectionClassReason::CLEARANCE_DROPOUT_HOLD:
      stats->empty_column_count += 1.0;
      stats->clearance_dropout_hold_count += 1.0;
      break;
    case ProjectionClassReason::CLEARANCE_BOUNDED_HOLE_FILL:
      stats->empty_column_count += 1.0;
      stats->clearance_hole_fill_count += 1.0;
      break;
    case ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR:
    case ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR:
      stats->insufficient_observation_count += 1.0;
      break;
    case ProjectionClassReason::THIN_SURFACE:
    case ProjectionClassReason::OVERHEAD_CLEARANCE_OK:
    case ProjectionClassReason::CLEARANCE_OK:
    case ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK:
      stats->thin_surface_count += 1.0;
      break;
    case ProjectionClassReason::SOLID_VERTICAL_WALL:
      stats->vertical_wall_count += 1.0;
      break;
    case ProjectionClassReason::HOLLOW_TUNNEL:
      stats->hollow_tunnel_count += 1.0;
      break;
    case ProjectionClassReason::AMBIGUOUS_OCCUPIED:
    case ProjectionClassReason::GROUND_UNVERIFIED:
    case ProjectionClassReason::HEADROOM_UNVERIFIED:
    case ProjectionClassReason::HEADROOM_BLOCKED:
      stats->ambiguous_occupied_count += 1.0;
      break;
    }
  }
}

void ProjectionLayer::stageOneCell(int x, int y, double now,
                                   const ProjectionLayerConfig &config,
                                   const ColumnScanner &scanner) {
  (void)now;
  const ColumnStats stats = scanner(x, y);
  const int hash_id = hashIndexFromLocal(x, y);
  const CellData previous = cell_buffer_[static_cast<size_t>(hash_id)];

  CellData cell = previous;
  cell.last_hit_time = static_cast<float>(stats.last_hit_time);
  cell.last_update_time = static_cast<float>(stats.last_update_time);
  cell.confidence = 0.0F;
  cell.occupied_z_min_abs = std::numeric_limits<float>::quiet_NaN();
  cell.occupied_z_max_abs = std::numeric_limits<float>::quiet_NaN();
  cell.height_delta = 0.0F;
  cell.vertical_occupancy_ratio = 0.0F;
  cell.ground_z_abs = std::numeric_limits<float>::quiet_NaN();
  cell.ceiling_z_abs = std::numeric_limits<float>::quiet_NaN();
  cell.headroom = std::numeric_limits<float>::infinity();
  cell.headroom_known_ratio = 0.0F;
  cell.ground_candidate = 0U;
  cell.ground_verified = 0U;
  cell.ground_bridge_verified = 0U;
  cell.ground_support_bridge_eligible = 0U;
  cell.prior_known_free = stats.prior_known_free;
  cell.ground_support_known = stats.ground_support_known;
  cell.ground_support_verified = 0U;
  cell.empty_support_verified = 0U;
  cell.ground_support_z_abs = static_cast<float>(stats.ground_support_z_abs);
  cell.clearance_verified = 0U;
  cell.footprint_clear_eligible = 0U;
  cell.clearance_dropout_eligible = 0U;
  cell.near_field_prior_fill_eligible = 0U;
  cell.inside_current_footprint = 0U;
  cell.inside_surveyed_near_field = 0U;
  cell.continuous_ground_support = 0U;
  cell.reference_ground_z_abs = std::numeric_limits<float>::quiet_NaN();
  cell.support_match_tolerance = std::numeric_limits<float>::quiet_NaN();
  cell.support_match_error = std::numeric_limits<float>::quiet_NaN();
  cell.envelope_body_x = std::numeric_limits<float>::quiet_NaN();
  cell.envelope_body_y = std::numeric_limits<float>::quiet_NaN();
  cell.traversable = 0U;
  cell.candidate_reason = ProjectionClassReason::INSUFFICIENT_OBSERVATION;
  cell.candidate_type = classifyCell(stats, cell, config, resolution_);
  const bool contains_occupied_voxel =
      std::any_of(stats.vertical_states.begin(), stats.vertical_states.end(),
                  [](VerticalVoxelState state) {
                    return state == VerticalVoxelState::OCCUPIED;
                  });
  const bool zero_hit_unverified =
      cell.candidate_reason ==
          ProjectionClassReason::INSUFFICIENT_OBSERVATION ||
      cell.candidate_reason == ProjectionClassReason::HEADROOM_UNVERIFIED ||
      cell.candidate_reason == ProjectionClassReason::GROUND_UNVERIFIED;
  const bool surveyed_ground_with_headroom_gap =
      config.require_ground_support && config.near_field_prior_fill_en &&
      cell.candidate_reason == ProjectionClassReason::HEADROOM_UNVERIFIED &&
      cell.ground_candidate != 0U && stats.occupied_count > 0 &&
      contains_occupied_voxel;
  if ((config.clearance_check_en || config.require_ground_support) &&
      zero_hit_unverified &&
      ((stats.occupied_count == 0 && !contains_occupied_voxel) ||
       surveyed_ground_with_headroom_gap)) {
    // A matching, connected ground return is not an obstacle merely because
    // the spinning LiDAR has not yet filled the body-height ray column above
    // it. Let the bounded near-field branch below evaluate that case against
    // surveyed free space and support height. A second body-band run or a low
    // ceiling is classified HEADROOM_BLOCKED and remains an immediate veto.
    cell.footprint_clear_eligible = 1U;
  } else if (stats.occupied_count != 0 || contains_occupied_voxel ||
             !zero_hit_unverified) {
    // Any occupied return that was not classified as matching ground is an
    // immediate veto. Do not allow a bridge confirmation accumulated before
    // the return to leak into this frame.
    cell.ground_support_bridge_count = 0U;
  }
  cell_buffer_[static_cast<size_t>(hash_id)] = cell;
}

void ProjectionLayer::commitCell(CellData &cell, CellType raw_type,
                                 ProjectionClassReason raw_reason, double now,
                                 const ProjectionLayerConfig &config) {
  const CellType previous_raw_type = cell.raw_type;
  const ProjectionClassReason previous_raw_reason = cell.raw_reason;
  const bool previous_measured_obstacle =
      previous_raw_type == CellType::OCCUPIED &&
      isMeasuredObstacleReason(previous_raw_reason);
  // Only the measured-obstacle branch below can start this deadline. Once
  // started, keep it active even though raw_reason follows the new free proof.
  const bool hold_active = cell.occupied_clear_deadline > 0.0;
  const bool zero_hit_headroom_dropout =
      config.require_ground_support &&
      config.clearance_dropout_hold_time > 0.0 &&
      raw_type == CellType::OCCUPIED &&
      raw_reason == ProjectionClassReason::HEADROOM_UNVERIFIED &&
      cell.footprint_clear_eligible != 0U &&
      cell.clearance_dropout_eligible != 0U;
  const bool starts_dropout_hold = zero_hit_headroom_dropout &&
                                   isTraversableCellType(previous_raw_type) &&
                                   isClearanceProofReason(previous_raw_reason);
  const bool continues_dropout_hold =
      zero_hit_headroom_dropout && isTraversableCellType(previous_raw_type) &&
      previous_raw_reason == ProjectionClassReason::CLEARANCE_DROPOUT_HOLD &&
      cell.clearance_dropout_deadline > now;
  if (starts_dropout_hold) {
    cell.clearance_dropout_deadline = now + config.clearance_dropout_hold_time;
  }
  if ((starts_dropout_hold || continues_dropout_hold) &&
      now < cell.clearance_dropout_deadline) {
    // Missing body-band miss rays are not new obstacle evidence. Retain a
    // recent clearance proof only for a bounded interval and only while the
    // surveyed support remains continuous. Body-band, low-ceiling, or
    // wrong-height occupied evidence makes the cell ineligible above and
    // therefore blocks immediately.
    raw_type = previous_raw_type;
    raw_reason = ProjectionClassReason::CLEARANCE_DROPOUT_HOLD;
    cell.clearance_verified = 1U;
    cell.empty_support_verified = 1U;
    cell.ground_support_verified = 1U;
    cell.traversable = 1U;
  } else if (!zero_hit_headroom_dropout ||
             now >= cell.clearance_dropout_deadline) {
    cell.clearance_dropout_deadline = 0.0;
  }
  cell.raw_type = raw_type;
  cell.raw_reason = raw_reason;
  if (raw_type == CellType::OCCUPIED) {
    cell.base_type = CellType::OCCUPIED;
    cell.occupied_clear_deadline = 0.0;
    cell.pending_type = CellType::OCCUPIED;
    cell.pending_count = 0U;
  } else if (config.obstacle_hold_time > 0.0 &&
             (previous_measured_obstacle || hold_active)) {
    if (!hold_active) {
      cell.occupied_clear_deadline = now + config.obstacle_hold_time;
    }
    if (now < cell.occupied_clear_deadline) {
      cell.base_type = CellType::OCCUPIED;
      cell.pending_type = raw_type;
      cell.pending_count = 0U;
    } else {
      cell.base_type = raw_type;
      cell.occupied_clear_deadline = 0.0;
      cell.pending_type = raw_type;
      cell.pending_count = 0U;
    }
  } else if ((cell.clearance_verified != 0U &&
              isClearanceProofReason(raw_reason)) ||
             raw_reason == ProjectionClassReason::CLEARANCE_DROPOUT_HOLD) {
    // These classifications already carry an explicit clearance proof (or its
    // bounded zero-hit continuation). Keep the wall-clock obstacle hold above
    // authoritative when configured, but do not add a second frame-count
    // delay that makes a narrow opening flicker back to lethal.
    cell.base_type = raw_type;
    cell.occupied_clear_deadline = 0.0;
    cell.pending_type = raw_type;
    cell.pending_count = 0U;
  } else {
    cell.occupied_clear_deadline = 0.0;
    cell.base_type = applyHysteresis(cell, raw_type, config);
  }
  cell.type = cell.base_type;
  cell.hole_filled = false;
}

void ProjectionLayer::resolveGroundConnectivityAndCommit(
    double now, const ProjectionLayerConfig &config) {
  if (!config.clearance_check_en && !config.require_ground_support) {
    for (auto &cell : cell_buffer_) {
      cell.near_field_prior_fill_eligible = 0U;
      commitCell(cell, cell.candidate_type, cell.candidate_reason, now, config);
    }
    return;
  }

  struct ConnectivityNode {
    int view_id;
    double ground_z;
    double bridge_distance;
    int bridge_start_view_id;
    bool is_empty_bridge;
    bool bridge_used;
  };
  std::queue<ConnectivityNode> open;
  std::vector<double> best_empty_bridge_distance(
      static_cast<size_t>(width_) * static_cast<size_t>(height_),
      std::numeric_limits<double>::infinity());
  struct SeedCandidate {
    int view_id;
    double ground_z;
    double distance;
  };
  std::vector<SeedCandidate> seed_candidates;
  double seed_ground_z = std::numeric_limits<double>::infinity();
  double seed_distance = std::numeric_limits<double>::infinity();
  constexpr double kPi = 3.14159265358979323846;
  const double slope = std::tan(config.max_ground_slope_deg * kPi / 180.0);
  const auto has_connected_ground_support = [&](int x, int y) {
    const CellData &cell =
        cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
    if (cell.prior_known_free == 0U || cell.ground_support_known == 0U ||
        !std::isfinite(cell.ground_support_z_abs)) {
      return false;
    }
    constexpr std::array<std::array<int, 2>, 4> kCardinalNeighbors{{
        {{1, 0}},
        {{-1, 0}},
        {{0, 1}},
        {{0, -1}},
    }};
    for (const auto &direction : kCardinalNeighbors) {
      const int neighbor_x = x + direction[0];
      const int neighbor_y = y + direction[1];
      if (neighbor_x < 0 || neighbor_x >= width_ || neighbor_y < 0 ||
          neighbor_y >= height_) {
        continue;
      }
      const CellData &neighbor = cell_buffer_[static_cast<size_t>(
          hashIndexFromLocal(neighbor_x, neighbor_y))];
      if (neighbor.prior_known_free == 0U ||
          neighbor.ground_support_known == 0U ||
          !std::isfinite(neighbor.ground_support_z_abs)) {
        continue;
      }
      const double allowed_step =
          std::max(config.max_ground_step, slope * resolution_);
      if (std::abs(static_cast<double>(cell.ground_support_z_abs) -
                   static_cast<double>(neighbor.ground_support_z_abs)) <=
          allowed_step + 1.0e-6) {
        // One connected cardinal neighbor is sufficient evidence that this
        // surveyed cell belongs to a local support surface. Requiring every
        // eight-neighbor to agree dilates a lateral step into every footprint
        // sample beside it. Actual occupied voxels are classified before this
        // support gate and remain an unconditional veto.
        return true;
      }
    }
    return false;
  };
  const auto has_observed_support_bridge = [&](int x, int y) {
    if (!config.observed_ground_support_bridge_en ||
        !config.require_ground_support) {
      return false;
    }
    const double wx =
        origin_.x() + (static_cast<double>(x) + 0.5) * resolution_;
    const double wy =
        origin_.y() + (static_cast<double>(y) + 0.5) * resolution_;
    const bool inside_footprint =
        config.clear_robot_footprint_unknown &&
        insideRobotEnvelope(wx, wy, config, config.robot_footprint_clear_length,
                            config.robot_footprint_clear_width, resolution_);
    if (!inside_footprint) {
      return false;
    }

    // Use the hard current footprint only. The larger near-field prior sweep
    // has a separate fail-closed path and must not become a blanket bridge.
    // Cardinal neighbors only: diagonal/8-neighbor expansion lets a footprint
    // sample beside a step borrow support from around the corner.
    constexpr std::array<std::array<int, 2>, 4> kCardinalNeighbors{{
        {{1, 0}},
        {{-1, 0}},
        {{0, 1}},
        {{0, -1}},
    }};
    int support_count = 0;
    double support_min = std::numeric_limits<double>::infinity();
    double support_max = -std::numeric_limits<double>::infinity();
    for (const auto &direction : kCardinalNeighbors) {
      const int neighbor_x = x + direction[0];
      const int neighbor_y = y + direction[1];
      if (neighbor_x < 0 || neighbor_x >= width_ || neighbor_y < 0 ||
          neighbor_y >= height_) {
        continue;
      }
      const CellData &neighbor = cell_buffer_[static_cast<size_t>(
          hashIndexFromLocal(neighbor_x, neighbor_y))];
      if (neighbor.candidate_type == CellType::OCCUPIED ||
          neighbor.traversable == 0U || neighbor.clearance_verified == 0U ||
          neighbor.prior_known_free == 0U ||
          neighbor.ground_support_known == 0U ||
          !std::isfinite(neighbor.ground_support_z_abs)) {
        continue;
      }
      const double support_z =
          static_cast<double>(neighbor.ground_support_z_abs);
      if (std::abs(support_z - config.reference_ground_z_abs) >
          config.max_ground_height_delta + 1.0e-6) {
        continue;
      }
      ++support_count;
      support_min = std::min(support_min, support_z);
      support_max = std::max(support_max, support_z);
    }
    return support_count >= config.observed_ground_support_bridge_min_neighbors &&
           std::isfinite(support_min) && std::isfinite(support_max) &&
           support_max - support_min <=
               config.observed_ground_support_bridge_max_height_delta + 1.0e-6;
  };
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      CellData &cell =
          cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      cell.ground_verified = 0U;
      cell.ground_bridge_verified = 0U;
      cell.ground_support_verified = 0U;
      cell.ground_support_bridge_eligible = 0U;
      if (cell.ground_candidate == 0U || !std::isfinite(cell.ground_z_abs)) {
        continue;
      }
      if (config.require_ground_support) {
        if (matchesTrustedGroundSupport(cell, config) &&
            has_connected_ground_support(x, y)) {
          cell.ground_verified = 1U;
          cell.ground_support_verified = 1U;
        }
        continue;
      }
      const double wx =
          origin_.x() + (static_cast<double>(x) + 0.5) * resolution_;
      const double wy =
          origin_.y() + (static_cast<double>(y) + 0.5) * resolution_;
      const double distance =
          std::hypot(wx - config.robot_x, wy - config.robot_y);
      const double ground_z = static_cast<double>(cell.ground_z_abs);
      if (distance > config.ground_seed_radius ||
          std::abs(ground_z - config.reference_ground_z_abs) >
              config.ground_seed_tolerance + 1.0e-6) {
        continue;
      }
      seed_candidates.push_back({y * width_ + x, ground_z, distance});

      // The lowest reference-consistent candidate anchors a narrow seed height
      // band. Multiple coplanar seeds are needed when the LiDAR blind ring or a
      // wall splits the first observed ground ring into disconnected arcs.
      if (ground_z < seed_ground_z - 1.0e-6 ||
          (std::abs(ground_z - seed_ground_z) <= 1.0e-6 &&
           distance < seed_distance)) {
        seed_ground_z = ground_z;
        seed_distance = distance;
      }
    }
  }

  const double seed_height_band =
      std::min(std::max(0.0, config.max_ground_step), resolution_);
  for (const auto &candidate : seed_candidates) {
    if (std::abs(candidate.ground_z - seed_ground_z) >
        seed_height_band + 1.0e-6) {
      continue;
    }
    const int seed_x = candidate.view_id % width_;
    const int seed_y = candidate.view_id / width_;
    CellData &seed =
        cell_buffer_[static_cast<size_t>(hashIndexFromLocal(seed_x, seed_y))];
    seed.ground_verified = 1U;
    open.push({candidate.view_id, candidate.ground_z, 0.0, candidate.view_id,
               false, false});
  }

  const auto has_consistent_landing_evidence = [&](int start_view_id,
                                                   int landing_view_id,
                                                   double start_ground_z,
                                                   double landing_ground_z) {
    const int start_x = start_view_id % width_;
    const int start_y = start_view_id / width_;
    const int landing_x = landing_view_id % width_;
    const int landing_y = landing_view_id / width_;
    const double direction_x = static_cast<double>(landing_x - start_x);
    const double direction_y = static_cast<double>(landing_y - start_y);
    const double direction_norm = std::hypot(direction_x, direction_y);
    if (direction_norm <= 1.0e-9) {
      return false;
    }

    const double bridge_height = landing_ground_z - start_ground_z;
    if (std::abs(bridge_height) <= config.max_ground_step + 1.0e-6) {
      return false;
    }
    const double height_sign = bridge_height > 0.0 ? 1.0 : -1.0;
    const double unit_x = direction_x / direction_norm;
    const double unit_y = direction_y / direction_norm;
    const int sample_count =
        std::max(1, static_cast<int>(std::ceil(
                        config.ground_connectivity_bridge_landing_min_length /
                        resolution_)));
    int previous_x = landing_x;
    int previous_y = landing_y;
    double previous_z = landing_ground_z;
    double sampled_length = 0.0;
    double signed_height_progress = 0.0;
    for (int sample_index = 1; sample_index <= sample_count; ++sample_index) {
      const int sample_x = static_cast<int>(
          std::llround(static_cast<double>(landing_x) +
                       unit_x * static_cast<double>(sample_index)));
      const int sample_y = static_cast<int>(
          std::llround(static_cast<double>(landing_y) +
                       unit_y * static_cast<double>(sample_index)));
      if (sample_x < 0 || sample_x >= width_ || sample_y < 0 ||
          sample_y >= height_) {
        return false;
      }
      if (sample_x == previous_x && sample_y == previous_y) {
        continue;
      }
      const CellData &sample = cell_buffer_[static_cast<size_t>(
          hashIndexFromLocal(sample_x, sample_y))];
      if (sample.ground_candidate == 0U ||
          sample.candidate_type != CellType::PASSABLE ||
          sample.clearance_verified == 0U ||
          !std::isfinite(sample.ground_z_abs)) {
        return false;
      }
      const double segment_length =
          resolution_ *
          std::hypot(sample_x - previous_x, sample_y - previous_y);
      const double sample_z = static_cast<double>(sample.ground_z_abs);
      const double height_step = sample_z - previous_z;
      const double allowed_step =
          std::max(config.max_ground_step, slope * segment_length);
      if (std::abs(height_step) > allowed_step + 1.0e-6 ||
          height_sign * height_step < -1.0e-6) {
        return false;
      }
      sampled_length += segment_length;
      signed_height_progress = height_sign * (sample_z - landing_ground_z);
      previous_x = sample_x;
      previous_y = sample_y;
      previous_z = sample_z;
    }
    return sampled_length + 1.0e-6 >=
               config.ground_connectivity_bridge_landing_min_length &&
           signed_height_progress + 1.0e-6 >=
               config.ground_connectivity_bridge_landing_min_height_delta;
  };
  while (!open.empty()) {
    const ConnectivityNode node = open.front();
    open.pop();
    const int view_id = node.view_id;
    const int x = view_id % width_;
    const int y = view_id / width_;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || x + dx < 0 || x + dx >= width_ ||
            y + dy < 0 || y + dy >= height_) {
          continue;
        }
        CellData &neighbor = cell_buffer_[static_cast<size_t>(
            hashIndexFromLocal(x + dx, y + dy))];
        const int neighbor_view_id = (y + dy) * width_ + x + dx;
        const double planar_step = resolution_ * std::hypot(dx, dy);
        if (neighbor.ground_candidate == 0U) {
          const bool verified_observed_empty =
              config.bridge_observed_empty_for_ground_connectivity &&
              !config.require_ground_support && !node.bridge_used &&
              neighbor.candidate_type == CellType::FREE &&
              neighbor.candidate_reason ==
                  ProjectionClassReason::EMPTY_COLUMN &&
              neighbor.clearance_verified != 0U && neighbor.traversable != 0U;
          const double bridge_distance = node.bridge_distance + planar_step;
          if (verified_observed_empty &&
              bridge_distance <=
                  config.ground_connectivity_bridge_max_length + 1.0e-6 &&
              bridge_distance + 1.0e-9 <
                  best_empty_bridge_distance[static_cast<size_t>(
                      neighbor_view_id)]) {
            best_empty_bridge_distance[static_cast<size_t>(neighbor_view_id)] =
                bridge_distance;
            const int bridge_start_view_id =
                node.is_empty_bridge ? node.bridge_start_view_id : node.view_id;
            open.push({neighbor_view_id, node.ground_z, bridge_distance,
                       bridge_start_view_id, true, false});
          }
          continue;
        }
        if (neighbor.ground_verified != 0U ||
            !std::isfinite(neighbor.ground_z_abs)) {
          continue;
        }
        if (config.require_ground_support &&
            !matchesTrustedGroundSupport(neighbor, config)) {
          continue;
        }
        if (std::abs(static_cast<double>(neighbor.ground_z_abs) -
                     config.reference_ground_z_abs) >
            config.max_ground_height_delta) {
          continue;
        }
        const double connection_distance =
            node.is_empty_bridge ? node.bridge_distance + planar_step
                                 : planar_step;
        if (node.is_empty_bridge &&
            connection_distance >
                config.ground_connectivity_bridge_max_length + 1.0e-6) {
          continue;
        }
        double allowed_height_step =
            std::max(config.max_ground_step, slope * connection_distance);
        if (node.is_empty_bridge) {
          allowed_height_step =
              std::min(allowed_height_step,
                       config.ground_connectivity_bridge_max_height_delta);
        }
        if (std::abs(static_cast<double>(neighbor.ground_z_abs) -
                     node.ground_z) > allowed_height_step + 1.0e-6) {
          continue;
        }
        const bool multi_cell_bridge =
            node.is_empty_bridge &&
            connection_distance > 2.0 * resolution_ + 1.0e-6;
        if (multi_cell_bridge &&
            !has_consistent_landing_evidence(
                node.bridge_start_view_id, neighbor_view_id, node.ground_z,
                static_cast<double>(neighbor.ground_z_abs))) {
          continue;
        }
        neighbor.ground_verified = 1U;
        neighbor.ground_support_verified =
            config.require_ground_support ? 1U : 0U;
        neighbor.ground_bridge_verified = node.is_empty_bridge ? 1U : 0U;
        open.push({neighbor_view_id, static_cast<double>(neighbor.ground_z_abs),
                   0.0, neighbor_view_id, false,
                   node.bridge_used || node.is_empty_bridge});
      }
    }
  }

  if (config.require_ground_support && config.clearance_hole_fill_en &&
      resolution_ <= config.clearance_hole_fill_max_width + 1.0e-6) {
    constexpr std::array<std::array<int, 2>, 4> kSearchDirections{{
        {{1, 0}},
        {{0, 1}},
        {{1, 1}},
        {{1, -1}},
    }};
    const int max_gap_cells = std::max(
        1, static_cast<int>(std::floor(
               (config.clearance_hole_fill_max_width + 1.0e-6) / resolution_)));
    const auto is_hole_candidate = [&](int x, int y) {
      if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
      }
      const CellData &cell =
          cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      return cell.candidate_type == CellType::OCCUPIED &&
             cell.candidate_reason ==
                 ProjectionClassReason::HEADROOM_UNVERIFIED &&
             cell.footprint_clear_eligible != 0U &&
             cell.prior_known_free != 0U && cell.ground_support_known != 0U &&
             std::isfinite(cell.ground_support_z_abs) &&
             has_connected_ground_support(x, y);
    };
    const auto is_verified_endpoint = [&](int x, int y) {
      if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
      }
      const CellData &cell =
          cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      return isTraversableCellType(cell.candidate_type) &&
             cell.clearance_verified != 0U &&
             isClearanceProofReason(cell.candidate_reason) &&
             (cell.ground_candidate == 0U || cell.ground_verified != 0U) &&
             cell.prior_known_free != 0U && cell.ground_support_known != 0U &&
             std::isfinite(cell.ground_support_z_abs) &&
             has_connected_ground_support(x, y);
    };
    const auto find_endpoint = [&](int start_x, int start_y, int dx, int dy,
                                   int &interior_count) {
      interior_count = 0;
      for (int step = 1; step <= max_gap_cells + 1; ++step) {
        const int x = start_x + step * dx;
        const int y = start_y + step * dy;
        if (is_verified_endpoint(x, y)) {
          return true;
        }
        if (!is_hole_candidate(x, y)) {
          return false;
        }
        ++interior_count;
        if (interior_count >= max_gap_cells) {
          return false;
        }
      }
      return false;
    };

    std::vector<uint8_t> bounded_hole_mask(cell_buffer_.size(), 0U);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (!is_hole_candidate(x, y)) {
          continue;
        }
        for (const auto &direction : kSearchDirections) {
          int negative_interior = 0;
          int positive_interior = 0;
          if (find_endpoint(x, y, -direction[0], -direction[1],
                            negative_interior) &&
              find_endpoint(x, y, direction[0], direction[1],
                            positive_interior) &&
              1 + negative_interior + positive_interior <= max_gap_cells) {
            bounded_hole_mask[static_cast<size_t>(hashIndexFromLocal(x, y))] =
                1U;
            break;
          }
        }
      }
    }
    // Apply only after scanning the original classifications so inferred cells
    // cannot recursively open a wider unobserved band.
    for (size_t hash_id = 0; hash_id < bounded_hole_mask.size(); ++hash_id) {
      if (bounded_hole_mask[hash_id] == 0U) {
        continue;
      }
      CellData &cell = cell_buffer_[hash_id];
      cell.candidate_type = CellType::FREE;
      cell.candidate_reason =
          ProjectionClassReason::CLEARANCE_BOUNDED_HOLE_FILL;
      cell.clearance_verified = 1U;
      cell.empty_support_verified = 1U;
      cell.ground_support_verified = 1U;
      cell.footprint_clear_eligible = 0U;
      cell.clearance_dropout_eligible = 0U;
      cell.traversable = 1U;
    }
  }

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      CellData &cell =
          cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      cell.near_field_prior_fill_eligible = 0U;
      CellType final_type = cell.candidate_type;
      ProjectionClassReason final_reason = cell.candidate_reason;
      const bool support_bridge_evidence =
          cell.footprint_clear_eligible != 0U &&
          cell.ground_candidate == 0U &&
          has_observed_support_bridge(x, y);
      if (support_bridge_evidence) {
        cell.ground_support_bridge_eligible = 1U;
        cell.ground_support_bridge_count = static_cast<uint8_t>(std::min(
            255, static_cast<int>(cell.ground_support_bridge_count) + 1));
      } else {
        cell.ground_support_bridge_eligible = 0U;
        cell.ground_support_bridge_count = 0U;
      }
      const bool support_bridge_confirmed =
          support_bridge_evidence &&
          static_cast<int>(cell.ground_support_bridge_count) >=
              std::max(1, config.observed_ground_support_bridge_hysteresis_count);
      if (support_bridge_confirmed) {
        // The current cell has no occupied voxel, while two cardinal neighbors provide
        // a continuous, height-consistent measured support surface. This is a
        // bounded one-cell bridge, not a general unknown-as-free rule.
        final_type = CellType::FREE;
        final_reason = ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK;
        cell.ground_bridge_verified = 1U;
        cell.ground_support_verified = 1U;
        cell.empty_support_verified = 1U;
        cell.clearance_verified = 1U;
        cell.traversable = 1U;
      }
      const bool disconnected_support = config.require_ground_support &&
                                        cell.clearance_verified != 0U &&
                                        !has_connected_ground_support(x, y);
      const bool surveyed_ground_with_headroom_gap =
          cell.ground_candidate != 0U && cell.ground_verified != 0U &&
          cell.footprint_clear_eligible != 0U &&
          cell.candidate_reason ==
              ProjectionClassReason::HEADROOM_UNVERIFIED;
      if (!support_bridge_confirmed && disconnected_support) {
        final_type = CellType::OCCUPIED;
        final_reason = ProjectionClassReason::GROUND_UNVERIFIED;
        cell.ground_verified = 0U;
        cell.ground_support_verified = 0U;
        cell.empty_support_verified = 0U;
        cell.clearance_verified = 0U;
        cell.traversable = 0U;
      } else if (!support_bridge_confirmed && cell.ground_candidate != 0U &&
                 cell.ground_verified == 0U) {
        final_type = CellType::OCCUPIED;
        final_reason = ProjectionClassReason::GROUND_UNVERIFIED;
        cell.clearance_verified = 0U;
      } else if (cell.ground_candidate != 0U &&
                 !surveyed_ground_with_headroom_gap) {
        cell.clearance_verified = (cell.ground_verified != 0U &&
                                   cell.candidate_type == CellType::PASSABLE)
                                      ? 1U
                                      : 0U;
        if (cell.clearance_verified != 0U &&
            cell.ground_bridge_verified != 0U) {
          final_reason = ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK;
        }
      } else if (cell.footprint_clear_eligible != 0U) {
        const double wx =
            origin_.x() + (static_cast<double>(x) + 0.5) * resolution_;
        const double wy =
            origin_.y() + (static_cast<double>(y) + 0.5) * resolution_;
        const double envelope_dx = wx - config.robot_x;
        const double envelope_dy = wy - config.robot_y;
        const double envelope_cos_yaw = std::cos(config.robot_yaw);
        const double envelope_sin_yaw = std::sin(config.robot_yaw);
        cell.envelope_body_x = static_cast<float>(
            envelope_cos_yaw * envelope_dx + envelope_sin_yaw * envelope_dy);
        cell.envelope_body_y = static_cast<float>(
            -envelope_sin_yaw * envelope_dx + envelope_cos_yaw * envelope_dy);
        const bool inside_current_footprint = insideRobotEnvelope(
            wx, wy, config, config.robot_footprint_clear_length,
            config.robot_footprint_clear_width, resolution_);
        const bool inside_surveyed_near_field =
            config.require_ground_support && config.near_field_prior_fill_en &&
            insideRobotEnvelope(wx, wy, config,
                                config.near_field_prior_fill_length,
                                config.near_field_prior_fill_width,
                                resolution_);
        cell.inside_current_footprint = inside_current_footprint ? 1U : 0U;
        cell.inside_surveyed_near_field =
            inside_surveyed_near_field ? 1U : 0U;
        bool footprint_clear_authorized = !config.require_ground_support;
        if (config.require_ground_support) {
          const bool continuous_trusted_support =
              cell.prior_known_free != 0U && cell.ground_support_known != 0U &&
              std::isfinite(cell.ground_support_z_abs) &&
              has_connected_ground_support(x, y);
          cell.continuous_ground_support =
              continuous_trusted_support ? 1U : 0U;
          // This is the support gate for the bounded clearance dropout hold in
          // commitCell(). It does not itself clear the cell or alter the public
          // ground_support_verified diagnostic.
          cell.clearance_dropout_eligible =
              continuous_trusted_support ? 1U : 0U;
          const double support_planar_distance =
              std::hypot(wx - config.robot_x, wy - config.robot_y);
          const double support_match_tolerance =
              std::max(config.ground_support_tolerance,
                       slope * support_planar_distance);
          const double support_match_error =
              std::isfinite(cell.ground_support_z_abs) &&
                      std::isfinite(config.reference_ground_z_abs)
                  ? std::abs(static_cast<double>(cell.ground_support_z_abs) -
                             config.reference_ground_z_abs)
                  : std::numeric_limits<double>::quiet_NaN();
          cell.reference_ground_z_abs =
              static_cast<float>(config.reference_ground_z_abs);
          cell.support_match_tolerance =
              static_cast<float>(support_match_tolerance);
          cell.support_match_error = static_cast<float>(support_match_error);
          const bool support_matches_robot =
              continuous_trusted_support &&
              std::isfinite(config.reference_ground_z_abs) &&
              support_match_error <=
                  support_match_tolerance + 1.0e-6;
          footprint_clear_authorized = support_matches_robot;
        }
        const bool clear_current_footprint =
            config.clear_robot_footprint_unknown && inside_current_footprint;
        if ((clear_current_footprint || inside_surveyed_near_field) &&
            footprint_clear_authorized) {
          final_type = CellType::FREE;
          final_reason = clear_current_footprint
                             ? ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR
                             : ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR;
          cell.traversable = 1U;
          if (config.require_ground_support) {
            // The current footprint or explicitly configured near-field sweep
            // may bootstrap a blind headroom column. Surveyed,
            // height-consistent support is still mandatory. Only the matching
            // ground return is tolerated; body-band or wrong-height occupied
            // evidence vetoes eligibility before this branch.
            cell.ground_support_verified = 1U;
            cell.clearance_verified = 1U;
          }
        } else if (!config.require_ground_support &&
                   config.near_field_prior_fill_en &&
                   insideRobotEnvelope(wx, wy, config,
                                       config.near_field_prior_fill_length,
                                       config.near_field_prior_fill_width)) {
          // Keep the dynamic layer fail-closed. Prior-map fusion may release
          // this zero-hit near-field cell only when static space is known free.
          cell.near_field_prior_fill_eligible = 1U;
        }
      }
      commitCell(cell, final_type, final_reason, now, config);
    }
  }
}

bool ProjectionLayer::advanceObstacleClearance(
    double now, const ProjectionLayerConfig &config) {
  std::vector<int> changed_indices;
  for (size_t hash_id = 0; hash_id < cell_buffer_.size(); ++hash_id) {
    auto &cell = cell_buffer_[hash_id];
    if (cell.occupied_clear_deadline <= 0.0 ||
        now < cell.occupied_clear_deadline) {
      continue;
    }
    cell.occupied_clear_deadline = 0.0;
    if (cell.raw_type == CellType::OCCUPIED ||
        cell.base_type != CellType::OCCUPIED) {
      continue;
    }
    cell.base_type = cell.raw_type;
    cell.pending_type = cell.raw_type;
    cell.pending_count = 0U;
    Vec3i global_id;
    hashIdToGlobalIndex(static_cast<int>(hash_id), global_id);
    const int x = global_id.x() - local_map_bound_min_i_.x();
    const int y = global_id.y() - local_map_bound_min_i_.y();
    if (x >= 0 && y >= 0 && x < width_ && y < height_) {
      changed_indices.push_back(y * width_ + x);
    }
  }
  if (!changed_indices.empty()) {
    filterMask(config, &changed_indices);
  }
  return !changed_indices.empty();
}

CellType ProjectionLayer::applyHysteresis(CellData &cell, CellType raw_type,
                                          const ProjectionLayerConfig &config) {
  // OCCUPIED enters immediately; clearing to FREE/PASSABLE waits for repeated
  // confirmation.
  if (!config.hysteresis_en || config.hysteresis_count <= 0 ||
      cell.base_type == CellType::UNKNOWN || raw_type == CellType::OCCUPIED) {
    cell.base_type = raw_type;
    cell.pending_type = raw_type;
    cell.pending_count = 0U;
    return cell.base_type;
  }

  if (raw_type == cell.base_type) {
    cell.pending_type = raw_type;
    cell.pending_count = 0U;
    return cell.base_type;
  }

  if (cell.base_type == CellType::OCCUPIED && raw_type != CellType::OCCUPIED) {
    if (cell.pending_type != raw_type) {
      cell.pending_type = raw_type;
      cell.pending_count = 1U;
    } else {
      cell.pending_count =
          static_cast<uint8_t>(std::min<int>(255, cell.pending_count + 1));
    }
    if (cell.pending_count >= static_cast<uint8_t>(config.hysteresis_count)) {
      cell.base_type = raw_type;
      cell.pending_count = 0U;
    }
    return cell.base_type;
  }

  cell.base_type = raw_type;
  cell.pending_type = raw_type;
  cell.pending_count = 0U;
  return cell.base_type;
}

void ProjectionLayer::filterMask(const ProjectionLayerConfig &config,
                                 const std::vector<int> *base_dirty_indices,
                                 double *view_time_ms) {
  if (width_ <= 0 || height_ <= 0 || cell_buffer_.empty()) {
    return;
  }

  std::vector<int> affected;
  if (base_dirty_indices) {
    constexpr int kFilterInfluenceRadius = 2;
    const size_t expected =
        static_cast<size_t>(width_) * static_cast<size_t>(height_);
    std::vector<uint8_t> update_mask(expected, 0U);
    affected.reserve(base_dirty_indices->size() * 25U);
    for (const int view_id : *base_dirty_indices) {
      if (view_id < 0 || view_id >= static_cast<int>(expected)) {
        continue;
      }
      const int cx = view_id % width_;
      const int cy = view_id / width_;
      for (int dy = -kFilterInfluenceRadius; dy <= kFilterInfluenceRadius;
           ++dy) {
        for (int dx = -kFilterInfluenceRadius; dx <= kFilterInfluenceRadius;
             ++dx) {
          const int nx = cx + dx;
          const int ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
            continue;
          }
          const int nidx = ny * width_ + nx;
          if (update_mask[static_cast<size_t>(nidx)] == 0U) {
            update_mask[static_cast<size_t>(nidx)] = 1U;
            affected.push_back(nidx);
          }
        }
      }
    }
  } else {
    const int expected = width_ * height_;
    affected.resize(static_cast<size_t>(expected));
    for (int i = 0; i < expected; ++i) {
      affected[static_cast<size_t>(i)] = i;
    }
  }

  if (config.mask_filter_en) {
    fillMask(config, affected);
    denoiseMask(config, affected);
  } else {
    for (const int view_id : affected) {
      const int x = view_id % width_;
      const int y = view_id / width_;
      CellData &cell =
          cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
      cell.hole_filled = false;
      cell.type = cell.base_type;
    }
  }

  const auto view_start = std::chrono::steady_clock::now();
  if (base_dirty_indices) {
    for (const int view_id : affected) {
      updateView(view_id, config);
    }
  } else {
    rebuildViews(config);
  }
  if (view_time_ms) {
    *view_time_ms += elapsedMs(view_start);
  }
}

void ProjectionLayer::fillMask(const ProjectionLayerConfig &config,
                               const std::vector<int> &candidate_indices) {
  for (const int view_id : candidate_indices) {
    const int x = view_id % width_;
    const int y = view_id / width_;
    CellData &cell =
        cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
    cell.hole_filled = false;
    if (x <= 0 || y <= 0 || x >= width_ - 1 || y >= height_ - 1 ||
        cell.raw_reason == ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR ||
        cell.raw_reason == ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR ||
        (cell.base_type != CellType::UNKNOWN &&
         cell.base_type != CellType::FREE)) {
      continue;
    }

    int occupied_neighbors = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const CellData &neighbor = cell_buffer_[static_cast<size_t>(
            hashIndexFromLocal(x + dx, y + dy))];
        if (neighbor.base_type == CellType::OCCUPIED) {
          ++occupied_neighbors;
        }
      }
    }
    cell.hole_filled = occupied_neighbors >= config.fill_occ_min;
  }
}

void ProjectionLayer::denoiseMask(const ProjectionLayerConfig &config,
                                  const std::vector<int> &candidate_indices) {
  for (const int view_id : candidate_indices) {
    const int x = view_id % width_;
    const int y = view_id / width_;
    CellData &cell =
        cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
    const CellType filled_type =
        cell.hole_filled ? CellType::OCCUPIED : cell.base_type;
    cell.type = filled_type;
    if (filled_type != CellType::OCCUPIED || x <= 0 || y <= 0 ||
        x >= width_ - 1 || y >= height_ - 1) {
      continue;
    }

    int occupied_neighbors = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const CellData &neighbor = cell_buffer_[static_cast<size_t>(
            hashIndexFromLocal(x + dx, y + dy))];
        if (neighbor.base_type == CellType::OCCUPIED || neighbor.hole_filled) {
          ++occupied_neighbors;
        }
      }
    }
    if (occupied_neighbors <= config.denoise_occ_max) {
      cell.type = CellType::UNKNOWN;
    }
  }
}

void ProjectionLayer::updateView(int view_id,
                                 const ProjectionLayerConfig &config) {
  const int x = view_id % width_;
  const int y = view_id / width_;
  const size_t view_index = static_cast<size_t>(view_id);
  CellData &stored =
      cell_buffer_[static_cast<size_t>(hashIndexFromLocal(x, y))];
  applyValueAndMask(stored, config);
  cells_[view_index] = stored;
  values_[view_index] = stored.value;
  mask_[view_index] = stored.mask;
}

void ProjectionLayer::rebuildViews(const ProjectionLayerConfig &config) {
  const size_t expected = static_cast<size_t>(std::max(0, width_)) *
                          static_cast<size_t>(std::max(0, height_));
  cells_.resize(expected);
  values_.resize(expected);
  mask_.resize(expected);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const size_t view_id =
          static_cast<size_t>(y) * static_cast<size_t>(width_) +
          static_cast<size_t>(x);
      updateView(static_cast<int>(view_id), config);
    }
  }
}

int ProjectionLayer::hashIndexFromLocal(int x, int y) const {
  const Vec3i global_id(local_map_bound_min_i_.x() + x,
                        local_map_bound_min_i_.y() + y, 0);
  return SlidingMap::getHashIndexFromGlobalIndex(global_id);
}

void ProjectionLayer::resetLocalMap() {
  slide_dirty_hash_ids_.clear();
  for (size_t i = 0; i < cell_buffer_.size(); ++i) {
    CellData cell;
    applyValueAndMask(cell, current_config_);
    cell_buffer_[i] = cell;
    slide_dirty_hash_ids_.push_back(static_cast<int>(i));
  }
}

void ProjectionLayer::resetCell(const int &hash_id) {
  if (hash_id < 0 || hash_id >= static_cast<int>(cell_buffer_.size())) {
    return;
  }
  CellData cell;
  applyValueAndMask(cell, current_config_);
  cell_buffer_[static_cast<size_t>(hash_id)] = cell;
  slide_dirty_hash_ids_.push_back(hash_id);
}

} // namespace rog_map
