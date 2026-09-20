#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <vector>

#include <rog_map/prior_map.hpp>
#include <rog_map/projection_layer.hpp>
#include <rog_map/query_adapter.hpp>

namespace {

constexpr int kWidth = 5;
constexpr int kHeight = 5;
constexpr double kResolution = 0.10;
const Eigen::Vector2d kOrigin(0.0, 0.0);

rog_map::ProjectionLayerConfig clearanceConfig() {
  rog_map::ProjectionLayerConfig config;
  config.clearance_check_en = true;
  config.unknown_as_occupied = true;
  config.min_observed_voxels = 1;
  config.surface_height_delta_max = 0.10;
  config.vehicle_height = 0.30;
  config.headroom_margin = 0.05;
  config.body_bottom_clearance = 0.02;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_height_delta = 0.35;
  config.max_ground_step = 0.05;
  config.max_ground_slope_deg = 28.0;
  config.ground_seed_radius = 0.18;
  config.min_headroom_known_ratio = 0.50;
  config.clearance_unknown_as_occupied = true;
  config.robot_x = 0.25;
  config.robot_y = 0.25;
  config.reference_ground_z_abs = 0.05;
  config.hysteresis_en = false;
  config.obstacle_hold_time = 0.0;
  config.mask_filter_en = false;
  return config;
}

rog_map::ColumnStats column(std::initializer_list<int> occupied_indices) {
  rog_map::ColumnStats stats;
  stats.scan_z_index_min = 0;
  stats.scan_z_min_abs = -0.10;
  stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::KNOWN_FREE);
  stats.observed_count = static_cast<int>(stats.vertical_states.size());
  for (const int index : occupied_indices) {
    stats.vertical_states.at(static_cast<size_t>(index)) =
        rog_map::VerticalVoxelState::OCCUPIED;
    ++stats.occupied_count;
    stats.occupied_z_index_min = std::min(stats.occupied_z_index_min, index);
    stats.occupied_z_index_max = std::max(stats.occupied_z_index_max, index);
    const double z =
        stats.scan_z_min_abs + static_cast<double>(index) * kResolution;
    stats.occupied_z_min_abs = std::min(stats.occupied_z_min_abs, z);
    stats.occupied_z_max_abs = std::max(stats.occupied_z_max_abs, z);
  }
  return stats;
}

rog_map::ColumnStats groundColumn(double ground_z, double resolution) {
  rog_map::ColumnStats stats;
  stats.scan_z_index_min = 0;
  stats.scan_z_min_abs = -0.5 * resolution;
  stats.vertical_states.assign(40U, rog_map::VerticalVoxelState::KNOWN_FREE);
  stats.observed_count = static_cast<int>(stats.vertical_states.size());
  const int index = static_cast<int>(std::llround(ground_z / resolution));
  stats.vertical_states.at(static_cast<size_t>(index)) =
      rog_map::VerticalVoxelState::OCCUPIED;
  stats.occupied_count = 1;
  stats.occupied_z_index_min = index;
  stats.occupied_z_index_max = index;
  const double occupied_z =
      stats.scan_z_min_abs + static_cast<double>(index) * resolution;
  stats.occupied_z_min_abs = occupied_z;
  stats.occupied_z_max_abs = occupied_z;
  return stats;
}

rog_map::ColumnStats emptyColumn(double resolution,
                                 bool body_volume_known = true) {
  rog_map::ColumnStats stats;
  stats.scan_z_index_min = 0;
  stats.scan_z_min_abs = -0.5 * resolution;
  stats.vertical_states.assign(
      40U, body_volume_known ? rog_map::VerticalVoxelState::KNOWN_FREE
                             : rog_map::VerticalVoxelState::UNKNOWN);
  if (!body_volume_known) {
    stats.vertical_states.back() = rog_map::VerticalVoxelState::KNOWN_FREE;
    stats.observed_count = 1;
  } else {
    stats.observed_count = static_cast<int>(stats.vertical_states.size());
  }
  return stats;
}

const rog_map::CellData &center(const rog_map::ProjectionLayer &layer) {
  return layer.cells().at(2U * kWidth + 2U);
}

rog_map::ProjectionLayerConfig requiredSupportConfig() {
  auto config = clearanceConfig();
  config.require_ground_support = true;
  config.ground_support_tolerance = 0.051;
  config.observed_empty_as_free = false;
  return config;
}

void attachSupport(rog_map::ColumnStats &stats, double support_z,
                   bool known_free = true, bool support_known = true) {
  stats.prior_known_free = known_free ? 1U : 0U;
  stats.ground_support_known = support_known ? 1U : 0U;
  stats.ground_support_z_abs = support_z;
}

TEST(ProjectionClearance, AcceptsThinOverheadWithKnownBodyClearance) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  config.clear_robot_footprint_unknown = true;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({6}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::OVERHEAD_CLEARANCE_OK);
  EXPECT_EQ(center(layer).ground_verified, 0U);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_EQ(center(layer).traversable, 1U);
  EXPECT_FLOAT_EQ(center(layer).headroom_known_ratio, 1.0F);
}

TEST(ProjectionClearance, ConservativeVoxelBoundariesRejectMarginalGap) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({1, 5}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
  EXPECT_NEAR(center(layer).headroom, 0.30F, 1.0e-5F);
}

TEST(ProjectionClearance, CenterEstimateAcceptsQuantizedSimulatedGap) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.headroom_voxel_inset_fraction = 0.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({1, 5}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::CLEARANCE_OK);
  EXPECT_NEAR(center(layer).headroom, 0.40F, 1.0e-5F);
}

TEST(ProjectionClearance, CenterEstimateStillRejectsLowCeiling) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.headroom_voxel_inset_fraction = 0.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({1, 4}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
}

TEST(GroundSupportProjection, IsolatedPlateCannotSelfSeedWithoutSupport) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.reference_ground_z_abs = 0.25;
  config.ground_seed_radius = 10.0;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int x, int y) {
        auto stats = (x == 2 && y == 2) ? groundColumn(0.25, kResolution)
                                        : emptyColumn(kResolution, false);
        stats.prior_known_free = 1U;
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).ground_verified, 0U);
  EXPECT_EQ(center(layer).ground_support_verified, 0U);
}

TEST(GroundSupportProjection, PitEmptyWithoutSupportStaysClosed) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.near_field_prior_fill_en = true;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = emptyColumn(kResolution, true);
                 attachSupport(stats, 0.0, true, false);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).empty_support_verified, 0U);
  EXPECT_EQ(center(layer).near_field_prior_fill_eligible, 0U);
}

TEST(GroundSupportProjection,
     CurrentFootprintBootstrapsOnlyWithMatchingContinuousSupport) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.reference_ground_z_abs = 0.0;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        attachSupport(stats, 0.0);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  EXPECT_EQ(center(layer).ground_support_verified, 1U);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  const auto &outside = layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(outside.type, rog_map::CellType::UNKNOWN);
  EXPECT_EQ(outside.traversable, 0U);
}

TEST(GroundSupportProjection,
     CurrentFootprintRasterizesIntersectingBoundaryCell) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.robot_x = 0.26;
  config.reference_ground_z_abs = 0.0;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        attachSupport(stats, 0.0);
        return stats;
      });

  // The footprint starts at x=0.16. The x=0.15 cell center is outside the
  // continuous rectangle, but its 0.10 m cell intersects the footprint.
  const auto &boundary = layer.cells().at(2U * kWidth + 1U);
  EXPECT_EQ(boundary.type, rog_map::CellType::FREE);
  EXPECT_EQ(boundary.raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  const auto &non_intersecting = layer.cells().at(2U * kWidth);
  EXPECT_EQ(non_intersecting.type, rog_map::CellType::UNKNOWN);
}

TEST(GroundSupportProjection,
     CurrentFootprintDoesNotBootstrapAtMismatchedSupportHeight) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.reference_ground_z_abs = 0.20;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        attachSupport(stats, 0.0);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::UNKNOWN);
  EXPECT_EQ(center(layer).ground_support_verified, 0U);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
}

TEST(GroundSupportProjection,
     CurrentFootprintDoesNotDilateAdjacentSupportDiscontinuity) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.reference_ground_z_abs = 0.0;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int x, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        attachSupport(stats, x == 3 ? 0.30 : 0.0);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  EXPECT_EQ(center(layer).ground_support_verified, 1U);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_EQ(center(layer).traversable, 1U);
}

TEST(GroundSupportProjection,
     CurrentFootprintDoesNotBootstrapIsolatedSupport) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.reference_ground_z_abs = 0.0;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int x, int y) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        attachSupport(stats, x == 2 && y == 2 ? 0.0 : 0.30);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::UNKNOWN);
  EXPECT_EQ(center(layer).ground_support_verified, 0U);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
}

TEST(GroundSupportProjection, CurrentFootprintNeverOverridesOccupiedEvidence) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.reference_ground_z_abs = 0.0;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.at(5U) = rog_map::VerticalVoxelState::OCCUPIED;
        stats.observed_count = 1;
        stats.occupied_count = 1;
        attachSupport(stats, 0.0);
        return stats;
      });

  EXPECT_NE(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).ground_support_verified, 0U);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
}

TEST(GroundSupportProjection,
     CurrentFootprintClearDoesNotWaitForFreeSpaceHysteresis) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.reference_ground_z_abs = 0.0;
  config.hysteresis_en = true;
  config.hysteresis_count = 2;
  config.obstacle_hold_time = 0.0;

  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.at(6U) = rog_map::VerticalVoxelState::OCCUPIED;
        stats.observed_count = 1;
        stats.occupied_count = 1;
        attachSupport(stats, 0.0);
        return stats;
      });
  ASSERT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);

  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 2.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        attachSupport(stats, 0.0);
        return stats;
      });

  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  EXPECT_EQ(center(layer).raw_type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).pending_count, 0U);
}

TEST(GroundSupportProjection,
     ZeroHitUnknownDoesNotStartMeasuredObstacleHold) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = false;
  config.reference_ground_z_abs = 0.0;
  config.hysteresis_en = false;
  config.obstacle_hold_time = 0.50;

  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        auto stats = emptyColumn(kResolution, false);
        attachSupport(stats, 0.0);
        return stats;
      });
  ASSERT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  ASSERT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);

  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.10, config, [](int, int) {
        auto stats = emptyColumn(kResolution, false);
        attachSupport(stats, 0.0);
        return stats;
      });

  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
}

TEST(GroundSupportProjection, MeasuredObstacleStillUsesConfiguredHold) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.hysteresis_en = false;
  config.obstacle_hold_time = 0.50;

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = column({1, 4});
                 attachSupport(stats, 0.05);
                 return stats;
               });
  ASSERT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  ASSERT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_BLOCKED);

  const auto verified_empty = [](int, int) {
    auto stats = emptyColumn(kResolution, true);
    attachSupport(stats, 0.05);
    return stats;
  };
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.10, config,
               verified_empty);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::EMPTY_COLUMN);
  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_GT(center(layer).occupied_clear_deadline, 1.10);

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.61, config,
               verified_empty);
  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
}

TEST(GroundSupportProjection,
     VerifiedEmptyColumnDoesNotWaitForFreeSpaceHysteresis) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.hysteresis_en = true;
  config.hysteresis_count = 2;
  config.obstacle_hold_time = 0.0;

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = column({1, 4});
                 attachSupport(stats, 0.05);
                 return stats;
               });
  ASSERT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);

  layer.update(kWidth, kHeight, kResolution, kOrigin, 2.0, config,
               [](int, int) {
                 auto stats = emptyColumn(kResolution, true);
                 attachSupport(stats, 0.05);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::EMPTY_COLUMN);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_EQ(center(layer).pending_count, 0U);
}

TEST(GroundSupportProjection,
     BoundedZeroHitHeadroomDropoutRetainsPriorClearance) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clearance_dropout_hold_time = 0.25;

  const auto verified_empty = [](int, int) {
    auto stats = emptyColumn(kResolution, true);
    attachSupport(stats, 0.05);
    return stats;
  };
  const auto zero_hit_dropout = [](int, int) {
    auto stats = emptyColumn(kResolution, false);
    attachSupport(stats, 0.05);
    return stats;
  };

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               verified_empty);
  ASSERT_EQ(center(layer).type, rog_map::CellType::FREE);
  ASSERT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::EMPTY_COLUMN);

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.10, config,
               zero_hit_dropout);
  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::CLEARANCE_DROPOUT_HOLD);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_EQ(center(layer).ground_support_verified, 1U);

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.20, config,
               zero_hit_dropout);
  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::CLEARANCE_DROPOUT_HOLD);

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.36, config,
               zero_hit_dropout);
  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
}

TEST(GroundSupportProjection,
     ClearanceDropoutHoldNeverOverridesAnOccupiedReturn) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clearance_dropout_hold_time = 0.25;
  config.min_observed_overhead_headroom_known_ratio = 0.80;

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = emptyColumn(kResolution, true);
                 attachSupport(stats, 0.05);
                 return stats;
               });
  ASSERT_EQ(center(layer).type, rog_map::CellType::FREE);

  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.10, config, [](int, int) {
        auto stats = column({6});
        std::fill(stats.vertical_states.begin(), stats.vertical_states.end(),
                  rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.at(6U) = rog_map::VerticalVoxelState::OCCUPIED;
        stats.observed_count = 1;
        attachSupport(stats, 0.05);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
}

TEST(GroundSupportProjection,
     BoundedClearanceHoleFillRequiresOpposingVerifiedEndpoints) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clearance_hole_fill_en = true;
  config.clearance_hole_fill_max_width = kResolution;

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 const bool verified_neighbor = y == 2 && (x == 1 || x == 3);
                 auto stats = emptyColumn(kResolution, verified_neighbor);
                 attachSupport(stats, 0.05);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::CLEARANCE_BOUNDED_HOLE_FILL);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_EQ(center(layer).ground_support_verified, 1U);
}

TEST(GroundSupportProjection, BoundedClearanceHoleFillSupportsTwoCellGap) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clearance_hole_fill_en = true;
  config.clearance_hole_fill_max_width = 2.0 * kResolution;

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 const bool verified_endpoint = y == 2 && (x == 1 || x == 4);
                 auto stats = emptyColumn(kResolution, verified_endpoint);
                 attachSupport(stats, 0.05);
                 return stats;
               });

  const auto &cells = layer.cells();
  EXPECT_EQ(cells.at(2U * kWidth + 2U).type, rog_map::CellType::FREE);
  EXPECT_EQ(cells.at(2U * kWidth + 3U).type, rog_map::CellType::FREE);
  EXPECT_EQ(cells.at(2U * kWidth + 2U).raw_reason,
            rog_map::ProjectionClassReason::CLEARANCE_BOUNDED_HOLE_FILL);
  EXPECT_EQ(cells.at(2U * kWidth + 3U).raw_reason,
            rog_map::ProjectionClassReason::CLEARANCE_BOUNDED_HOLE_FILL);
}

TEST(GroundSupportProjection,
     BoundedClearanceHoleFillDoesNotExceedConfiguredWidth) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clearance_hole_fill_en = true;
  config.clearance_hole_fill_max_width = 2.0 * kResolution;

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 const bool verified_endpoint = y == 2 && (x == 0 || x == 4);
                 auto stats = emptyColumn(kResolution, verified_endpoint);
                 attachSupport(stats, 0.05);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
}

TEST(GroundSupportProjection,
     BoundedClearanceHoleFillNeverOverridesOccupiedReturn) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clearance_hole_fill_en = true;
  config.clearance_hole_fill_max_width = 2.0 * kResolution;

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 if (x == 2 && y == 2) {
                   auto stats = column({1, 4});
                   attachSupport(stats, 0.05);
                   return stats;
                 }
                 const bool verified_neighbor = y == 2 && (x == 1 || x == 3);
                 auto stats = emptyColumn(kResolution, verified_neighbor);
                 attachSupport(stats, 0.05);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
}

TEST(GroundSupportProjection, EmptyWithSupportAndObservedBodyIsFree) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = emptyColumn(kResolution, true);
                 attachSupport(stats, 0.0);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).empty_support_verified, 1U);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
}

TEST(GroundSupportProjection, MatchingRampCandidatesAreVerifiedBySurvey) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.max_ground_height_delta = 0.05;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int) {
                 const double support_z = 0.05 * static_cast<double>(x);
                 auto stats = groundColumn(support_z, kResolution);
                 attachSupport(stats, support_z);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(layer).ground_verified, 1U);
  EXPECT_EQ(center(layer).ground_support_verified, 1U);
}

TEST(GroundSupportProjection,
     SurveyedVerticalStepDoesNotDilateIntoConnectedSurface) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int) {
                 const double support_z = x < 2 ? 0.0 : 0.20;
                 auto stats = groundColumn(support_z, kResolution);
                 attachSupport(stats, support_z);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(layer).ground_verified, 1U);
  EXPECT_EQ(center(layer).ground_support_verified, 1U);
}

TEST(GroundSupportProjection, IsolatedSurveyedSupportSpikeIsRejected) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 const double support_z = x == 2 && y == 2 ? 0.20 : 0.0;
                 auto stats = groundColumn(support_z, kResolution);
                 attachSupport(stats, support_z);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).ground_verified, 0U);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
}

TEST(GroundSupportProjection,
     EmptyColumnBesideStepKeepsConnectedSurveyedSupport) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int) {
                 auto stats = emptyColumn(kResolution, true);
                 attachSupport(stats, x < 2 ? 0.0 : 0.20);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).empty_support_verified, 1U);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::EMPTY_COLUMN);
}

TEST(GroundSupportProjection,
     FootprintClearAllowsContinuousSurveyedSlopeAroundRobot) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.ground_support_tolerance = 0.02;
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.40;
  config.robot_footprint_clear_width = 0.20;
  config.reference_ground_z_abs = 0.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int) {
                 auto stats = emptyColumn(kResolution, false);
                 attachSupport(stats, 0.05 * static_cast<double>(x - 2));
                 return stats;
               });

  const auto &front_footprint = layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(front_footprint.type, rog_map::CellType::FREE);
  EXPECT_EQ(front_footprint.raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  EXPECT_EQ(front_footprint.ground_support_verified, 1U);
  EXPECT_EQ(front_footprint.clearance_verified, 1U);
}

TEST(GroundSupportProjection,
     SurveyedNearFieldClearsZeroHitMotionSweepOutsideFootprint) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.near_field_prior_fill_en = true;
  config.near_field_prior_fill_length = 0.60;
  config.near_field_prior_fill_width = 0.60;
  config.reference_ground_z_abs = 0.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = emptyColumn(kResolution, false);
                 attachSupport(stats, 0.0);
                 return stats;
               });

  const auto &outside_footprint = layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(outside_footprint.type, rog_map::CellType::FREE);
  EXPECT_EQ(outside_footprint.raw_reason,
            rog_map::ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR);
  EXPECT_EQ(outside_footprint.ground_support_verified, 1U);
  EXPECT_EQ(outside_footprint.clearance_verified, 1U);
}

TEST(GroundSupportProjection, SurveyedNearFieldIsFailClosedByDefault) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.near_field_prior_fill_length = 0.60;
  config.near_field_prior_fill_width = 0.60;
  config.reference_ground_z_abs = 0.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = emptyColumn(kResolution, false);
                 attachSupport(stats, 0.0);
                 return stats;
               });

  const auto &outside_footprint = layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(outside_footprint.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(outside_footprint.raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
}

TEST(GroundSupportProjection,
     SurveyedNearFieldRejectsDiscontinuousSupportAndOccupiedReturns) {
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.near_field_prior_fill_en = true;
  config.near_field_prior_fill_length = 0.60;
  config.near_field_prior_fill_width = 0.60;
  config.reference_ground_z_abs = 0.0;

  rog_map::ProjectionLayer step_layer;
  step_layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
                    [](int x, int y) {
                      auto stats = emptyColumn(kResolution, false);
                      attachSupport(stats,
                                    x == 4 && y == 2 ? 0.10 : 0.0);
                      return stats;
                    });
  const auto &step = step_layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(step.type, rog_map::CellType::OCCUPIED);
  EXPECT_NE(step.raw_reason,
            rog_map::ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR);

  rog_map::ProjectionLayer occupied_layer;
  occupied_layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
                        [](int x, int) {
                          if (x != 4) {
                            auto stats = emptyColumn(kResolution, false);
                            attachSupport(stats, 0.0);
                            return stats;
                          }
                          auto stats = groundColumn(0.20, kResolution);
                          attachSupport(stats, 0.0);
                          return stats;
                        });
  const auto &occupied = occupied_layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(occupied.type, rog_map::CellType::OCCUPIED);
  EXPECT_NE(occupied.raw_reason,
            rog_map::ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR);
}

TEST(GroundSupportProjection, WrongHeightFloatingPlateIsBlocked) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = groundColumn(0.20, kResolution);
                 attachSupport(stats, 0.0);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
  EXPECT_EQ(center(layer).ground_verified, 0U);
}

TEST(GroundSupportProjection, PriorOccupiedVetoesMatchingCandidate) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = groundColumn(0.0, kResolution);
                 attachSupport(stats, 0.0, false, true);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).ground_verified, 0U);
}

TEST(GroundSupportProjection, DynamicLowObstacleVetoesMatchingGroundSupport) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = column({1, 4});
                 attachSupport(stats, 0.05);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
}

TEST(GroundSupportProjection, OverheadUsesLocalSupportAndKnownBodyBand) {
  auto config = requiredSupportConfig();

  rog_map::ProjectionLayer supported_layer;
  supported_layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
                         [](int, int) {
                           auto stats = column({6});
                           attachSupport(stats, 0.05);
                           return stats;
                         });
  EXPECT_EQ(center(supported_layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(supported_layer).raw_reason,
            rog_map::ProjectionClassReason::OVERHEAD_CLEARANCE_OK);

  rog_map::ProjectionLayer missing_support_layer;
  missing_support_layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0,
                               config, [](int, int) {
                                 auto stats = column({6});
                                 stats.prior_known_free = 1U;
                                 return stats;
                               });
  EXPECT_EQ(center(missing_support_layer).type, rog_map::CellType::OCCUPIED);

  rog_map::ProjectionLayer unknown_body_layer;
  unknown_body_layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        auto stats = column({6});
        std::fill(stats.vertical_states.begin(), stats.vertical_states.end(),
                  rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.at(6U) = rog_map::VerticalVoxelState::OCCUPIED;
        stats.observed_count = 1;
        attachSupport(stats, 0.05);
        return stats;
      });
  EXPECT_EQ(center(unknown_body_layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(unknown_body_layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
}

TEST(GroundSupportProjection,
     UnsupportedUnknownIsBlockedEvenIfLegacyPolicyIsOpen) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.unknown_as_occupied = false;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.scan_z_min_abs = -0.5 * kResolution;
        stats.vertical_states.assign(40U, rog_map::VerticalVoxelState::UNKNOWN);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::UNKNOWN);
  EXPECT_EQ(center(layer).mask, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
}

TEST(GroundSupportProjection,
     RequiredGateCannotBeBypassedByDisablingClearance) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clearance_check_en = false;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = groundColumn(0.20, kResolution);
                 attachSupport(stats, 0.0);
                 return stats;
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).ground_verified, 0U);
  EXPECT_EQ(center(layer).mask, 0U);
}

TEST(ProjectionClearance, RejectsThinOverheadWithoutGroundlessMode) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = false;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({6}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
}

TEST(ProjectionClearance, RejectsThinOverheadWithUnknownBodyBand) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        auto stats = column({6});
        std::fill(stats.vertical_states.begin(), stats.vertical_states.end(),
                  rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.at(6U) = rog_map::VerticalVoxelState::OCCUPIED;
        stats.observed_count = 1;
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
}

TEST(GroundSupportProjection,
     ObservedOverheadThresholdDoesNotWeakenEmptyColumnGate) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.min_headroom_known_ratio = 0.80;
  config.min_observed_overhead_headroom_known_ratio = 0.0;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int x, int y) {
        auto stats =
            x == 2 && y == 2 ? column({6}) : emptyColumn(kResolution, false);
        if (x == 2 && y == 2) {
          std::fill(stats.vertical_states.begin(), stats.vertical_states.end(),
                    rog_map::VerticalVoxelState::UNKNOWN);
          stats.vertical_states.at(6U) = rog_map::VerticalVoxelState::OCCUPIED;
          stats.observed_count = 1;
        }
        attachSupport(stats, 0.05);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::OVERHEAD_CLEARANCE_OK);
  EXPECT_FLOAT_EQ(center(layer).headroom_known_ratio, 0.0F);
  EXPECT_EQ(center(layer).clearance_verified, 1U);

  const auto &empty_neighbor = layer.cells().at(2U * kWidth + 1U);
  EXPECT_EQ(empty_neighbor.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(empty_neighbor.raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
}

TEST(GroundSupportProjection,
     ZeroObservedOverheadThresholdStillRequiresSurveyedSupport) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.min_observed_overhead_headroom_known_ratio = 0.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = column({6});
                 std::fill(stats.vertical_states.begin(), stats.vertical_states.end(),
                           rog_map::VerticalVoxelState::UNKNOWN);
                 stats.vertical_states.at(6U) = rog_map::VerticalVoxelState::OCCUPIED;
                 stats.observed_count = 1;
                 attachSupport(stats, 0.05, true, false);
                 return stats;
               });
  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason, rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
}

TEST(GroundSupportProjection,
     ZeroObservedOverheadThresholdNeverOverridesLowOccupiedReturn) {
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.min_observed_overhead_headroom_known_ratio = 0.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) {
                 auto stats = column({1, 3, 6});
                 attachSupport(stats, 0.05);
                 return stats;
               });
  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason, rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
}

TEST(ProjectionClearance, AcceptsVerifiedGroundWithEnoughHeadroom) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({1, 7}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::CLEARANCE_OK);
  EXPECT_EQ(center(layer).ground_verified, 1U);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_GT(center(layer).headroom,
            config.vehicle_height + config.headroom_margin);
}

TEST(ProjectionClearance, RejectsVerifiedGroundUnderLowCeiling) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.vehicle_height = 0.40;
  config.clear_robot_footprint_unknown = true;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({1, 6}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
  EXPECT_EQ(center(layer).ground_verified, 1U);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
}

TEST(ProjectionClearance, AcceptsHighVerticalRunWithKnownBodyClearance) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({6, 7, 8}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::OVERHEAD_CLEARANCE_OK);
  EXPECT_EQ(center(layer).ground_candidate, 0U);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_EQ(center(layer).traversable, 1U);
  EXPECT_FLOAT_EQ(center(layer).headroom_known_ratio, 1.0F);
  EXPECT_GE(center(layer).headroom,
            config.vehicle_height + config.headroom_margin);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 1U);
}

TEST(ProjectionClearance, RejectsHighVerticalRunWithoutGroundlessMode) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = false;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({6, 7, 8}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 0U);
}

TEST(ProjectionClearance, RejectsVerticalRunInsideVehicleBodyBand) {
  rog_map::ProjectionLayer layer;
  const auto config = clearanceConfig();
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({3, 4, 5}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 0U);
}

TEST(ProjectionClearance, RejectsHighVerticalRunWithUnknownBodyBand) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.min_observed_voxels = 2;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        auto stats = column({6, 7, 8});
        std::fill(stats.vertical_states.begin(), stats.vertical_states.end(),
                  rog_map::VerticalVoxelState::UNKNOWN);
        for (const size_t index : {6U, 7U, 8U}) {
          stats.vertical_states.at(index) =
              rog_map::VerticalVoxelState::OCCUPIED;
        }
        stats.observed_count = 3;
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(center(layer).traversable, 0U);
  EXPECT_FLOAT_EQ(center(layer).headroom_known_ratio, 0.0F);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 0U);
}

TEST(ProjectionClearance, ConnectsShortRampButRejectsHeightDisconnectedPlate) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.ground_seed_radius = 0.30;
  config.max_ground_step = 0.10;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 if (x == 4 && y == 4) {
                   return column({4});
                 }
                 return x < 2 ? column({1}) : column({2});
               });

  const auto &cells = layer.cells();
  EXPECT_EQ(cells.at(2U * kWidth + 3U).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(cells.at(2U * kWidth + 3U).ground_verified, 1U);
  EXPECT_EQ(cells.at(4U * kWidth + 4U).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(cells.at(4U * kWidth + 4U).raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
}

TEST(ProjectionClearance, RejectsObservedEmptyColumnWithoutGroundSupport) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.unknown_as_occupied = false;
  config.clearance_unknown_as_occupied = false;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::EMPTY_COLUMN);
  EXPECT_EQ(center(layer).traversable, 0U);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 0U);
}

TEST(ProjectionClearance, PriormapModeMayAcceptObservedEmptyColumn) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int, int) { return column({}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::EMPTY_COLUMN);
  EXPECT_EQ(center(layer).traversable, 1U);
  EXPECT_EQ(center(layer).clearance_verified, 1U);
  EXPECT_EQ(center(layer).ground_verified, 0U);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 1U);
}

TEST(ProjectionClearance,
     PriormapModeRejectsEmptyColumnWithoutVerifiedBodyVolume) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  config.min_headroom_known_ratio = 0.80;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        auto stats = column({});
        // The body interval starts above reference ground 0.05 m. Leave most of
        // that interval unknown even though the column has enough observations.
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.front() = rog_map::VerticalVoxelState::KNOWN_FREE;
        stats.observed_count = 1;
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
  EXPECT_EQ(center(layer).traversable, 0U);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 0U);
}

TEST(ProjectionClearance, KeepsUnknownColumnClosed) {
  rog_map::ProjectionLayer layer;
  const auto config = clearanceConfig();
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::UNKNOWN);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::INSUFFICIENT_OBSERVATION);
  EXPECT_EQ(center(layer).traversable, 0U);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 0U);
}

TEST(ProjectionClearance, RejectsEightCentimeterFloatingPlateInsideSeedRadius) {
  constexpr int width = 9;
  constexpr int height = 9;
  constexpr double resolution = 0.02;
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.robot_x = 0.09;
  config.robot_y = 0.09;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.08;
  // Deliberately include the plate in the height tolerance. It must still not
  // become a second seed disconnected from the lower support surface.
  config.ground_seed_tolerance = 0.10;
  config.max_ground_step = 0.03;
  config.min_headroom_known_ratio = 0.80;
  layer.update(width, height, resolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 const bool floating_plate =
                     std::abs(x - 4) <= 1 && std::abs(y - 4) <= 1;
                 return groundColumn(floating_plate ? 0.08 : 0.0, resolution);
               });

  const auto &plate = layer.cells().at(4U * width + 4U);
  EXPECT_EQ(plate.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(plate.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
  EXPECT_EQ(plate.ground_verified, 0U);
  EXPECT_EQ(layer.cells().at(4U * width + 2U).type,
            rog_map::CellType::PASSABLE);
}

TEST(ProjectionClearance, SeedsCoplanarGroundArcsSplitBySensorBlindRegion) {
  constexpr int width = 7;
  constexpr int height = 7;
  constexpr double resolution = 0.05;
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.robot_x = 0.175;
  config.robot_y = 0.175;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.11;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  layer.update(
      width, height, resolution, kOrigin, 1.0, config, [](int x, int y) {
        if (y == 3 && (x == 1 || x == 2 || x == 4 || x == 5)) {
          return groundColumn(0.0, resolution);
        }
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(40U, rog_map::VerticalVoxelState::UNKNOWN);
        return stats;
      });

  const auto &cells = layer.cells();
  EXPECT_EQ(cells.at(3U * width + 2U).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(cells.at(3U * width + 2U).ground_verified, 1U);
  EXPECT_EQ(cells.at(3U * width + 4U).type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(cells.at(3U * width + 4U).ground_verified, 1U);
  EXPECT_EQ(cells.at(3U * width + 3U).type, rog_map::CellType::UNKNOWN);
}

TEST(ProjectionClearance, BridgesOneClearanceVerifiedObservedEmptyColumn) {
  constexpr int width = 9;
  constexpr int height = 5;
  constexpr double resolution = 0.05;
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.robot_x = 0.125;
  config.robot_y = 0.125;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.12;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  config.observed_empty_as_free = true;
  config.bridge_observed_empty_for_ground_connectivity = true;
  layer.update(width, height, resolution, kOrigin, 1.0, config,
               [resolution](int x, int) {
                 if (x == 4) {
                   return emptyColumn(resolution);
                 }
                 return groundColumn(x >= 5 ? 0.05 : 0.0, resolution);
               });

  const auto &bridge = layer.cells().at(2U * width + 4U);
  const auto &ramp = layer.cells().at(2U * width + 8U);
  EXPECT_EQ(bridge.type, rog_map::CellType::FREE);
  EXPECT_EQ(bridge.raw_reason, rog_map::ProjectionClassReason::EMPTY_COLUMN);
  EXPECT_EQ(bridge.clearance_verified, 1U);
  EXPECT_EQ(ramp.type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(ramp.ground_verified, 1U);
}

TEST(ProjectionClearance, DoesNotChainAcrossTwoObservedEmptyColumns) {
  constexpr int width = 9;
  constexpr int height = 5;
  constexpr double resolution = 0.05;
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.robot_x = 0.125;
  config.robot_y = 0.125;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.12;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  config.observed_empty_as_free = true;
  config.bridge_observed_empty_for_ground_connectivity = true;
  layer.update(width, height, resolution, kOrigin, 1.0, config,
               [resolution](int x, int) {
                 if (x == 4 || x == 5) {
                   return emptyColumn(resolution);
                 }
                 return groundColumn(x >= 6 ? 0.05 : 0.0, resolution);
               });

  const auto &disconnected = layer.cells().at(2U * width + 8U);
  EXPECT_EQ(disconnected.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(disconnected.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
  EXPECT_EQ(disconnected.ground_verified, 0U);
}

TEST(ProjectionClearance, BridgesCalibratedObservedEmptyBlindRegion) {
  constexpr int width = 17;
  constexpr int height = 5;
  constexpr double resolution = 0.05;
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.robot_x = 0.125;
  config.robot_y = 0.125;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.12;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  config.observed_empty_as_free = true;
  config.bridge_observed_empty_for_ground_connectivity = true;
  config.ground_connectivity_bridge_max_length = 0.31;
  config.ground_connectivity_bridge_max_height_delta = 0.12;
  config.ground_connectivity_bridge_landing_min_length = 0.25;
  config.ground_connectivity_bridge_landing_min_height_delta = 0.04;
  layer.update(width, height, resolution, kOrigin, 1.0, config,
               [resolution](int x, int) {
                 if (x >= 4 && x <= 8) {
                   return emptyColumn(resolution);
                 }
                 const double ramp_height = x >= 13 ? 0.15 : 0.10;
                 return groundColumn(x >= 9 ? ramp_height : 0.0, resolution);
               });

  const auto &landing = layer.cells().at(2U * width + 9U);
  EXPECT_EQ(landing.type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(landing.ground_verified, 1U);
  EXPECT_EQ(landing.ground_bridge_verified, 1U);
  EXPECT_EQ(landing.raw_reason,
            rog_map::ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK);
}

TEST(ProjectionClearance, BoundedBridgeRejectsExcessiveLengthOrHeight) {
  constexpr int width = 17;
  constexpr int height = 5;
  constexpr double resolution = 0.05;
  auto config = clearanceConfig();
  config.robot_x = 0.125;
  config.robot_y = 0.125;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.12;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  config.observed_empty_as_free = true;
  config.bridge_observed_empty_for_ground_connectivity = true;
  config.ground_connectivity_bridge_max_height_delta = 0.12;
  config.ground_connectivity_bridge_landing_min_length = 0.25;
  config.ground_connectivity_bridge_landing_min_height_delta = 0.04;

  rog_map::ProjectionLayer long_gap_layer;
  config.ground_connectivity_bridge_max_length = 0.25;
  long_gap_layer.update(width, height, resolution, kOrigin, 1.0, config,
                        [resolution](int x, int) {
                          if (x >= 4 && x <= 8) {
                            return emptyColumn(resolution);
                          }
                          const double ramp_height = x >= 13 ? 0.15 : 0.10;
                          return groundColumn(x >= 9 ? ramp_height : 0.0,
                                              resolution);
                        });
  const auto &long_gap = long_gap_layer.cells().at(2U * width + 9U);
  EXPECT_EQ(long_gap.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(long_gap.ground_verified, 0U);
  EXPECT_EQ(long_gap.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);

  rog_map::ProjectionLayer high_landing_layer;
  config.ground_connectivity_bridge_max_length = 0.31;
  config.ground_connectivity_bridge_max_height_delta = 0.08;
  high_landing_layer.update(width, height, resolution, kOrigin, 1.0, config,
                            [resolution](int x, int) {
                              if (x >= 4 && x <= 8) {
                                return emptyColumn(resolution);
                              }
                              const double ramp_height = x >= 13 ? 0.15 : 0.10;
                              return groundColumn(x >= 9 ? ramp_height : 0.0,
                                                  resolution);
                            });
  const auto &high_landing = high_landing_layer.cells().at(2U * width + 9U);
  EXPECT_EQ(high_landing.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(high_landing.ground_verified, 0U);
  EXPECT_EQ(high_landing.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
}

TEST(ProjectionClearance, MultiCellBridgeRejectsHorizontalFloatingLanding) {
  constexpr int width = 17;
  constexpr int height = 5;
  constexpr double resolution = 0.05;
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.robot_x = 0.125;
  config.robot_y = 0.125;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.12;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  config.observed_empty_as_free = true;
  config.bridge_observed_empty_for_ground_connectivity = true;
  config.ground_connectivity_bridge_max_length = 0.31;
  config.ground_connectivity_bridge_max_height_delta = 0.12;
  config.ground_connectivity_bridge_landing_min_length = 0.25;
  config.ground_connectivity_bridge_landing_min_height_delta = 0.04;
  layer.update(width, height, resolution, kOrigin, 1.0, config,
               [resolution](int x, int) {
                 if (x >= 4 && x <= 8) {
                   return emptyColumn(resolution);
                 }
                 return groundColumn(x >= 9 ? 0.10 : 0.0, resolution);
               });

  const auto &plate = layer.cells().at(2U * width + 9U);
  EXPECT_EQ(plate.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(plate.ground_verified, 0U);
  EXPECT_EQ(plate.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
}

TEST(ProjectionClearance, DoesNotChainMultipleCalibratedBlindRegionBridges) {
  constexpr int width = 27;
  constexpr int height = 5;
  constexpr double resolution = 0.05;
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.robot_x = 0.125;
  config.robot_y = 0.125;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.12;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  config.observed_empty_as_free = true;
  config.bridge_observed_empty_for_ground_connectivity = true;
  config.ground_connectivity_bridge_max_length = 0.31;
  config.ground_connectivity_bridge_max_height_delta = 0.12;
  config.ground_connectivity_bridge_landing_min_length = 0.25;
  config.ground_connectivity_bridge_landing_min_height_delta = 0.04;
  layer.update(width, height, resolution, kOrigin, 1.0, config,
               [resolution](int x, int) {
                 if ((x >= 4 && x <= 8) || (x >= 15 && x <= 19)) {
                   return emptyColumn(resolution);
                 }
                 double height = 0.0;
                 if (x >= 20) {
                   height = x >= 24 ? 0.30 : 0.25;
                 } else if (x >= 9) {
                   height = x >= 13 ? 0.15 : 0.10;
                 }
                 return groundColumn(height, resolution);
               });

  const auto &first_landing = layer.cells().at(2U * width + 9U);
  const auto &second_landing = layer.cells().at(2U * width + 20U);
  EXPECT_EQ(first_landing.type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(first_landing.ground_bridge_verified, 1U);
  EXPECT_EQ(second_landing.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(second_landing.ground_verified, 0U);
  EXPECT_EQ(second_landing.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
}

TEST(ProjectionClearance, BridgeRejectsUnverifiedEmptyColumnAndFloatingPlate) {
  constexpr int width = 9;
  constexpr int height = 5;
  constexpr double resolution = 0.05;
  auto config = clearanceConfig();
  config.robot_x = 0.125;
  config.robot_y = 0.125;
  config.reference_ground_z_abs = 0.0;
  config.ground_seed_radius = 0.12;
  config.ground_seed_tolerance = 0.06;
  config.max_ground_step = 0.06;
  config.observed_empty_as_free = true;
  config.bridge_observed_empty_for_ground_connectivity = true;

  rog_map::ProjectionLayer unknown_gap_layer;
  unknown_gap_layer.update(width, height, resolution, kOrigin, 1.0, config,
                           [resolution](int x, int) {
                             if (x == 4) {
                               return emptyColumn(resolution, false);
                             }
                             return groundColumn(x >= 5 ? 0.05 : 0.0,
                                                 resolution);
                           });
  const auto &unknown_gap = unknown_gap_layer.cells().at(2U * width + 4U);
  const auto &unknown_gap_ramp = unknown_gap_layer.cells().at(2U * width + 8U);
  EXPECT_EQ(unknown_gap.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(unknown_gap.raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
  EXPECT_EQ(unknown_gap_ramp.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);

  rog_map::ProjectionLayer plate_gap_layer;
  plate_gap_layer.update(width, height, resolution, kOrigin, 1.0, config,
                         [resolution](int x, int) {
                           if (x == 4) {
                             return groundColumn(0.20, resolution);
                           }
                           return groundColumn(x >= 5 ? 0.05 : 0.0, resolution);
                         });
  const auto &plate = plate_gap_layer.cells().at(2U * width + 4U);
  const auto &plate_gap_ramp = plate_gap_layer.cells().at(2U * width + 8U);
  EXPECT_EQ(plate.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(plate.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
  EXPECT_EQ(plate_gap_ramp.raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
}

TEST(ProjectionClearance, ClearsOnlyZeroHitUnknownInsideCurrentRobotFootprint) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  EXPECT_EQ(center(layer).clearance_verified, 0U);
  EXPECT_EQ(layer.cells().front().type, rog_map::CellType::UNKNOWN);
}

TEST(ProjectionClearance,
     ClearsZeroHitHeadroomUnverifiedInsideCurrentRobotFootprint) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  const auto headroom_unverified = [](int, int) {
    rog_map::ColumnStats stats;
    stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
    stats.vertical_states.front() = rog_map::VerticalVoxelState::KNOWN_FREE;
    stats.observed_count = 1;
    return stats;
  };
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               headroom_unverified);

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  const auto &outside = layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(outside.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(outside.raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
}

TEST(ProjectionClearance, BridgesOnePriorNoDataCellFromTwoCardinalSupports)
{
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.observed_ground_support_bridge_en = true;
  config.observed_ground_support_bridge_min_neighbors = 2;
  config.observed_ground_support_bridge_max_height_delta = 0.06;
  config.observed_ground_support_bridge_hysteresis_count = 2;

  const auto scanner = [](int x, int y) {
    if (x == 2 && y == 2) {
      return emptyColumn(kResolution);
    }
    auto stats = groundColumn(0.05, kResolution);
    attachSupport(stats, 0.05);
    return stats;
  };

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config, scanner);
  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).ground_support_bridge_count, 1U);

  layer.update(kWidth, kHeight, kResolution, kOrigin, 2.0, config, scanner);
  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK);
  EXPECT_EQ(center(layer).ground_bridge_verified, 1U);
  EXPECT_EQ(center(layer).ground_support_bridge_count, 2U);
}

TEST(ProjectionClearance, ObservedSupportBridgeRejectsOccupiedReturn)
{
  rog_map::ProjectionLayer layer;
  auto config = requiredSupportConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.observed_ground_support_bridge_en = true;
  config.observed_ground_support_bridge_min_neighbors = 2;
  config.observed_ground_support_bridge_hysteresis_count = 1;

  const auto scanner = [](int x, int y) {
    if (x == 2 && y == 2) {
      auto stats = column({1, 2, 3, 4, 5});
      attachSupport(stats, 0.05);
      return stats;
    }
    auto stats = groundColumn(0.05, kResolution);
    attachSupport(stats, 0.05);
    return stats;
  };

  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config, scanner);
  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_NE(center(layer).raw_reason,
            rog_map::ProjectionClassReason::GROUND_BRIDGE_CLEARANCE_OK);
  EXPECT_EQ(center(layer).ground_support_bridge_count, 0U);
}

TEST(ProjectionClearance,
     MarksZeroHitNearFieldForPriorWithoutOpeningDynamicLayer) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.observed_empty_as_free = true;
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.near_field_prior_fill_en = true;
  config.near_field_prior_fill_length = 0.60;
  config.near_field_prior_fill_width = 0.40;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        rog_map::ColumnStats stats;
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.front() = rog_map::VerticalVoxelState::KNOWN_FREE;
        stats.observed_count = 1;
        return stats;
      });

  const auto &outside_footprint = layer.cells().at(2U * kWidth + 4U);
  EXPECT_EQ(outside_footprint.type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(outside_footprint.raw_reason,
            rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
  EXPECT_EQ(outside_footprint.near_field_prior_fill_eligible, 1U);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 4U), 0U);
}

TEST(ProjectionClearance, FootprintClearSurvivesOccupiedHoleFilling) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.05;
  config.robot_footprint_clear_width = 0.05;
  config.mask_filter_en = true;
  config.fill_occ_min = 7;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int y) {
                 if (x == 2 && y == 2) {
                   rog_map::ColumnStats stats;
                   stats.vertical_states.assign(
                       11U, rog_map::VerticalVoxelState::UNKNOWN);
                   return stats;
                 }
                 return column({6});
               });

  EXPECT_EQ(center(layer).type, rog_map::CellType::FREE);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR);
  EXPECT_FALSE(center(layer).hole_filled);
  EXPECT_EQ(layer.mask().at(2U * kWidth + 2U), 1U);
}

TEST(ProjectionClearance, FootprintClearTracksPoseWithoutLeavingAFreeTrail) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.20;
  config.robot_footprint_clear_width = 0.20;
  config.robot_x = 0.15;
  const auto unknown_scanner = [](int, int) {
    rog_map::ColumnStats stats;
    stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
    return stats;
  };
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               unknown_scanner);
  EXPECT_EQ(layer.cells().at(2U * kWidth + 1U).type, rog_map::CellType::FREE);

  config.robot_x = 0.35;
  layer.updateDirty(kWidth, kHeight, kResolution, kOrigin, 2.0, config,
                    unknown_scanner, {}, false);

  EXPECT_EQ(layer.cells().at(2U * kWidth + 1U).type,
            rog_map::CellType::UNKNOWN);
  EXPECT_EQ(layer.cells().at(2U * kWidth + 3U).type, rog_map::CellType::FREE);
}

TEST(ProjectionClearance, FootprintClearUsesRobotYawAtEnvelopeBoundary) {
  auto config = clearanceConfig();
  config.clear_robot_footprint_unknown = true;
  config.robot_footprint_clear_length = 0.40;
  config.robot_footprint_clear_width = 0.20;
  const auto unknown_scanner = [](int, int) {
    rog_map::ColumnStats stats;
    stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
    return stats;
  };

  rog_map::ProjectionLayer aligned_layer;
  config.robot_yaw = 0.0;
  aligned_layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
                       unknown_scanner);
  EXPECT_EQ(aligned_layer.cells().at(2U * kWidth + 4U).type,
            rog_map::CellType::FREE);

  rog_map::ProjectionLayer rotated_layer;
  config.robot_yaw = 0.5 * std::acos(-1.0);
  rotated_layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
                       unknown_scanner);
  EXPECT_EQ(rotated_layer.cells().at(2U * kWidth + 4U).type,
            rog_map::CellType::UNKNOWN);
}

TEST(ProjectionClearance, FootprintClearNeverOverridesAnOccupiedReturn) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.min_observed_voxels = 2;
  config.clear_robot_footprint_unknown = true;
  config.near_field_prior_fill_en = true;
  layer.update(
      kWidth, kHeight, kResolution, kOrigin, 1.0, config, [](int, int) {
        auto stats = column({6});
        stats.vertical_states.assign(11U, rog_map::VerticalVoxelState::UNKNOWN);
        stats.vertical_states.at(6U) = rog_map::VerticalVoxelState::OCCUPIED;
        stats.observed_count = 1;
        stats.occupied_count = 1;
        return stats;
      });

  EXPECT_EQ(center(layer).type, rog_map::CellType::UNKNOWN);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::INSUFFICIENT_OBSERVATION);
  EXPECT_EQ(center(layer).traversable, 0U);
  EXPECT_EQ(center(layer).near_field_prior_fill_eligible, 0U);
}

TEST(ProjectionClearance, DoesNotAddSlopeAndStepAllowances) {
  rog_map::ProjectionLayer layer;
  auto config = clearanceConfig();
  config.max_ground_step = 0.05;
  config.max_ground_slope_deg = 28.0;
  layer.update(kWidth, kHeight, kResolution, kOrigin, 1.0, config,
               [](int x, int) { return x < 2 ? column({1}) : column({2}); });

  EXPECT_EQ(center(layer).type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(center(layer).raw_reason,
            rog_map::ProjectionClassReason::GROUND_UNVERIFIED);
}

TEST(QueryAdapter, ExposesSnapshotTimestamp) {
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->sequence = 3U;
  snapshot->snapshot_sequence = 3U;
  snapshot->stamp = 42.25;
  snapshot->commit_stamp = 42.50;
  snapshot->width = 2;
  snapshot->height = 2;
  snapshot->resolution = 1.0;
  snapshot->values.assign(4U, 0U);
  snapshot->values[0] = 254U;
  snapshot->dynamic_values = {254U, 0U, 0U, 0U};
  snapshot->prior_occupied.assign(4U, 0U);
  snapshot->types.assign(4U, static_cast<uint8_t>(rog_map::CellType::FREE));
  snapshot->types[0] = static_cast<uint8_t>(rog_map::CellType::OCCUPIED);
  snapshot->raw_types.assign(4U, static_cast<uint8_t>(rog_map::CellType::FREE));
  snapshot->candidate_types = snapshot->raw_types;
  snapshot->base_types = snapshot->types;
  snapshot->pending_types = snapshot->raw_types;
  snapshot->pending_counts = {1U, 0U, 0U, 0U};
  snapshot->raw_reasons.assign(
      4U, static_cast<uint8_t>(rog_map::ProjectionClassReason::CLEARANCE_OK));
  snapshot->raw_reasons[0] =
      static_cast<uint8_t>(rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
  snapshot->candidate_reasons = snapshot->raw_reasons;
  snapshot->hole_filled = {0U, 0U, 0U, 0U};
  snapshot->headroom.assign(4U, 1.0F);
  snapshot->headroom[0] = 0.18F;
  snapshot->obstacle_hold_remaining = {0.30F, 0.0F, 0.0F, 0.0F};
  snapshot->distances.assign(4U, 1.0);

  rog_map::QueryAdapter adapter;
  adapter.update(snapshot, nullptr);
  const auto result = adapter.query(Eigen::Vector3d(0.25, 0.25, 0.0));

  ASSERT_TRUE(result.ok);
  ASSERT_TRUE(result.projected_cost_valid);
  EXPECT_EQ(result.projected_cost, 254U);
  EXPECT_DOUBLE_EQ(result.snapshot_stamp, snapshot->stamp);
  EXPECT_DOUBLE_EQ(result.snapshot_commit_stamp, snapshot->commit_stamp);
  EXPECT_DOUBLE_EQ(result.snapshot_processing_age_ms, 250.0);
  ASSERT_TRUE(result.projection.valid);
  EXPECT_EQ(result.projection.cost_source,
            rog_map::ProjectedCostSource::DYNAMIC_PROJECTION);
  EXPECT_EQ(result.projection.cost_cause,
            rog_map::ProjectedCostCause::OBSTACLE_HOLD);
  EXPECT_EQ(result.projection.cell_type,
            static_cast<uint8_t>(rog_map::CellType::OCCUPIED));
  EXPECT_EQ(
      result.projection.raw_reason,
      static_cast<uint8_t>(rog_map::ProjectionClassReason::HEADROOM_BLOCKED));
  EXPECT_FLOAT_EQ(result.projection.headroom, 0.18F);
  EXPECT_EQ(result.projection.base_type,
            static_cast<uint8_t>(rog_map::CellType::OCCUPIED));
  EXPECT_EQ(result.projection.pending_count, 1U);
  EXPECT_FLOAT_EQ(result.projection.obstacle_hold_remaining, 0.30F);
  EXPECT_STREQ(rog_map::projectionClassReasonName(result.projection.raw_reason),
               "HEADROOM_BLOCKED");
}

TEST(QueryAdapter, IdentifiesPriorMapAsFusedLethalSource) {
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->stamp = 10.0;
  snapshot->commit_stamp = 10.01;
  snapshot->width = 2;
  snapshot->height = 2;
  snapshot->resolution = 1.0;
  snapshot->values.assign(4U, 0U);
  snapshot->values[0] = 254U;
  snapshot->dynamic_values.assign(4U, 0U);
  snapshot->prior_occupied = {1U, 0U, 0U, 0U};
  snapshot->types.assign(4U, static_cast<uint8_t>(rog_map::CellType::FREE));
  snapshot->raw_reasons.assign(
      4U, static_cast<uint8_t>(rog_map::ProjectionClassReason::CLEARANCE_OK));
  snapshot->candidate_reasons = snapshot->raw_reasons;
  snapshot->distances.assign(4U, 1.0);

  rog_map::QueryAdapter adapter;
  adapter.update(snapshot, nullptr);
  const auto result = adapter.query(Eigen::Vector3d(0.25, 0.25, 0.0));

  ASSERT_TRUE(result.ok);
  ASSERT_TRUE(result.projection.valid);
  EXPECT_EQ(result.projection.cost_source,
            rog_map::ProjectedCostSource::PRIOR_MAP);
  EXPECT_EQ(result.projection.cost_cause,
            rog_map::ProjectedCostCause::PRIOR_MAP);
  EXPECT_TRUE(result.projection.prior_occupied);
  EXPECT_EQ(result.projection.dynamic_cost, 0U);
}

TEST(QueryAdapter, IdentifiesFailClosedMaskDenoise) {
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->stamp = 12.0;
  snapshot->commit_stamp = 12.01;
  snapshot->width = 2;
  snapshot->height = 2;
  snapshot->resolution = 1.0;
  snapshot->values = {254U, 0U, 0U, 0U};
  snapshot->dynamic_values = snapshot->values;
  snapshot->prior_occupied.assign(4U, 0U);
  snapshot->types = {static_cast<uint8_t>(rog_map::CellType::UNKNOWN),
                     static_cast<uint8_t>(rog_map::CellType::FREE),
                     static_cast<uint8_t>(rog_map::CellType::FREE),
                     static_cast<uint8_t>(rog_map::CellType::FREE)};
  snapshot->raw_types.assign(4U,
                             static_cast<uint8_t>(rog_map::CellType::PASSABLE));
  snapshot->candidate_types = snapshot->raw_types;
  snapshot->base_types.assign(4U,
                              static_cast<uint8_t>(rog_map::CellType::FREE));
  snapshot->base_types[0] = static_cast<uint8_t>(rog_map::CellType::OCCUPIED);
  snapshot->pending_types = snapshot->raw_types;
  snapshot->pending_counts.assign(4U, 0U);
  snapshot->raw_reasons.assign(
      4U, static_cast<uint8_t>(rog_map::ProjectionClassReason::CLEARANCE_OK));
  snapshot->candidate_reasons = snapshot->raw_reasons;
  snapshot->hole_filled.assign(4U, 0U);
  snapshot->distances.assign(4U, 0.0);

  rog_map::QueryAdapter adapter;
  adapter.update(snapshot, nullptr);
  const auto result = adapter.query(Eigen::Vector3d(0.25, 0.25, 0.0));

  ASSERT_TRUE(result.ok);
  ASSERT_TRUE(result.projection.valid);
  EXPECT_EQ(result.projection.cost_source,
            rog_map::ProjectedCostSource::DYNAMIC_PROJECTION);
  EXPECT_EQ(result.projection.cost_cause,
            rog_map::ProjectedCostCause::MASK_DENOISE_TO_UNKNOWN);
  EXPECT_STREQ(rog_map::projectedCostCauseName(result.projection.cost_cause),
               "MASK_DENOISE_TO_UNKNOWN");
}

TEST(QueryAdapter, BatchUsesOneSnapshotAndReturnsProjectedCosts) {
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->sequence = 17U;
  snapshot->snapshot_sequence = 17U;
  snapshot->stamp = 51.0;
  snapshot->width = 3;
  snapshot->height = 2;
  snapshot->resolution = 1.0;
  snapshot->values = {0U, 1U, 2U, 3U, 4U, 5U};
  snapshot->distances.assign(6U, 1.0);

  rog_map::QueryAdapter adapter;
  adapter.update(snapshot, nullptr);
  const auto results = adapter.queryBatch(
      {Eigen::Vector3d(0.25, 0.25, 0.0), Eigen::Vector3d(2.25, 1.25, 0.0)});

  ASSERT_EQ(results.size(), 2U);
  for (const auto &result : results) {
    ASSERT_TRUE(result.ok);
    ASSERT_TRUE(result.projected_cost_valid);
    EXPECT_EQ(result.snapshot_sequence, 17U);
    EXPECT_DOUBLE_EQ(result.snapshot_stamp, 51.0);
  }
  EXPECT_EQ(results[0].projected_cost, 0U);
  EXPECT_EQ(results[1].projected_cost, 5U);
}

TEST(QueryAdapter, SamplesDistanceAtPublishedCellCenter) {
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->width = 4;
  snapshot->height = 4;
  snapshot->resolution = 0.5;
  snapshot->origin_x = -1.0;
  snapshot->origin_y = -2.0;
  snapshot->values.assign(16U, 0U);
  snapshot->distances = {0.0,  1.0,  2.0,  3.0,  10.0, 11.0, 12.0, 13.0,
                         20.0, 21.0, 22.0, 23.0, 30.0, 31.0, 32.0, 33.0};
  snapshot->field_clamp_distance = false;
  snapshot->interpolation = rog_map::InterpolationMode::BILINEAR;

  rog_map::QueryAdapter adapter;
  adapter.update(snapshot, nullptr);
  const Eigen::Vector3d cell_center(
      snapshot->origin_x + 1.5 * snapshot->resolution,
      snapshot->origin_y + 2.5 * snapshot->resolution, 0.0);
  const auto result = adapter.query(cell_center);

  ASSERT_TRUE(result.ok);
  EXPECT_DOUBLE_EQ(result.distance, snapshot->distances.at(9U));
}

TEST(DynamicLayer, EvaluateMatchesStoredDistanceAtCellCenter) {
  constexpr int width = 5;
  constexpr int height = 5;
  constexpr double resolution = 0.2;
  const Eigen::Vector2d origin(-0.5, -0.5);
  std::vector<uint8_t> mask(static_cast<size_t>(width * height), 1U);
  mask.at(0U) = 0U;

  rog_map::DynamicLayer field;
  field.rebuild(width, height, resolution, origin, mask, 0.0, 3.0, -1.0, true,
                rog_map::InterpolationMode::BILINEAR);
  const auto distances = field.distances();
  const int x = 2;
  const int y = 2;
  const Eigen::Vector3d cell_center(
      origin.x() + (static_cast<double>(x) + 0.5) * resolution,
      origin.y() + (static_cast<double>(y) + 0.5) * resolution, 0.0);
  double distance = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  field.evaluate(cell_center, distance, gradient);

  EXPECT_DOUBLE_EQ(distance, distances.at(static_cast<size_t>(y * width + x)));
}

TEST(PriorMapFusion, KnownFreeFillsOnlyDynamicUnknown) {
  rog_map::PriorMapData prior;
  prior.loaded = true;
  prior.transform_ready = true;
  prior.projection_cache_ready = true;
  prior.cached_mask = {1U, 1U, 0U, 1U};
  prior.cached_free_mask = {1U, 1U, 0U, 0U};

  // Match the active configuration: unknown_as_occupied encodes UNKNOWN as
  // value 254/mask 0, exactly like OCCUPIED. The semantic mask disambiguates
  // it.
  const std::vector<uint8_t> dynamic_mask = {0U, 0U, 1U, 0U};
  const std::vector<uint8_t> dynamic_values = {254U, 254U, 0U, 254U};
  const std::vector<uint8_t> dynamic_unknown_mask = {1U, 0U, 0U, 1U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;
  rog_map::fusePriorMapProjection(true, true, prior, dynamic_mask,
                                  dynamic_values, dynamic_unknown_mask,
                                  {0U, 0U, 0U, 0U}, fused_mask, fused_values);

  EXPECT_EQ(fused_mask, (std::vector<uint8_t>{1U, 0U, 0U, 0U}));
  EXPECT_EQ(fused_values, (std::vector<uint8_t>{0U, 254U, 254U, 254U}));
}

TEST(PriorMapFusion, DoesNotTrustStaticFreeUnlessExplicitlyEnabled) {
  rog_map::PriorMapData prior;
  prior.loaded = true;
  prior.transform_ready = true;
  prior.projection_cache_ready = true;
  prior.cached_mask = {1U};
  prior.cached_free_mask = {1U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;

  rog_map::fusePriorMapProjection(true, false, prior, {0U}, {254U}, {1U}, {0U},
                                  fused_mask, fused_values);

  EXPECT_EQ(fused_mask, (std::vector<uint8_t>{0U}));
  EXPECT_EQ(fused_values, (std::vector<uint8_t>{254U}));
}

TEST(PriorMapFusion, RequiredSupportDisablesEveryTwoDimensionalFreeFill) {
  rog_map::PriorMapData prior;
  prior.loaded = true;
  prior.transform_ready = true;
  prior.projection_cache_ready = true;
  prior.cached_mask = {1U};
  prior.cached_free_mask = {1U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;

  rog_map::fusePriorMapProjection(true, true, prior, {0U}, {254U}, {1U}, {1U},
                                  fused_mask, fused_values, true);

  EXPECT_EQ(fused_mask, (std::vector<uint8_t>{0U}));
  EXPECT_EQ(fused_values, (std::vector<uint8_t>{254U}));
}

TEST(PriorMapFusion, BoundedNearFieldFillRequiresKnownStaticFree) {
  rog_map::PriorMapData prior;
  prior.loaded = true;
  prior.transform_ready = true;
  prior.projection_cache_ready = true;
  prior.cached_mask = {1U, 1U, 0U};
  prior.cached_free_mask = {1U, 0U, 0U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;

  rog_map::fusePriorMapProjection(true, false, prior, {0U, 0U, 0U},
                                  {254U, 254U, 254U}, {0U, 0U, 0U},
                                  {1U, 1U, 1U}, fused_mask, fused_values);

  EXPECT_EQ(fused_mask, (std::vector<uint8_t>{1U, 0U, 0U}));
  EXPECT_EQ(fused_values, (std::vector<uint8_t>{0U, 254U, 254U}));
}

TEST(PriorMapFusion, MissingTransformBlocksEveryCell) {
  rog_map::PriorMapData prior;
  prior.loaded = true;
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;

  rog_map::fusePriorMapProjection(true, false, prior, {1U, 1U}, {0U, 0U},
                                  {0U, 0U}, {0U, 0U}, fused_mask, fused_values);

  EXPECT_EQ(fused_mask, (std::vector<uint8_t>{0U, 0U}));
  EXPECT_EQ(fused_values, (std::vector<uint8_t>{254U, 254U}));
}

TEST(PriorMapFusion, TransformChangeInvalidatesProjectionCache) {
  rog_map::PriorMapData prior;
  prior.loaded = true;
  prior.origin_yaw = 0.0;
  prior.projection_cache_ready = true;

  EXPECT_TRUE(rog_map::updatePriorMapTransform(
      prior, rog_map::PriorMapTransform2D{1.0, 2.0, 0.25}));
  EXPECT_TRUE(prior.transform_ready);
  EXPECT_FALSE(prior.projection_cache_ready);

  prior.projection_cache_ready = true;
  EXPECT_FALSE(rog_map::updatePriorMapTransform(
      prior, rog_map::PriorMapTransform2D{1.0, 2.0, 0.25}));
  EXPECT_TRUE(prior.projection_cache_ready);

  EXPECT_TRUE(rog_map::updatePriorMapTransform(
      prior, rog_map::PriorMapTransform2D{1.1, 2.0, 0.25}));
  EXPECT_FALSE(prior.projection_cache_ready);

  EXPECT_TRUE(rog_map::invalidatePriorMapTransform(prior));
  EXPECT_FALSE(prior.transform_ready);
  EXPECT_FALSE(rog_map::invalidatePriorMapTransform(prior));
}

TEST(PriorMapLoader, PreservesMapSaverGrayAsUnknownWithStandardThreshold) {
  const auto test_dir = std::filesystem::temp_directory_path() /
                        "rog_map_prior_loader_threshold_test";
  std::filesystem::create_directories(test_dir);
  const auto pgm_path = test_dir / "map.pgm";
  const auto yaml_path = test_dir / "map.yaml";
  {
    std::ofstream pgm(pgm_path);
    pgm << "P2\n3 1\n255\n0 205 254\n";
  }
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n"
         << "resolution: 0.05\n"
         << "origin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
  }

  const auto prior = rog_map::loadPriorMap(yaml_path.string(), "");
  ASSERT_EQ(prior.occupied.size(), 3U);
  ASSERT_EQ(prior.known_free.size(), 3U);
  EXPECT_EQ(prior.occupied, (std::vector<uint8_t>{1U, 0U, 0U}));
  EXPECT_EQ(prior.known_free, (std::vector<uint8_t>{0U, 0U, 1U}));

  std::filesystem::remove_all(test_dir);
}

TEST(PriorMapLoader, GroundElevationAppliesMapToRogZTranslation) {
  const auto test_dir = std::filesystem::temp_directory_path() /
                        "rog_map_ground_elevation_transform_test";
  std::filesystem::create_directories(test_dir);
  const auto pgm_path = test_dir / "map.pgm";
  const auto yaml_path = test_dir / "map.yaml";
  {
    std::ofstream pgm(pgm_path);
    pgm << "P2\n3 3\n255\n"
        << "255 255 255\n255 255 255\n255 255 255\n";
  }
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n"
         << "resolution: 1.0\n"
         << "origin: [-1.5, -1.5, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n"
         << "ground_elevation:\n"
         << "  default_height: 0.0\n"
         << "  patches:\n"
         << "    - bounds: [0.25, -0.5, 1.25, 0.5]\n"
         << "      reference: [0.25, 0.0, 0.10]\n"
         << "      slope: [0.20, 0.0]\n";
  }

  auto prior = rog_map::loadPriorMap(yaml_path.string(), "");
  rog_map::PriorMapTransform2D transform;
  transform.tz = 0.06;
  ASSERT_TRUE(rog_map::updatePriorMapTransform(prior, transform));
  double support_z = 0.0;
  ASSERT_TRUE(rog_map::priorMapGroundSupport(prior, 0.0, 0.0, support_z));
  EXPECT_NEAR(support_z, -0.06, 1.0e-9);
  ASSERT_TRUE(rog_map::priorMapGroundSupport(prior, 0.75, 0.0, support_z));
  EXPECT_NEAR(support_z, 0.14, 1.0e-9);

  transform.pitch = 2.0e-5;
  EXPECT_TRUE(rog_map::updatePriorMapTransform(prior, transform));
  EXPECT_FALSE(prior.transform_ready);
  EXPECT_FALSE(rog_map::priorMapGroundSupport(prior, 0.0, 0.0, support_z));

  std::filesystem::remove_all(test_dir);
}

TEST(PriorMapLoader, LoadsAlignedSixteenBitGroundElevationGridFailClosed) {
  const auto test_dir = std::filesystem::temp_directory_path() /
                        "rog_map_ground_elevation_grid_test";
  std::filesystem::create_directories(test_dir);
  const auto pgm_path = test_dir / "map.pgm";
  const auto elevation_path = test_dir / "elevation.pgm";
  const auto yaml_path = test_dir / "map.yaml";
  {
    std::ofstream pgm(pgm_path);
    pgm << "P2\n3 2\n255\n255 255 255\n255 255 255\n";
  }
  {
    std::ofstream elevation(elevation_path, std::ios::binary);
    elevation << "P5\n3 2\n65535\n";
    const std::array<uint16_t, 6> values{65535U, 1100U, 1200U,
                                         1000U,  1050U, 65535U};
    for (const uint16_t value : values) {
      elevation.put(static_cast<char>((value >> 8U) & 0xffU));
      elevation.put(static_cast<char>(value & 0xffU));
    }
  }
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n"
         << "resolution: 1.0\n"
         << "origin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n"
         << "ground_elevation:\n"
         << "  default_height: 0.0\n"
         << "  grid:\n"
         << "    image: elevation.pgm\n"
         << "    scale: 0.001\n"
         << "    offset: -1.0\n"
         << "    no_data: 65535\n"
         << "  patches:\n"
         << "    - bounds: [0.0, 1.0, 1.0, 2.0]\n"
         << "      reference: [0.0, 1.0, 0.25]\n"
         << "      slope: [0.0, 0.0]\n";
  }

  auto prior = rog_map::loadPriorMap(yaml_path.string(), "");
  EXPECT_TRUE(prior.ground_elevation_grid_loaded);
  EXPECT_EQ(prior.ground_elevation_grid_width, 3);
  EXPECT_EQ(prior.ground_elevation_grid_height, 2);
  ASSERT_TRUE(
      rog_map::updatePriorMapTransform(prior, rog_map::PriorMapTransform2D{}));

  double support_z = 0.0;
  ASSERT_TRUE(rog_map::priorMapGroundSupport(prior, 0.5, 0.5, support_z));
  EXPECT_NEAR(support_z, 0.0, 1.0e-9);
  ASSERT_TRUE(rog_map::priorMapGroundSupport(prior, 1.5, 1.5, support_z));
  EXPECT_NEAR(support_z, 0.10, 1.0e-9);
  EXPECT_FALSE(rog_map::priorMapGroundSupport(prior, 2.5, 0.5, support_z));

  // A deliberately surveyed patch may authorize and override a no-data cell.
  ASSERT_TRUE(rog_map::priorMapGroundSupport(prior, 0.5, 1.5, support_z));
  EXPECT_NEAR(support_z, 0.25, 1.0e-9);

  std::filesystem::remove_all(test_dir);
}

TEST(PriorMapLoader, RejectsMismatchedGroundElevationGridDimensions) {
  const auto test_dir = std::filesystem::temp_directory_path() /
                        "rog_map_ground_elevation_grid_dimensions_test";
  std::filesystem::create_directories(test_dir);
  const auto pgm_path = test_dir / "map.pgm";
  const auto elevation_path = test_dir / "elevation.pgm";
  const auto yaml_path = test_dir / "map.yaml";
  {
    std::ofstream pgm(pgm_path);
    pgm << "P2\n2 2\n255\n255 255 255 255\n";
  }
  {
    std::ofstream elevation(elevation_path);
    elevation << "P2\n1 2\n65535\n1000 1000\n";
  }
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n"
         << "resolution: 1.0\n"
         << "origin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n"
         << "ground_elevation:\n"
         << "  default_height: 0.0\n"
         << "  grid: {image: elevation.pgm, scale: 0.001, offset: -1.0, "
            "no_data: 65535}\n";
  }

  EXPECT_THROW(rog_map::loadPriorMap(yaml_path.string(), ""),
               std::runtime_error);
  std::filesystem::remove_all(test_dir);
}

TEST(PriorMapLoader, RejectsMalformedGroundElevation) {
  const auto test_dir = std::filesystem::temp_directory_path() /
                        "rog_map_ground_elevation_malformed_test";
  std::filesystem::create_directories(test_dir);
  const auto pgm_path = test_dir / "map.pgm";
  const auto yaml_path = test_dir / "map.yaml";
  {
    std::ofstream pgm(pgm_path);
    pgm << "P2\n1 1\n255\n255\n";
  }
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n"
         << "resolution: 1.0\n"
         << "origin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n"
         << "ground_elevation:\n"
         << "  patches: []\n";
  }

  EXPECT_THROW(rog_map::loadPriorMap(yaml_path.string(), ""),
               std::runtime_error);
  std::filesystem::remove_all(test_dir);
}

TEST(PriorMapLoader, SlidingProjectionKeepsGroundSupportAligned) {
  const auto test_dir = std::filesystem::temp_directory_path() /
                        "rog_map_ground_elevation_cache_test";
  std::filesystem::create_directories(test_dir);
  const auto pgm_path = test_dir / "map.pgm";
  const auto yaml_path = test_dir / "map.yaml";
  {
    std::ofstream pgm(pgm_path);
    pgm << "P2\n5 3\n255\n";
    for (int index = 0; index < 15; ++index) {
      pgm << "255 ";
    }
    pgm << "\n";
  }
  {
    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n"
         << "resolution: 1.0\n"
         << "origin: [0.0, 0.0, 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n"
         << "ground_elevation:\n"
         << "  default_height: 0.0\n"
         << "  patches:\n"
         << "    - bounds: [2.0, 0.0, 5.0, 3.0]\n"
         << "      reference: [2.0, 0.0, 0.20]\n"
         << "      slope: [0.10, 0.0]\n";
  }

  auto prior = rog_map::loadPriorMap(yaml_path.string(), "");
  rog_map::PriorMapTransform2D transform;
  transform.tz = 0.10;
  ASSERT_TRUE(rog_map::updatePriorMapTransform(prior, transform));
  ASSERT_TRUE(rog_map::refreshPriorMapProjectionCache(prior, 0, 0, 3, 3, 1.0,
                                                      0.0, 0.0));
  ASSERT_EQ(prior.cached_ground_support_mask.size(), 9U);
  EXPECT_EQ(prior.cached_ground_support_mask.at(3U), 1U);
  EXPECT_NEAR(prior.cached_ground_support_z.at(3U), -0.10, 1.0e-6);
  EXPECT_NEAR(prior.cached_ground_support_z.at(5U), 0.15, 1.0e-6);

  ASSERT_TRUE(rog_map::refreshPriorMapProjectionCache(prior, 1, 0, 3, 3, 1.0,
                                                      1.0, 0.0));
  EXPECT_NEAR(prior.cached_ground_support_z.at(3U), -0.10, 1.0e-6);
  EXPECT_NEAR(prior.cached_ground_support_z.at(4U), 0.15, 1.0e-6);
  EXPECT_NEAR(prior.cached_ground_support_z.at(5U), 0.25, 1.0e-6);

  std::filesystem::remove_all(test_dir);
}

} // namespace
