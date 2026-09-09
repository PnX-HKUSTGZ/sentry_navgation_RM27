#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include <rog_map/rog_map_core/common_lib.hpp>
#include <rog_map/rog_map_core/sliding_map.h>

namespace rog_map {

enum class CellType : uint8_t {
  UNKNOWN = 0,
  FREE = 1,
  PASSABLE = 2,
  OCCUPIED = 3
};

enum class ProjectionClassReason : uint8_t {
  INSUFFICIENT_OBSERVATION = 0,
  EMPTY_COLUMN,
  ROBOT_FOOTPRINT_CLEAR,
  THIN_SURFACE,
  SOLID_VERTICAL_WALL,
  HOLLOW_TUNNEL,
  AMBIGUOUS_OCCUPIED,
  GROUND_UNVERIFIED,
  HEADROOM_UNVERIFIED,
  HEADROOM_BLOCKED,
  OVERHEAD_CLEARANCE_OK,
  CLEARANCE_OK,
  GROUND_BRIDGE_CLEARANCE_OK,
  CLEARANCE_DROPOUT_HOLD,
  CLEARANCE_BOUNDED_HOLE_FILL,
  SURVEYED_NEAR_FIELD_CLEAR
};

enum class VerticalVoxelState : uint8_t {
  UNKNOWN = 0,
  KNOWN_FREE = 1,
  OCCUPIED = 2
};

struct CellData {
  CellType type{CellType::UNKNOWN};
  CellType raw_type{CellType::UNKNOWN};
  CellType candidate_type{CellType::UNKNOWN};
  CellType base_type{CellType::UNKNOWN};
  CellType pending_type{CellType::UNKNOWN};
  uint8_t value{255};
  uint8_t mask{1};
  uint8_t pending_count{0};
  float confidence{0.0f};
  float occupied_z_min_abs{std::numeric_limits<float>::quiet_NaN()};
  float occupied_z_max_abs{std::numeric_limits<float>::quiet_NaN()};
  float height_delta{0.0f};
  float vertical_occupancy_ratio{0.0f};
  ProjectionClassReason raw_reason{
      ProjectionClassReason::INSUFFICIENT_OBSERVATION};
  ProjectionClassReason candidate_reason{
      ProjectionClassReason::INSUFFICIENT_OBSERVATION};
  float ground_z_abs{std::numeric_limits<float>::quiet_NaN()};
  float ceiling_z_abs{std::numeric_limits<float>::quiet_NaN()};
  float headroom{std::numeric_limits<float>::infinity()};
  float headroom_known_ratio{0.0f};
  uint8_t traversable{0};
  uint8_t ground_candidate{0};
  uint8_t ground_verified{0};
  uint8_t ground_bridge_verified{0};
  uint8_t prior_known_free{0};
  uint8_t ground_support_known{0};
  uint8_t ground_support_verified{0};
  uint8_t empty_support_verified{0};
  float ground_support_z_abs{std::numeric_limits<float>::quiet_NaN()};
  uint8_t clearance_verified{0};
  uint8_t footprint_clear_eligible{0};
  uint8_t clearance_dropout_eligible{0};
  uint8_t near_field_prior_fill_eligible{0};
  uint8_t inside_current_footprint{0};
  uint8_t inside_surveyed_near_field{0};
  uint8_t continuous_ground_support{0};
  float reference_ground_z_abs{std::numeric_limits<float>::quiet_NaN()};
  float support_match_tolerance{std::numeric_limits<float>::quiet_NaN()};
  float support_match_error{std::numeric_limits<float>::quiet_NaN()};
  float envelope_body_x{std::numeric_limits<float>::quiet_NaN()};
  float envelope_body_y{std::numeric_limits<float>::quiet_NaN()};
  bool hole_filled{false};
  float last_hit_time{0.0f};
  float last_update_time{0.0f};
  double occupied_clear_deadline{0.0};
  double clearance_dropout_deadline{0.0};
};

struct ProjectionLayerConfig {
  bool unknown_as_occupied{false};
  int min_observed_voxels{2};
  double surface_height_delta_max{0.10};
  double wall_height_delta_min{0.20};
  double wall_occupancy_ratio_min{0.80};
  double tunnel_height_delta_min{0.24};
  double tunnel_height_delta_max{0.40};
  double tunnel_occupancy_ratio_max{0.55};
  uint8_t passable_cost{50};
  bool passable_as_free{true};
  bool hysteresis_en{true};
  int hysteresis_count{2};
  double obstacle_hold_time{0.0};
  double clearance_dropout_hold_time{0.0};
  bool clearance_hole_fill_en{false};
  double clearance_hole_fill_max_width{0.05};
  bool mask_filter_en{true};
  int fill_occ_min{5};
  int denoise_occ_max{0};
  bool clearance_check_en{false};
  bool observed_empty_as_free{false};
  bool require_ground_support{false};
  double ground_support_tolerance{0.08};
  bool bridge_observed_empty_for_ground_connectivity{false};
  double ground_connectivity_bridge_max_length{0.10};
  double ground_connectivity_bridge_max_height_delta{0.06};
  double ground_connectivity_bridge_landing_min_length{0.25};
  double ground_connectivity_bridge_landing_min_height_delta{0.04};
  bool clear_robot_footprint_unknown{false};
  double robot_footprint_clear_length{0.30};
  double robot_footprint_clear_width{0.20};
  bool near_field_prior_fill_en{false};
  double near_field_prior_fill_length{0.80};
  double near_field_prior_fill_width{0.50};
  double robot_origin_to_ground{0.14};
  double vehicle_height{0.35};
  double headroom_margin{0.05};
  double headroom_voxel_inset_fraction{0.5};
  double body_bottom_clearance{0.04};
  double ground_seed_tolerance{0.05};
  double max_ground_height_delta{0.35};
  double max_ground_step{0.05};
  double max_ground_slope_deg{28.0};
  double ground_seed_radius{0.10};
  double min_headroom_known_ratio{0.80};
  double min_observed_overhead_headroom_known_ratio{0.80};
  bool clearance_unknown_as_occupied{true};
  double robot_x{0.0};
  double robot_y{0.0};
  double robot_yaw{0.0};
  double reference_ground_z_abs{0.0};
};

struct ColumnStats {
  int observed_count{0};
  int occupied_count{0};
  int occupied_z_index_min{std::numeric_limits<int>::max()};
  int occupied_z_index_max{std::numeric_limits<int>::min()};
  double occupied_z_min_abs{std::numeric_limits<double>::infinity()};
  double occupied_z_max_abs{-std::numeric_limits<double>::infinity()};
  double last_hit_time{0.0};
  double last_update_time{0.0};
  int scan_z_index_min{0};
  double scan_z_min_abs{0.0};
  uint8_t prior_known_free{0};
  uint8_t ground_support_known{0};
  double ground_support_z_abs{std::numeric_limits<double>::quiet_NaN()};
  std::vector<VerticalVoxelState> vertical_states;
};

struct ProjectionUpdateStats {
  double update_full_time_ms{0.0};
  double update_dirty_time_ms{0.0};
  double mask_filter_time_ms{0.0};
  double value_mask_time_ms{0.0};
  double thin_surface_count{0.0};
  double vertical_wall_count{0.0};
  double hollow_tunnel_count{0.0};
  double ambiguous_occupied_count{0.0};
  double empty_column_count{0.0};
  double clearance_dropout_hold_count{0.0};
  double clearance_hole_fill_count{0.0};
  double insufficient_observation_count{0.0};
};

struct ProjectionSlideResult {
  bool window_moved{false};
  bool full_refresh_required{false};
  std::vector<int> dirty_columns;
};

class ProjectionLayer : protected SlidingMap {
public:
  using ColumnScanner = std::function<ColumnStats(int gx, int gy)>;

  ProjectionSlideResult syncSlidingWindow(int width, int height,
                                          double resolution,
                                          const Eigen::Vector2i &min_id,
                                          const Eigen::Vector2d &origin,
                                          const ProjectionLayerConfig &config);

  void update(int width, int height, double resolution,
              const Eigen::Vector2d &origin, double now,
              const ProjectionLayerConfig &config, const ColumnScanner &scanner,
              ProjectionUpdateStats *stats = nullptr);

  void updateFull(int width, int height, double resolution,
                  const Eigen::Vector2d &origin, double now,
                  const ProjectionLayerConfig &config,
                  const ColumnScanner &scanner,
                  ProjectionUpdateStats *stats = nullptr);

  void updateDirty(int width, int height, double resolution,
                   const Eigen::Vector2d &origin, double now,
                   const ProjectionLayerConfig &config,
                   const ColumnScanner &scanner,
                   const std::vector<int> &dirty_columns,
                   bool force_full_refresh,
                   ProjectionUpdateStats *stats = nullptr);

  bool advanceObstacleClearance(double now,
                                const ProjectionLayerConfig &config);

  bool matchesGeometry(int width, int height, double resolution,
                       const Eigen::Vector2d &origin) const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  const Eigen::Vector2d &origin() const { return origin_; }

  const std::vector<CellData> &cells() const { return cells_; }
  const std::vector<uint8_t> &values() const { return values_; }
  const std::vector<uint8_t> &mask() const { return mask_; }
  bool empty() const { return values_.empty(); }
  size_t storageCapacity() const { return cell_buffer_.size(); }

private:
  static void applyValueAndMask(CellData &cell,
                                const ProjectionLayerConfig &config);
  static void collectClassificationStats(const std::vector<CellData> &cells,
                                         ProjectionUpdateStats *stats);
  CellType applyHysteresis(CellData &cell, CellType raw_type,
                           const ProjectionLayerConfig &config);
  void stageOneCell(int x, int y, double now,
                    const ProjectionLayerConfig &config,
                    const ColumnScanner &scanner);
  void resolveGroundConnectivityAndCommit(double now,
                                          const ProjectionLayerConfig &config);
  void commitCell(CellData &cell, CellType raw_type,
                  ProjectionClassReason raw_reason, double now,
                  const ProjectionLayerConfig &config);
  void filterMask(const ProjectionLayerConfig &config,
                  const std::vector<int> *base_dirty_indices,
                  double *view_time_ms = nullptr);
  void fillMask(const ProjectionLayerConfig &config,
                const std::vector<int> &candidate_indices);
  void denoiseMask(const ProjectionLayerConfig &config,
                   const std::vector<int> &candidate_indices);
  void updateView(int view_id, const ProjectionLayerConfig &config);
  void rebuildViews(const ProjectionLayerConfig &config);
  int hashIndexFromLocal(int x, int y) const;
  void resetLocalMap() override;
  void resetCell(const int &hash_id) override;

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Eigen::Vector2d origin_{0.0, 0.0};
  std::vector<CellData> cells_;
  std::vector<uint8_t> values_;
  std::vector<uint8_t> mask_;
  std::vector<CellData> cell_buffer_;
  std::vector<int> slide_dirty_hash_ids_;
  ProjectionLayerConfig current_config_;
  bool initialized_{false};
  bool view_rebuild_required_{false};
};

} // namespace rog_map
