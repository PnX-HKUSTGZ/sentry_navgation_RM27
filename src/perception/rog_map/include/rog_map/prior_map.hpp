#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rog_map {

struct PriorMapTransform2D {
  double tx{0.0};
  double ty{0.0};
  double yaw{0.0};
  double tz{0.0};
  double roll{0.0};
  double pitch{0.0};
};

struct GroundElevationPatch {
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
  double reference_x{0.0};
  double reference_y{0.0};
  double reference_z{0.0};
  double slope_x{0.0};
  double slope_y{0.0};
};

struct PriorMapData {
  bool loaded{false};
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  bool negate{false};
  double occupied_thresh{0.65};
  double free_thresh{0.25};
  std::vector<uint8_t> occupied;
  std::vector<uint8_t> known_free;

  bool ground_elevation_loaded{false};
  double ground_elevation_default_height{0.0};
  bool ground_elevation_grid_loaded{false};
  int ground_elevation_grid_width{0};
  int ground_elevation_grid_height{0};
  int ground_elevation_grid_max_value{0};
  double ground_elevation_grid_scale{1.0};
  double ground_elevation_grid_offset{0.0};
  uint16_t ground_elevation_grid_no_data{0U};
  std::vector<uint16_t> ground_elevation_grid;
  std::vector<GroundElevationPatch> ground_elevation_patches;

  bool transform_ready{false};
  PriorMapTransform2D fixed_transform{};
  double fixed_transform_cos{1.0};
  double fixed_transform_sin{0.0};
  double fast_origin_yaw_cos{1.0};
  double fast_origin_yaw_sin{0.0};

  bool projection_cache_ready{false};
  int cached_min_x{0};
  int cached_min_y{0};
  int cached_width{0};
  int cached_height{0};
  double cached_resolution{0.0};
  std::vector<uint8_t> cached_mask;
  std::vector<uint8_t> cached_free_mask;
  std::vector<uint8_t> cached_ground_support_mask;
  std::vector<float> cached_ground_support_z;
  std::vector<uint8_t> scratch_mask;
  std::vector<uint8_t> scratch_free_mask;
  std::vector<uint8_t> scratch_ground_support_mask;
  std::vector<float> scratch_ground_support_z;
};

PriorMapData loadPriorMap(const std::string &yaml_path,
                          const std::string &pgm_path);

bool priorMapOccupied(const PriorMapData &prior_map, double map_x,
                      double map_y);

// Query surveyed support directly in the prior map frame. Unlike
// priorMapGroundSupport(), this does not require the runtime prior-to-ROG TF.
bool priorMapGroundSupportAtMapPoint(const PriorMapData &prior_map,
                                     double map_x, double map_y,
                                     double &support_z_map);

bool priorMapGroundSupport(const PriorMapData &prior_map, double rog_x,
                           double rog_y, double &support_z_rog);

void transformPriorMapPoint(const PriorMapTransform2D &transform, double rog_x,
                            double rog_y, double &map_x, double &map_y);

bool updatePriorMapTransform(PriorMapData &prior_map,
                             const PriorMapTransform2D &transform);

bool invalidatePriorMapTransform(PriorMapData &prior_map);

bool refreshPriorMapProjectionCache(PriorMapData &prior_map, int min_global_x,
                                    int min_global_y, int width, int height,
                                    double resolution, double origin_x,
                                    double origin_y);

void fusePriorMapProjection(
    bool prior_enabled, bool free_fills_unknown, const PriorMapData &prior_map,
    const std::vector<uint8_t> &dynamic_mask,
    const std::vector<uint8_t> &dynamic_values,
    const std::vector<uint8_t> &dynamic_unknown_mask,
    const std::vector<uint8_t> &dynamic_near_field_prior_fill_mask,
    std::vector<uint8_t> &fused_mask, std::vector<uint8_t> &fused_values,
    bool require_ground_support = false);

} // namespace rog_map
