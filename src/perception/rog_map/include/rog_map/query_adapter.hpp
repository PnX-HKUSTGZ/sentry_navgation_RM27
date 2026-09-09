#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rog_map/field_layer.hpp>
#include <rog_map/map_query_interface.hpp>

namespace rog_map {

struct MapSnapshot
{
  uint64_t sequence{0};
  uint64_t projection_sequence{0};
  uint64_t mask_sequence{0};
  uint64_t snapshot_sequence{0};
  double stamp{0.0};
  double commit_stamp{0.0};
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<uint8_t> values;
  std::vector<uint8_t> dynamic_values;
  std::vector<uint8_t> prior_occupied;
  std::vector<uint8_t> types;
  std::vector<uint8_t> raw_types;
  std::vector<uint8_t> candidate_types;
  std::vector<uint8_t> base_types;
  std::vector<uint8_t> pending_types;
  std::vector<uint8_t> pending_counts;
  std::vector<uint8_t> raw_reasons;
  std::vector<uint8_t> candidate_reasons;
  std::vector<uint8_t> hole_filled;
  std::vector<double> distances;
  std::vector<float> height_deltas;
  std::vector<float> confidence;
  std::vector<float> occupied_z_min;
  std::vector<float> occupied_z_max;
  std::vector<float> vertical_occupancy_ratio;
  std::vector<float> ground_z;
  std::vector<float> ground_support_z;
  std::vector<float> ceiling_z;
  std::vector<float> headroom;
  std::vector<float> headroom_known_ratio;
  std::vector<float> obstacle_hold_remaining;
  std::vector<uint8_t> prior_known_free;
  std::vector<uint8_t> ground_candidate;
  std::vector<uint8_t> ground_verified;
  std::vector<uint8_t> ground_support_known;
  std::vector<uint8_t> ground_support_verified;
  std::vector<uint8_t> empty_support_verified;
  std::vector<uint8_t> clearance_verified;
  std::vector<uint8_t> footprint_clear_eligible;
  std::vector<uint8_t> inside_current_footprint;
  std::vector<uint8_t> inside_surveyed_near_field;
  std::vector<uint8_t> continuous_ground_support;
  std::vector<float> reference_ground_z;
  std::vector<float> support_match_tolerance;
  std::vector<float> support_match_error;
  std::vector<float> envelope_body_x;
  std::vector<float> envelope_body_y;
  uint64_t field_sequence{0};
  double field_stamp{0.0};
  bool field_stale{true};
  double field_max_distance{3.0};
  double field_min_distance{-1.0};
  bool field_clamp_distance{true};
  InterpolationMode interpolation{InterpolationMode::BILINEAR};
};

struct QueryCounters
{
  uint64_t ok{0};
  uint64_t failed{0};
  uint64_t out_of_map{0};
  uint64_t interpolation_failed{0};
  uint64_t tf_failed{0};
  uint64_t snapshot_invalid{0};
  uint64_t field_uninitialized{0};
  uint64_t nonfinite_input{0};
  uint64_t nonfinite_output{0};
};

class QueryAdapter : public MapQueryInterface
{
public:
  void update(
    const std::shared_ptr<const MapSnapshot> & snapshot, const std::shared_ptr<DynamicLayer> & field);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;

  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;

  uint8_t value(unsigned int mx, unsigned int my) const override;
  // Returned pointer is only valid until the next snapshot update. Prefer copyValues() or snapshot().
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  std::shared_ptr<const MapSnapshot> snapshot() const;
  QueryCounters queryCounters() const;

  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;

  QueryResult query(const Eigen::Vector3d & pos) const override;
  std::vector<QueryResult> queryBatch(
    const std::vector<Eigen::Vector3d> & positions) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  QueryResult recorded(QueryResult result) const;

  mutable std::mutex mutex_;
  mutable std::mutex counters_mutex_;
  std::shared_ptr<const MapSnapshot> snapshot_;
  std::shared_ptr<DynamicLayer> field_;
  mutable QueryCounters counters_;
};

}  // namespace rog_map
