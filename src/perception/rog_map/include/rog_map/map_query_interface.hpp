#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <vector>

namespace rog_map {

enum class QueryStatus : uint8_t
{
  OK = 0,
  OUT_OF_MAP,
  SNAPSHOT_INVALID,
  FIELD_UNINITIALIZED,
  INTERPOLATION_FAILED,
  TF_FAILED,
  NONFINITE_INPUT,
  NONFINITE_OUTPUT
};

enum class ProjectedCostSource : uint8_t
{
  UNKNOWN = 0,
  DYNAMIC_PROJECTION,
  PRIOR_MAP,
  DYNAMIC_AND_PRIOR
};

enum class ProjectedCostCause : uint8_t
{
  UNKNOWN = 0,
  RAW_OCCUPIED,
  UNKNOWN_AS_OCCUPIED,
  OBSTACLE_HOLD,
  HYSTERESIS,
  MASK_HOLE_FILL,
  MASK_DENOISE_TO_UNKNOWN,
  PRIOR_MAP,
  RETAINED_OR_FILTERED
};

// Cell-level evidence copied from the immutable projection snapshot used by a
// query. Keeping it beside the projected cost lets downstream safety gates
// explain why a fused lethal value was produced without reading mutable map
// state or repeating the query against a newer snapshot.
struct ProjectionDiagnostic
{
  bool valid{false};
  ProjectedCostSource cost_source{ProjectedCostSource::UNKNOWN};
  ProjectedCostCause cost_cause{ProjectedCostCause::UNKNOWN};
  uint8_t dynamic_cost{255U};
  uint8_t cell_type{0U};
  uint8_t raw_type{0U};
  uint8_t candidate_type{0U};
  uint8_t base_type{0U};
  uint8_t pending_type{0U};
  uint8_t pending_count{0U};
  uint8_t raw_reason{0U};
  uint8_t candidate_reason{0U};
  bool hole_filled{false};
  bool prior_occupied{false};
  bool prior_known_free{false};
  bool ground_candidate{false};
  bool ground_verified{false};
  bool ground_support_known{false};
  bool ground_support_verified{false};
  bool empty_support_verified{false};
  bool clearance_verified{false};
  bool footprint_clear_eligible{false};
  bool inside_current_footprint{false};
  bool inside_surveyed_near_field{false};
  bool continuous_ground_support{false};
  float confidence{0.0F};
  float occupied_z_min{std::numeric_limits<float>::quiet_NaN()};
  float occupied_z_max{std::numeric_limits<float>::quiet_NaN()};
  float height_delta{std::numeric_limits<float>::quiet_NaN()};
  float vertical_occupancy_ratio{std::numeric_limits<float>::quiet_NaN()};
  float ground_z{std::numeric_limits<float>::quiet_NaN()};
  float ground_support_z{std::numeric_limits<float>::quiet_NaN()};
  float ceiling_z{std::numeric_limits<float>::quiet_NaN()};
  float headroom{std::numeric_limits<float>::quiet_NaN()};
  float headroom_known_ratio{std::numeric_limits<float>::quiet_NaN()};
  float obstacle_hold_remaining{0.0F};
  float reference_ground_z{std::numeric_limits<float>::quiet_NaN()};
  float support_match_tolerance{std::numeric_limits<float>::quiet_NaN()};
  float support_match_error{std::numeric_limits<float>::quiet_NaN()};
  float envelope_body_x{std::numeric_limits<float>::quiet_NaN()};
  float envelope_body_y{std::numeric_limits<float>::quiet_NaN()};
};

struct QueryResult
{
  bool ok{false};
  QueryStatus status{QueryStatus::SNAPSHOT_INVALID};
  bool projected_cost_valid{false};
  uint8_t projected_cost{255U};
  double distance{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
  uint64_t projection_sequence{0};
  uint64_t mask_sequence{0};
  uint64_t field_sequence{0};
  uint64_t snapshot_sequence{0};
  double snapshot_stamp{std::numeric_limits<double>::quiet_NaN()};
  double snapshot_commit_stamp{std::numeric_limits<double>::quiet_NaN()};
  double snapshot_processing_age_ms{std::numeric_limits<double>::quiet_NaN()};
  double field_age_ms{std::numeric_limits<double>::quiet_NaN()};
  ProjectionDiagnostic projection;
};

const char * queryStatusName(QueryStatus status);
const char * projectedCostSourceName(ProjectedCostSource source);
const char * projectedCostCauseName(ProjectedCostCause cause);
const char * projectionCellTypeName(uint8_t type);
const char * projectionClassReasonName(uint8_t reason);

class MapQueryInterface
{
public:
  virtual ~MapQueryInterface() = default;

  virtual bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const = 0;
  virtual void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const = 0;

  virtual unsigned int sizeX() const = 0;
  virtual unsigned int sizeY() const = 0;
  virtual double resolution() const = 0;
  virtual double originX() const = 0;
  virtual double originY() const = 0;

  virtual uint8_t value(unsigned int mx, unsigned int my) const = 0;
  virtual const unsigned char * values() const = 0;
  virtual bool copyValues(std::vector<unsigned char> & out) const
  {
    const auto * data = values();
    const size_t count = static_cast<size_t>(sizeX()) * static_cast<size_t>(sizeY());
    if (!data || count == 0U) {
      out.clear();
      return false;
    }
    out.assign(data, data + count);
    return true;
  }

  virtual bool isValid(unsigned int mx, unsigned int my) const = 0;
  virtual bool isFree(unsigned int mx, unsigned int my) const = 0;

  virtual QueryResult query(const Eigen::Vector3d & pos) const;
  virtual std::vector<QueryResult> queryBatch(
    const std::vector<Eigen::Vector3d> & positions) const;
  virtual bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const = 0;
};

}  // namespace rog_map
