#ifndef MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_
#define MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_

#include "minco_core/components/surveyed_ground_edge_config.hpp"
#include "rog_map/prior_map.hpp"
#include "minco_core/header.hpp"

namespace minco_planner
{

class Nav2CostmapQuery : public rog_map::MapQueryInterface
{
public:
  explicit Nav2CostmapQuery(nav2_costmap_2d::Costmap2D * costmap);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  uint8_t value(unsigned int mx, unsigned int my) const override;
  // Borrowed pointer for interface compatibility. Prefer copyValues() across threads.
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  std::vector<rog_map::QueryResult> queryBatch(
    const std::vector<Eigen::Vector3d> & positions) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

// Expands only true static lethal cells into a hard centerline-clearance mask.
// Existing inflation costs retain their source semantics (including 253 being
// non-traversable), and unknown cells remain unknown. A known-free cell next
// to an unknown island which touches a true lethal source receives a high
// non-lethal guard cost. This keeps SMAC from selecting the anti-aliased map
// boundary as a centerline while preserving fail-closed unknown semantics.
class StaticObstacleClearanceQuery : public rog_map::MapQueryInterface
{
public:
  StaticObstacleClearanceQuery(
    std::shared_ptr<rog_map::MapQueryInterface> base,
    double clearance_radius);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  std::vector<rog_map::QueryResult> queryBatch(
    const std::vector<Eigen::Vector3d> & positions) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

  size_t hardenedCellCount() const {return hardened_cell_count_;}
  size_t unknownBoundaryGuardCellCount() const
  {
    return unknown_boundary_guard_cell_count_;
  }
  double clearanceRadius() const {return clearance_radius_;}

private:
  uint8_t combinedValue(unsigned int mx, unsigned int my, uint8_t base_cost) const;
  void buildOverlay();

  std::shared_ptr<rog_map::MapQueryInterface> base_;
  double clearance_radius_{0.0};
  std::vector<uint8_t> overlay_costs_;
  mutable std::vector<unsigned char> merged_values_;
  mutable std::mutex merged_values_mutex_;
  size_t hardened_cell_count_{0U};
  size_t unknown_boundary_guard_cell_count_{0U};
};

// Adds static costs from surveyed ground-height discontinuities to a global
// 2D query. Dynamic ROG unknowns deliberately remain outside global topology.
class SurveyedGroundEdgeQuery : public rog_map::MapQueryInterface
{
public:
  SurveyedGroundEdgeQuery(
    std::shared_ptr<rog_map::MapQueryInterface> base,
    const rog_map::PriorMapData & prior_map,
    const SurveyedGroundEdgeConfig & config);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  std::vector<rog_map::QueryResult> queryBatch(
    const std::vector<Eigen::Vector3d> & positions) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

  size_t edgeCellCount() const {return edge_cell_count_;}
  size_t clearanceCellCount() const {return clearance_cell_count_;}
  size_t unsupportedCellCount() const {return unsupported_cell_count_;}

private:
  uint8_t combinedValue(unsigned int mx, unsigned int my, uint8_t base_cost) const;
  void buildOverlay(
    const rog_map::PriorMapData & prior_map,
    const SurveyedGroundEdgeConfig & config);

  std::shared_ptr<rog_map::MapQueryInterface> base_;
  std::vector<uint8_t> overlay_costs_;
  mutable std::vector<unsigned char> merged_values_;
  mutable std::mutex merged_values_mutex_;
  size_t edge_cell_count_{0U};
  size_t clearance_cell_count_{0U};
  size_t unsupported_cell_count_{0U};
};

class FrameAwareRogQuery : public rog_map::MapQueryInterface
{
public:
  FrameAwareRogQuery(
    std::shared_ptr<rog_map::MapQueryInterface> raw,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::string planning_frame,
    std::string rog_frame,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  std::vector<rog_map::QueryResult> queryBatch(
    const std::vector<Eigen::Vector3d> & positions) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  bool transformPlanningToRog(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;
  bool transformRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;
  bool rotateRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;

  std::shared_ptr<rog_map::MapQueryInterface> raw_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string planning_frame_;
  std::string rog_frame_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_
