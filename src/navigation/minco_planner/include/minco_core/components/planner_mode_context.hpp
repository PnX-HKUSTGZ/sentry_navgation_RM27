#ifndef MINCO_PLANNER__PLANNER_MODE_CONTEXT_HPP_
#define MINCO_PLANNER__PLANNER_MODE_CONTEXT_HPP_

#include "minco_core/components/surveyed_ground_edge_config.hpp"
#include "minco_core/components/footprint_geometry.hpp"
#include "rog_map/prior_map.hpp"
#include "minco_core/header.hpp"

namespace minco_planner {

class StaticObstacleClearanceQuery;

enum class PlannerMode
{
  PRIORMAP,
  EXPLORATION
};

struct PlannerModeParams
{
  std::string planner_mode{"PRIORMAP"};
  std::string map_frame{"map"};
  std::string rog_frame{"camera_init"};

  bool priormap_use_nav2_global_search{true};
  // Use measured ROG occupied cells as a local dynamic hard mask during the
  // otherwise static global search. UNKNOWN/frontier cells are filtered by
  // Astar/SMAC and never become topology obstacles through this switch.
  bool priormap_dynamic_global_obstacle_enable{false};
  double priormap_dynamic_global_collision_distance{0.0};
  bool priormap_clip_seed_by_rog_boundary{true};
  double priormap_rog_boundary_margin{0.8};
  double priormap_rog_boundary_sample_step{0.1};
  double priormap_static_obstacle_clearance_radius{0.0};
  std::string priormap_static_clearance_mode{"circle"};
  std::vector<Eigen::Vector2d> priormap_static_footprint{};
  StaticOverlapPolicy static_overlap;
  double priormap_static_grid_guard{0.0};
  bool priormap_ground_edge_avoidance_enable{false};
  std::string priormap_ground_edge_yaml_path{};
  std::string priormap_ground_edge_pgm_path{};
  std::string priormap_ground_edge_frame{"map"};
  SurveyedGroundEdgeConfig priormap_ground_edge_config{};

  double exploration_boundary_margin{0.8};
  double exploration_boundary_sample_step{0.1};
  bool exploration_unknown_as_occupied{true};
  bool exploration_prefer_goal_direction{true};
};

class PlannerModeContext
{
public:
  void configure(const PlannerModeParams & params,
    const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
    nav2_costmap_2d::Costmap2DROS * costmap_ros,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock);

  void rebuildQueries(const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
    nav2_costmap_2d::Costmap2DROS * costmap_ros,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock);

  PlannerMode mode() const { return mode_; }
  const std::string & planningFrame() const { return planning_frame_; }
  const std::string & outputFrame() const { return output_frame_; }
  const std::string & mapFrame() const { return map_frame_; }
  const std::string & rogFrame() const { return rog_frame_; }

  bool directOdomPose() const { return direct_odom_pose_; }

  bool clipSeedByRogBoundary() const { return params_.priormap_clip_seed_by_rog_boundary; }
  bool dynamicGlobalObstacleEnabled() const
  {
    return params_.priormap_dynamic_global_obstacle_enable;
  }
  double dynamicGlobalCollisionDistance() const
  {
    return params_.priormap_dynamic_global_collision_distance;
  }
  double rogBoundaryMargin() const { return params_.priormap_rog_boundary_margin; }
  double rogBoundarySampleStep() const { return params_.priormap_rog_boundary_sample_step; }

  double explorationBoundaryMargin() const { return params_.exploration_boundary_margin; }
  double explorationBoundarySampleStep() const { return params_.exploration_boundary_sample_step; }
  bool explorationUnknownAsOccupied() const { return params_.exploration_unknown_as_occupied; }
  bool explorationPreferGoalDirection() const { return params_.exploration_prefer_goal_direction; }

  std::shared_ptr<rog_map::MapQueryInterface> globalQuery() const { return global_query_; }
  std::shared_ptr<rog_map::MapQueryInterface> dynamicQuery() const { return dynamic_query_; }
  std::shared_ptr<rog_map::MapQueryInterface> staticQuery() const { return static_query_; }
  std::shared_ptr<rog_map::MapQueryInterface> sparsifyQuery() const { return sparsify_query_; }
  bool usesPolygonStaticClearance() const
  {
    return mode_ == PlannerMode::PRIORMAP && params_.priormap_static_clearance_mode == "polygon";
  }
  void updateGlobalFootprintYaw(double yaw) const;

private:
  PlannerModeParams params_{};
  PlannerMode mode_{PlannerMode::PRIORMAP};
  std::string planning_frame_{"map"};
  std::string output_frame_{"map"};
  std::string map_frame_{"map"};
  std::string rog_frame_{"camera_init"};
  bool direct_odom_pose_{false};
  bool ground_edge_prior_ready_{false};
  rog_map::PriorMapData ground_edge_prior_{};

  std::shared_ptr<rog_map::MapQueryInterface> global_query_;
  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query_;
  std::shared_ptr<rog_map::MapQueryInterface> static_query_;
  std::shared_ptr<rog_map::MapQueryInterface> sparsify_query_;
  std::shared_ptr<StaticObstacleClearanceQuery> static_clearance_query_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNER_MODE_CONTEXT_HPP_
