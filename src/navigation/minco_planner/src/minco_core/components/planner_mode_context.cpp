#include "minco_core/components/planner_mode_context.hpp"

#include <cctype>

namespace minco_planner {

void PlannerModeContext::configure(const PlannerModeParams & params,
  const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
  nav2_costmap_2d::Costmap2DROS * costmap_ros,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock)
{
  params_ = params;
  ground_edge_prior_ready_ = false;
  ground_edge_prior_ = rog_map::PriorMapData{};
  map_frame_ = params_.map_frame.empty() ? "map" : params_.map_frame;
  rog_frame_ = params_.rog_frame.empty() ? "camera_init" : params_.rog_frame;

  std::string mode_upper = params_.planner_mode;
  std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  mode_ = PlannerMode::PRIORMAP;
  if (mode_upper == "EXPLORATION") {
    mode_ = PlannerMode::EXPLORATION;
  } else if (mode_upper != "PRIORMAP") {
    RCLCPP_WARN(logger,
      "[MincoPlanner] Unknown planner_mode='%s', fallback to PRIORMAP.",
      params_.planner_mode.c_str());
  }

  if (mode_ == PlannerMode::PRIORMAP) {
    planning_frame_ = map_frame_;
    output_frame_ = map_frame_;
    direct_odom_pose_ = false;
    if (!params_.priormap_use_nav2_global_search) {
      RCLCPP_WARN(logger,
        "[MincoPlanner] priormap.use_nav2_global_search=false is unsupported by planner_mode=PRIORMAP; "
        "forcing Nav2 costmap global search to preserve map semantics.");
    }
    if (params_.priormap_ground_edge_avoidance_enable) {
      if (params_.priormap_ground_edge_frame != map_frame_) {
        throw std::invalid_argument(
                "priormap.ground_edge_avoidance requires the prior-map frame "
                "to match frames.map_frame");
      }
      ground_edge_prior_ = rog_map::loadPriorMap(
        params_.priormap_ground_edge_yaml_path,
        params_.priormap_ground_edge_pgm_path);
      if (!ground_edge_prior_.ground_elevation_loaded) {
        throw std::invalid_argument(
                "priormap.ground_edge_avoidance requires ground_elevation in the selected map YAML");
      }
      ground_edge_prior_ready_ = true;
    }
  } else {
    planning_frame_ = rog_frame_;
    output_frame_ = rog_frame_;
    direct_odom_pose_ = true;
  }

  rebuildQueries(raw_rog_query, costmap_ros, tf, logger, clock);
}

void PlannerModeContext::rebuildQueries(const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
  nav2_costmap_2d::Costmap2DROS * costmap_ros,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock)
{
  if (mode_ == PlannerMode::PRIORMAP) {
    global_query_ = nullptr;
    if (costmap_ros && costmap_ros->getCostmap()) {
      global_query_ = std::make_shared<Nav2CostmapQuery>(costmap_ros->getCostmap());
      if (params_.priormap_static_obstacle_clearance_radius > 0.0) {
        auto static_clearance_query = std::make_shared<StaticObstacleClearanceQuery>(
          global_query_, params_.priormap_static_obstacle_clearance_radius);
        RCLCPP_INFO(
          logger,
          "[MincoPlanner] Static-obstacle global hard clearance: radius=%.3f m "
          "newly_hardened_cells=%zu",
          static_clearance_query->clearanceRadius(),
          static_clearance_query->hardenedCellCount());
        global_query_ = std::move(static_clearance_query);
      }
      if (params_.priormap_ground_edge_avoidance_enable && ground_edge_prior_ready_) {
        auto ground_edge_query = std::make_shared<SurveyedGroundEdgeQuery>(
          global_query_, ground_edge_prior_, params_.priormap_ground_edge_config);
        RCLCPP_INFO(
          logger,
          "[MincoPlanner] Surveyed ground-edge global overlay: edge_cells=%zu "
          "clearance_cells=%zu unsupported_cells=%zu max_step=%.3f m max_slope=%.1f deg "
          "lethal_clearance=%.3f m clearance=%.3f m cost=%u",
          ground_edge_query->edgeCellCount(), ground_edge_query->clearanceCellCount(),
          ground_edge_query->unsupportedCellCount(),
          params_.priormap_ground_edge_config.max_step,
          params_.priormap_ground_edge_config.max_slope_deg,
          params_.priormap_ground_edge_config.lethal_clearance_radius,
          params_.priormap_ground_edge_config.clearance_radius,
          static_cast<unsigned int>(params_.priormap_ground_edge_config.clearance_cost));
        global_query_ = std::move(ground_edge_query);
      }
    }
    dynamic_query_ = raw_rog_query ? std::make_shared<FrameAwareRogQuery>(
                                       raw_rog_query, tf, map_frame_, rog_frame_, logger, clock)
                                   : nullptr;
    sparsify_query_ = global_query_;
  } else {
    global_query_ = raw_rog_query;
    dynamic_query_ = raw_rog_query;
    sparsify_query_ = raw_rog_query;
  }
}

}  // namespace minco_planner
