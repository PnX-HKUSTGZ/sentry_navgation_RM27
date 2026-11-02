#include "minco_core/components/planner_mode_context.hpp"

#include <cctype>

#include "nav2_costmap_2d/costmap_layer.hpp"

namespace minco_planner {

namespace {

std::shared_ptr<rog_map::MapQueryInterface> staticLayerQuery(
  nav2_costmap_2d::Costmap2DROS * costmap_ros,
  const std::string & static_layer_suffix = "static_layer")
{
  if (!costmap_ros || !costmap_ros->getLayeredCostmap() ||
    !costmap_ros->getLayeredCostmap()->getPlugins())
  {
    return nullptr;
  }
  for (const auto & plugin : *costmap_ros->getLayeredCostmap()->getPlugins()) {
    if (!plugin) {
      continue;
    }
    const std::string & name = plugin->getName();
    const bool exact_name = name == static_layer_suffix;
    const bool qualified_name =
      name.size() > static_layer_suffix.size() &&
      name[name.size() - static_layer_suffix.size() - 1U] == '.' &&
      name.compare(
        name.size() - static_layer_suffix.size(),
        static_layer_suffix.size(),
        static_layer_suffix) == 0;
    if (!exact_name && !qualified_name)
    {
      continue;
    }
    const auto costmap_layer =
      std::dynamic_pointer_cast<nav2_costmap_2d::CostmapLayer>(plugin);
    if (costmap_layer) {
      return std::make_shared<Nav2CostmapQuery>(costmap_layer.get());
    }
  }
  return nullptr;
}

}  // namespace

void PlannerModeContext::configure(const PlannerModeParams & params,
  const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
  nav2_costmap_2d::Costmap2DROS * costmap_ros,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr & clock)
{
  params_ = params;
  if (params_.priormap_static_clearance_mode != "circle" &&
    params_.priormap_static_clearance_mode != "polygon")
  {
    throw std::invalid_argument("priormap.static_clearance_mode must be circle or polygon");
  }
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
  static_clearance_query_.reset();
  static_query_.reset();
  if (mode_ == PlannerMode::PRIORMAP) {
    dynamic_query_ = raw_rog_query ? std::make_shared<FrameAwareRogQuery>(
      raw_rog_query, tf, map_frame_, rog_frame_, logger, clock) : nullptr;
    global_query_ = nullptr;
    if (costmap_ros && costmap_ros->getCostmap()) {
      global_query_ = std::make_shared<Nav2CostmapQuery>(costmap_ros->getCostmap());
      static_query_ = staticLayerQuery(costmap_ros);
      if (!static_query_) {
        throw std::invalid_argument("PRIORMAP requires an original static_layer for footprint checks");
      }
      if (params_.priormap_static_obstacle_clearance_radius > 0.0) {
        const auto static_source = static_query_;
        if (static_source) {
          if (usesPolygonStaticClearance()) {
            // Only attribute 253 to static inflation when all obstacle-producing
            // layers are known. Custom layer stacks retain the original hard cost.
            const auto & layers = *costmap_ros->getLayeredCostmap()->getPlugins();
            const bool known_layers = layers.size() == 3U &&
              std::all_of(layers.begin(), layers.end(), [](const auto & layer) {
                if (!layer) {return false;}
                const auto & name = layer->getName();
                const auto last_dot = name.find_last_of('.');
                const auto local_name = name.substr(last_dot == std::string::npos ? 0U : last_dot + 1U);
                return local_name == "static_layer" || local_name == "rog_dynamic_obstacle_layer" ||
                  local_name == "inflation_layer";
              });
            auto measured = known_layers ?
              staticLayerQuery(costmap_ros, "rog_dynamic_obstacle_layer") : nullptr;
            static_clearance_query_ = std::make_shared<StaticObstacleClearanceQuery>(
              global_query_, static_source, params_.priormap_static_footprint,
              0.0, measured, costmap_ros->getLayeredCostmap()->getInscribedRadius(),
              params_.static_overlap);
            RCLCPP_INFO(logger,
              "[MincoPlanner] Original-PGM global polygon clearance: vertices=%zu "
              "grid_guard=%.3f m overlap_ratio=%.3f overlap_depth=%.3f m; "
              "static safety and recovery use the same source and tolerance.",
              params_.priormap_static_footprint.size(), 0.0,
              params_.static_overlap.max_ratio, params_.static_overlap.max_depth);
          } else {
            static_clearance_query_ = std::make_shared<StaticObstacleClearanceQuery>(
              global_query_, static_source, params_.priormap_static_obstacle_clearance_radius);
            RCLCPP_INFO(logger,
              "[MincoPlanner] Static-layer global hard clearance: radius=%.3f m "
              "newly_hardened_cells=%zu unknown_boundary_guard_cells=%zu",
              static_clearance_query_->clearanceRadius(),
              static_clearance_query_->hardenedCellCount(),
              static_clearance_query_->unknownBoundaryGuardCellCount());
          }
          global_query_ = static_clearance_query_;
        } else {
          if (usesPolygonStaticClearance()) {
            throw std::invalid_argument("Polygon static clearance requires a Nav2 static_layer");
          }
          RCLCPP_WARN(
            logger,
            "[MincoPlanner] static_layer is unavailable; skipping the static hard-clearance "
            "snapshot so transient ROG obstacles cannot become permanent global obstacles.");
        }
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
    sparsify_query_ = global_query_;
  } else {
    global_query_ = raw_rog_query;
    dynamic_query_ = raw_rog_query;
    sparsify_query_ = raw_rog_query;
  }
}

void PlannerModeContext::updateGlobalFootprintYaw(double yaw) const
{
  if (static_clearance_query_) {
    static_clearance_query_->setFootprintYaw(yaw);
  }
}

}  // namespace minco_planner
