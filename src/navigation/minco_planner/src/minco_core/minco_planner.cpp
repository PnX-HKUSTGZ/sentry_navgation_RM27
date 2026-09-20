// Corresponding header
#include "minco_core/minco_planner.hpp"
#include "minco_core/components/cached_trajectory_policy.hpp"
#include "nav2_util/node_utils.hpp"

// Project
#include "rog_map/map_registry.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>

namespace minco_planner {

using namespace color_text;

MincoPlanner::MincoPlanner() : tf_(nullptr) {}

MincoPlanner::~MincoPlanner() { planner_perf_monitor_.close(); }

void MincoPlanner::configureMincoPerfLogging(
    const nav2_util::LifecycleNode::SharedPtr &node,
    const std::string &prefix) {
  const std::string default_minco_csv_path = "/tmp/minco_perf_detailed.csv";

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.print_enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.detailed_csv_enable",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.odom_sub_debug_enable",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.print_period_sec",
      rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.csv_flush_every_n",
      rclcpp::ParameterValue(30));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.minco_csv_path",
      rclcpp::ParameterValue(default_minco_csv_path));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.run_id", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.scenario", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "performance.variant", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "rog_map.performance.enable",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "rog_map.performance.detailed_csv_enable",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "rog_map.performance.minco_csv_path",
      rclcpp::ParameterValue(default_minco_csv_path));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "rog_map.performance.run_id", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "rog_map.performance.scenario",
      rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "rog_map.performance.variant", rclcpp::ParameterValue(""));

  bool performance_enable = true;
  bool print_enable = true;
  bool detailed_csv_enable = false;
  bool odom_sub_debug_enable = true;
  double print_period_sec = 1.0;
  int csv_flush_every_n = 30;
  std::string minco_csv_path = default_minco_csv_path;
  std::string run_id;
  std::string scenario;
  std::string variant;
  node->get_parameter(prefix + "performance.enable", performance_enable);
  node->get_parameter(prefix + "performance.print_enable", print_enable);
  node->get_parameter(prefix + "performance.detailed_csv_enable",
                      detailed_csv_enable);
  node->get_parameter(prefix + "performance.odom_sub_debug_enable",
                      odom_sub_debug_enable);
  node->get_parameter(prefix + "performance.print_period_sec",
                      print_period_sec);
  node->get_parameter(prefix + "performance.csv_flush_every_n",
                      csv_flush_every_n);
  node->get_parameter(prefix + "performance.minco_csv_path", minco_csv_path);
  node->get_parameter(prefix + "performance.run_id", run_id);
  node->get_parameter(prefix + "performance.scenario", scenario);
  node->get_parameter(prefix + "performance.variant", variant);
  if (!detailed_csv_enable) {
    node->get_parameter(prefix + "rog_map.performance.detailed_csv_enable",
                        detailed_csv_enable);
  }
  if (performance_enable) {
    node->get_parameter(prefix + "rog_map.performance.enable",
                        performance_enable);
  }
  if (minco_csv_path == default_minco_csv_path) {
    node->get_parameter(prefix + "rog_map.performance.minco_csv_path",
                        minco_csv_path);
  }
  if (run_id.empty()) {
    node->get_parameter(prefix + "rog_map.performance.run_id", run_id);
  }
  if (scenario.empty()) {
    node->get_parameter(prefix + "rog_map.performance.scenario", scenario);
  }
  if (variant.empty()) {
    node->get_parameter(prefix + "rog_map.performance.variant", variant);
  }

  PlannerPerformanceConfig perf_cfg;
  perf_cfg.enable = performance_enable;
  perf_cfg.print_enable = print_enable;
  perf_cfg.detailed_csv_enable = detailed_csv_enable;
  perf_cfg.odom_sub_debug_enable = odom_sub_debug_enable;
  perf_cfg.detailed_csv_path = minco_csv_path;
  perf_cfg.run_id = run_id;
  perf_cfg.scenario = scenario;
  perf_cfg.variant = variant;
  perf_cfg.print_period_sec = print_period_sec;
  perf_cfg.csv_flush_every_n = csv_flush_every_n;

  planner_perf_monitor_.configure(perf_cfg, logger_);
}

bool MincoPlanner::configureRogMap(
    const nav2_util::LifecycleNode::SharedPtr &node,
    const std::string &plugin_prefix) {
  if (!node) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Cannot configure ROGMap without "
                          "planner_server LifecycleNode.");
    return false;
  }

  rog_map::Config rog_cfg;
  try {
    rog_cfg.loadFromRosNode(node, plugin_prefix + "rog_map");
    rog_map_ros_ = std::make_shared<rog_map::ROGMapROS>(node, rog_cfg, tf_);
    rog_query_raw_ = rog_map_ros_->queryInterface();
    priormap_ground_edge_yaml_path_ = rog_cfg.prior_map_yaml_path;
    priormap_ground_edge_pgm_path_ = rog_cfg.prior_map_pgm_path;
    priormap_ground_edge_frame_ = rog_cfg.prior_map_frame;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Failed to configure ROGMap: %s",
                 e.what());
    rog_map_ros_.reset();
    rog_query_raw_.reset();
    map_.reset();
    if (rog_cfg.prior_map_enable) {
      throw;
    }
    return false;
  }

  if (!rog_query_raw_) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap queryInterface is null.");
    rog_map_ros_.reset();
    return false;
  }

  rog_map::MapRegistry::set(rog_query_raw_);
  RCLCPP_INFO(logger_, "[MincoPlanner] ROGMap is created inside MincoPlanner "
                       "plugin and shared by pointer.");
  return true;
}

bool MincoPlanner::ensureMapAvailable() {
  if (rog_query_raw_ || map_) {
    return true;
  }

  auto map = rog_map::MapRegistry::get();
  if (map) {
    rog_query_raw_ = map;
    if (!map_) {
      map_ = map;
    }
    return true;
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_ERROR_THROTTLE(logger_, *node->get_clock(), 1000,
                          "[MincoPlanner] MapQueryInterface unavailable: "
                          "ROGMap was not created and MapRegistry is empty.");
  } else {
    RCLCPP_ERROR(logger_, "[MincoPlanner] MapQueryInterface unavailable.");
  }
  return false;
}

void MincoPlanner::rebuildModeDependentQueries() {
  if (!mode_context_) {
    return;
  }

  mode_context_->rebuildQueries(rog_query_raw_, costmap_ros_.get(), tf_,
                                logger_, clock_);
  map_ = mode_context_->dynamicQuery();

  if (global_path_searcher_) {
    global_path_searcher_->setQuery(mode_context_->globalQuery());
  }
  if (astar_planner_) {
    astar_planner_->setMap(mode_context_->globalQuery());
    astar_planner_->setESDFQuery(
      mode_context_->dynamicGlobalObstacleEnabled() ? mode_context_->dynamicQuery() : nullptr);
    astar_planner_->setCollisionDistance(
      mode_context_->dynamicGlobalObstacleEnabled() ?
      mode_context_->dynamicGlobalCollisionDistance() : 0.0);
  }
  if (smac_planner_) {
    smac_planner_->setMap(mode_context_->globalQuery());
    smac_planner_->setESDFQuery(
      mode_context_->dynamicGlobalObstacleEnabled() ? mode_context_->dynamicQuery() : nullptr);
    smac_planner_->setCollisionDistance(
      mode_context_->dynamicGlobalObstacleEnabled() ?
      mode_context_->dynamicGlobalCollisionDistance() : 0.0);
  }
  if (minco_optimizer_) {
    minco_optimizer_->setMap(mode_context_->dynamicQuery());
  }
  if (corridor_gen_) {
    corridor_gen_->setMap(mode_context_->dynamicQuery());
  }
  if (safety_checker_) {
    safety_checker_->setQuery(mode_context_->dynamicQuery());
  }

  RCLCPP_INFO(logger_, "[MincoPlanner] Rebuilt mode-dependent map queries.");
}

void MincoPlanner::initPlannerMode(const std::string &planner_mode_param,
                                   const std::string &map_frame,
                                   const std::string &rog_frame,
                                   double static_obstacle_clearance_radius) {
  mode_params_.planner_mode = planner_mode_param;
  mode_params_.map_frame = map_frame.empty() ? "map" : map_frame;
  mode_params_.rog_frame = rog_frame.empty() ? "camera_init" : rog_frame;
  mode_params_.priormap_use_nav2_global_search =
      priormap_use_nav2_global_search_;
  mode_params_.priormap_dynamic_global_obstacle_enable =
      priormap_dynamic_global_obstacle_enable_;
  mode_params_.priormap_dynamic_global_collision_distance =
      priormap_dynamic_global_collision_distance_;
  mode_params_.priormap_clip_seed_by_rog_boundary =
      priormap_clip_seed_by_rog_boundary_;
  mode_params_.priormap_rog_boundary_margin = priormap_rog_boundary_margin_;
  mode_params_.priormap_rog_boundary_sample_step =
      priormap_rog_boundary_sample_step_;
  mode_params_.priormap_static_obstacle_clearance_radius =
      static_obstacle_clearance_radius;
  mode_params_.priormap_ground_edge_avoidance_enable =
      priormap_ground_edge_avoidance_enable_;
  mode_params_.priormap_ground_edge_yaml_path =
      priormap_ground_edge_yaml_path_;
  mode_params_.priormap_ground_edge_pgm_path =
      priormap_ground_edge_pgm_path_;
  mode_params_.priormap_ground_edge_frame = priormap_ground_edge_frame_;
  mode_params_.priormap_ground_edge_config.max_step =
      priormap_ground_edge_max_step_;
  mode_params_.priormap_ground_edge_config.max_slope_deg =
      priormap_ground_edge_max_slope_deg_;
  mode_params_.priormap_ground_edge_config.lethal_clearance_radius =
      priormap_ground_edge_lethal_clearance_radius_;
  mode_params_.priormap_ground_edge_config.clearance_radius =
      priormap_ground_edge_clearance_radius_;
  mode_params_.priormap_ground_edge_config.clearance_cost =
      static_cast<uint8_t>(priormap_ground_edge_clearance_cost_);
  mode_params_.exploration_boundary_margin = exploration_boundary_margin_;
  mode_params_.exploration_boundary_sample_step =
      exploration_boundary_sample_step_;
  mode_params_.exploration_unknown_as_occupied =
      exploration_unknown_as_occupied_;
  mode_params_.exploration_prefer_goal_direction =
      exploration_prefer_goal_direction_;

  mode_context_ = std::make_unique<PlannerModeContext>();
  mode_context_->configure(mode_params_, rog_query_raw_, costmap_ros_.get(),
                           tf_, logger_, clock_);

  planning_frame_ = mode_context_->planningFrame();
  output_frame_ = mode_context_->outputFrame();
  map_frame_ = mode_context_->mapFrame();
  rog_frame_ = mode_context_->rogFrame();
  global_frame_ = output_frame_;
  map_ = mode_context_->dynamicQuery();

  RCLCPP_INFO(logger_, "[MincoPlanner] planner_mode=%s",
              mode_context_->mode() == PlannerMode::PRIORMAP ? "PRIORMAP"
                                                             : "EXPLORATION");
  RCLCPP_INFO(logger_,
              "[MincoPlanner] planning_frame=%s output_frame=%s rog_frame=%s "
              "physical_base_frame=%s",
              planning_frame_.c_str(), output_frame_.c_str(),
              rog_frame_.c_str(), physical_base_frame_.c_str());
  RCLCPP_INFO(
      logger_, "[MincoPlanner] global_search=%s dynamic_query=%s",
      mode_context_->mode() == PlannerMode::PRIORMAP ? "Nav2Costmap"
                                                     : "ROGMapBoundaryAstar",
      mode_context_->mode() == PlannerMode::PRIORMAP ? "FrameAwareRogQuery"
                                                     : "DirectRogQuery");
}

// -----------------------------------------------------------------------------
// 2) Lifecycle management
// -----------------------------------------------------------------------------

void MincoPlanner::configure(
    const nav2_util::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const std::string prefix = name_ + ".";
  configureMincoPerfLogging(node, prefix);

  // --- General config --------------------------------------------------------

  std::string planner_mode_param = "PRIORMAP";
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "planner_mode",
      rclcpp::ParameterValue(planner_mode_param));
  node->get_parameter(prefix + "planner_mode", planner_mode_param);

  std::string configured_map_frame = "map";
  std::string configured_rog_frame = "camera_init";
  std::string configured_physical_base_frame = "base_link";
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "frames.map_frame",
      rclcpp::ParameterValue(configured_map_frame));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "frames.rog_frame",
      rclcpp::ParameterValue(configured_rog_frame));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "frames.physical_base_frame",
      rclcpp::ParameterValue(configured_physical_base_frame));
  node->get_parameter(prefix + "frames.map_frame", configured_map_frame);
  node->get_parameter(prefix + "frames.rog_frame", configured_rog_frame);
  node->get_parameter(prefix + "frames.physical_base_frame",
                      configured_physical_base_frame);
  if (configured_physical_base_frame.empty()) {
    throw std::invalid_argument(prefix +
                                "frames.physical_base_frame must not be empty");
  }
  physical_base_frame_ = configured_physical_base_frame;

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.use_nav2_global_search",
      rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "priormap.use_nav2_global_search",
                      priormap_use_nav2_global_search_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.dynamic_global_obstacle.enable",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.dynamic_global_obstacle.collision_distance",
      rclcpp::ParameterValue(0.0));
  node->get_parameter(prefix + "priormap.dynamic_global_obstacle.enable",
                      priormap_dynamic_global_obstacle_enable_);
  node->get_parameter(prefix + "priormap.dynamic_global_obstacle.collision_distance",
                      priormap_dynamic_global_collision_distance_);
  if (!std::isfinite(priormap_dynamic_global_collision_distance_) ||
      priormap_dynamic_global_collision_distance_ < 0.0) {
    throw std::invalid_argument(
      prefix + "priormap.dynamic_global_obstacle.collision_distance must be finite and >= 0");
  }

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.clip_seed_by_rog_boundary",
      rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "priormap.clip_seed_by_rog_boundary",
                      priormap_clip_seed_by_rog_boundary_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.rog_boundary_margin",
      rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "priormap.rog_boundary_margin",
                      priormap_rog_boundary_margin_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.rog_boundary_sample_step",
      rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "priormap.rog_boundary_sample_step",
                      priormap_rog_boundary_sample_step_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.ground_edge_avoidance.enable",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.ground_edge_avoidance.max_step",
      rclcpp::ParameterValue(0.06));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.ground_edge_avoidance.max_slope_deg",
      rclcpp::ParameterValue(28.0));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.ground_edge_avoidance.clearance_radius",
      rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.ground_edge_avoidance.lethal_clearance_radius",
      rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "priormap.ground_edge_avoidance.clearance_cost",
      rclcpp::ParameterValue(240));
  node->get_parameter(prefix + "priormap.ground_edge_avoidance.enable",
                      priormap_ground_edge_avoidance_enable_);
  node->get_parameter(prefix + "priormap.ground_edge_avoidance.max_step",
                      priormap_ground_edge_max_step_);
  node->get_parameter(prefix + "priormap.ground_edge_avoidance.max_slope_deg",
                      priormap_ground_edge_max_slope_deg_);
  node->get_parameter(
      prefix + "priormap.ground_edge_avoidance.clearance_radius",
      priormap_ground_edge_clearance_radius_);
  node->get_parameter(
      prefix + "priormap.ground_edge_avoidance.lethal_clearance_radius",
      priormap_ground_edge_lethal_clearance_radius_);
  node->get_parameter(prefix + "priormap.ground_edge_avoidance.clearance_cost",
                      priormap_ground_edge_clearance_cost_);
  if (!std::isfinite(priormap_ground_edge_max_step_) ||
      priormap_ground_edge_max_step_ < 0.0 ||
      !std::isfinite(priormap_ground_edge_max_slope_deg_) ||
      priormap_ground_edge_max_slope_deg_ < 0.0 ||
      priormap_ground_edge_max_slope_deg_ >= 90.0 ||
      !std::isfinite(priormap_ground_edge_clearance_radius_) ||
      priormap_ground_edge_clearance_radius_ < 0.0 ||
      !std::isfinite(priormap_ground_edge_lethal_clearance_radius_) ||
      priormap_ground_edge_lethal_clearance_radius_ < 0.0 ||
      priormap_ground_edge_lethal_clearance_radius_ >
          priormap_ground_edge_clearance_radius_ + 1.0e-9 ||
      priormap_ground_edge_clearance_cost_ <= 0 ||
      priormap_ground_edge_clearance_cost_ >=
          static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
    throw std::invalid_argument(
        prefix + "priormap.ground_edge_avoidance configuration is invalid");
  }

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "exploration.boundary_margin",
      rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "exploration.boundary_margin",
                      exploration_boundary_margin_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "exploration.boundary_sample_step",
      rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "exploration.boundary_sample_step",
                      exploration_boundary_sample_step_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "exploration.unknown_as_occupied",
      rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "exploration.unknown_as_occupied",
                      exploration_unknown_as_occupied_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "exploration.prefer_goal_direction",
      rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "exploration.prefer_goal_direction",
                      exploration_prefer_goal_direction_);

  std::string configured_global_frame = "map";
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "global_frame",
      rclcpp::ParameterValue(configured_global_frame));
  node->get_parameter(prefix + "global_frame", configured_global_frame);
  global_frame_ =
      costmap_ros_ ? costmap_ros_->getGlobalFrameID() : configured_global_frame;
  if (!configureRogMap(node, prefix)) {
    ensureMapAvailable();
  }

  nav2_util::declare_parameter_if_not_declared(node, prefix + "tolerance",
                                               rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "tolerance", tolerance_);

  nav2_util::declare_parameter_if_not_declared(node, prefix + "use_smac",
                                               rclcpp::ParameterValue(false));
  node->get_parameter(prefix + "use_smac", use_smac_);

  nav2_util::declare_parameter_if_not_declared(node, prefix + "allow_unknown",
                                               rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(node, prefix + "lidar_offset_x",
                                               rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, prefix + "lidar_offset_y",
                                               rclcpp::ParameterValue(-0.2));
  node->get_parameter(prefix + "lidar_offset_x", lidar_offset_x_);
  node->get_parameter(prefix + "lidar_offset_y", lidar_offset_y_);

  // Odometry topic
  std::string odom_topic = "odom";
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "odom_topic", rclcpp::ParameterValue(odom_topic));
  node->get_parameter(prefix + "odom_topic", odom_topic);

  std::string trajectory_topic = "opt_path";
  std::string backup_trajectory_topic = "backup_path";
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "trajectory_topic",
      rclcpp::ParameterValue(trajectory_topic));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "backup_trajectory_topic",
      rclcpp::ParameterValue(backup_trajectory_topic));
  node->get_parameter(prefix + "trajectory_topic", trajectory_topic);
  node->get_parameter(prefix + "backup_trajectory_topic",
                      backup_trajectory_topic);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "request_lease_timeout", rclcpp::ParameterValue(0.0));
  node->get_parameter(prefix + "request_lease_timeout", request_lease_timeout_);
  if (!std::isfinite(request_lease_timeout_)) {
    throw std::invalid_argument(prefix +
                                "request_lease_timeout must be finite");
  }
  request_lease_.configure(request_lease_timeout_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.opt_freq", rclcpp::ParameterValue(20.0));
  node->get_parameter(prefix + "minco_optimizer.opt_freq", opt_freq_);
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.failed_replan_retry_period",
      rclcpp::ParameterValue(0.25));
  node->get_parameter(prefix + "minco_optimizer.failed_replan_retry_period",
                      failed_replan_retry_period_);
  if (!std::isfinite(failed_replan_retry_period_) ||
      failed_replan_retry_period_ < 0.0) {
    throw std::invalid_argument(prefix +
                                "minco_optimizer.failed_replan_retry_period "
                                "must be finite and non-negative");
  }
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.successful_replan_period",
      rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "minco_optimizer.successful_replan_period",
                      successful_replan_period_);
  if (!std::isfinite(successful_replan_period_) ||
      successful_replan_period_ < 0.0) {
    throw std::invalid_argument(prefix +
                                "minco_optimizer.successful_replan_period "
                                "must be finite and non-negative");
  }

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.lookahead_dist",
      rclcpp::ParameterValue(5.0));
  node->get_parameter(prefix + "minco_optimizer.lookahead_dist",
                      lookahead_dist_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.traj_goal_tolerance",
      rclcpp::ParameterValue(0.3));
  node->get_parameter(prefix + "minco_optimizer.traj_goal_tolerance",
                      traj_goal_tolerance_);

  double shortcut_peak_cost_slack = 10.0;
  double shortcut_mean_cost_slack = 5.0;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "local_path.shortcut_peak_cost_slack",
      rclcpp::ParameterValue(shortcut_peak_cost_slack));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "local_path.shortcut_mean_cost_slack",
      rclcpp::ParameterValue(shortcut_mean_cost_slack));
  node->get_parameter(prefix + "local_path.shortcut_peak_cost_slack",
                      shortcut_peak_cost_slack);
  node->get_parameter(prefix + "local_path.shortcut_mean_cost_slack",
                      shortcut_mean_cost_slack);
  if (!std::isfinite(shortcut_peak_cost_slack) || shortcut_peak_cost_slack < 0.0 ||
      !std::isfinite(shortcut_mean_cost_slack) || shortcut_mean_cost_slack < 0.0) {
    throw std::invalid_argument(
            prefix + "local_path shortcut cost slacks must be finite and non-negative");
  }

  // --- Optimizer config ------------------------------------------------------

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.safe_dist", rclcpp::ParameterValue(0.3));
  node->get_parameter(prefix + "minco_optimizer.safe_dist",
                      minco_config.safe_dist);

  double collision_dist = minco_config.safe_dist;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.collision_dist",
      rclcpp::ParameterValue(collision_dist));
  node->get_parameter(prefix + "minco_optimizer.collision_dist",
                      collision_dist);

  TrajectorySafetyChecker::Config safety_config;
  safety_config.safe_dist = collision_dist;
  safety_config.planning_frame = planning_frame_;
  safety_config.rog_frame = rog_frame_;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.footprint_length",
      rclcpp::ParameterValue(safety_config.footprint_length));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.footprint_width",
      rclcpp::ParameterValue(safety_config.footprint_width));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.footprint_margin",
      rclcpp::ParameterValue(safety_config.footprint_margin));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.sample_dt",
      rclcpp::ParameterValue(safety_config.sample_dt));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.map_timeout",
      rclcpp::ParameterValue(safety_config.map_timeout));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.future_tolerance",
      rclcpp::ParameterValue(safety_config.future_tolerance));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.check_horizon", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "safety.collision_cache_reuse_max_duration",
      rclcpp::ParameterValue(collision_cache_reuse_max_duration_));
  node->get_parameter(prefix + "safety.footprint_length",
                      safety_config.footprint_length);
  node->get_parameter(prefix + "safety.footprint_width",
                      safety_config.footprint_width);
  node->get_parameter(prefix + "safety.footprint_margin",
                      safety_config.footprint_margin);
  node->get_parameter(prefix + "safety.sample_dt", safety_config.sample_dt);
  node->get_parameter(prefix + "safety.map_timeout", safety_config.map_timeout);
  node->get_parameter(prefix + "safety.future_tolerance",
                      safety_config.future_tolerance);
  node->get_parameter(prefix + "safety.check_horizon",
                      safety_check_horizon_);
  node->get_parameter(prefix + "safety.collision_cache_reuse_max_duration",
                      collision_cache_reuse_max_duration_);
  if (!std::isfinite(collision_cache_reuse_max_duration_) ||
      collision_cache_reuse_max_duration_ < 0.0) {
    throw std::invalid_argument(
        "MincoPlanner safety.collision_cache_reuse_max_duration must be "
        "finite and nonnegative");
  }
  if (!std::isfinite(safety_check_horizon_) || safety_check_horizon_ <= 0.0) {
    throw std::invalid_argument(
        prefix + "safety.check_horizon must be finite and positive");
  }
  if (!std::isfinite(safety_config.footprint_length) ||
      safety_config.footprint_length <= 0.0 ||
      !std::isfinite(safety_config.footprint_width) ||
      safety_config.footprint_width <= 0.0 ||
      !std::isfinite(safety_config.footprint_margin) ||
      safety_config.footprint_margin <= 0.0) {
    throw std::invalid_argument(
        prefix + "safety footprint dimensions and margin must be finite and positive");
  }

  double grid_guard = 0.0;
  if (costmap_ros_ && costmap_ros_->getCostmap()) {
    const double costmap_resolution = costmap_ros_->getCostmap()->getResolution();
    if (std::isfinite(costmap_resolution) && costmap_resolution > 0.0) {
      grid_guard = costmap_resolution;
    }
  }
  if (rog_query_raw_) {
    const double rog_resolution = rog_query_raw_->resolution();
    if (std::isfinite(rog_resolution) && rog_resolution > 0.0) {
      grid_guard = std::max(grid_guard, rog_resolution);
    }
  }
  const double footprint_half_length =
      0.5 * safety_config.footprint_length + safety_config.footprint_margin;
  const double footprint_half_width =
      0.5 * safety_config.footprint_width + safety_config.footprint_margin;
  const double static_obstacle_clearance_radius =
      std::hypot(footprint_half_length, footprint_half_width) + grid_guard;
  initPlannerMode(planner_mode_param, configured_map_frame,
                  configured_rog_frame, static_obstacle_clearance_radius);
  safety_config.planning_frame = planning_frame_;
  safety_config.rog_frame = rog_frame_;
  RCLCPP_INFO(
      logger_,
      "[MincoPlanner] Global static hard-clearance radius %.3f m = "
      "footprint corner %.3f m + grid guard %.3f m.",
      static_obstacle_clearance_radius,
      std::hypot(footprint_half_length, footprint_half_width), grid_guard);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.max_velocity",
      rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "minco_optimizer.max_velocity",
                      minco_config.max_vel);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.max_acceleration",
      rclcpp::ParameterValue(4.0));
  node->get_parameter(prefix + "minco_optimizer.max_acceleration",
                      minco_config.max_acc);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.turn_angle_deadzone",
      rclcpp::ParameterValue(0.174));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_deadzone",
                      minco_config.turn_angle_deadzone);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.turn_angle_saturation",
      rclcpp::ParameterValue(1.57));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_saturation",
                      minco_config.turn_angle_saturation);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.min_turn_vel",
      rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "minco_optimizer.min_turn_vel",
                      minco_config.min_turn_vel);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.decay_power",
      rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "minco_optimizer.decay_power",
                      minco_config.decay_power);

  double max_yaw_dot = 3.14;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.max_yaw_dot",
      rclcpp::ParameterValue(max_yaw_dot));
  node->get_parameter(prefix + "minco_optimizer.max_yaw_dot", max_yaw_dot);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.enable_yaw_opt",
      rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.enable_yaw_opt", use_yaw_opt_);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.time_allocation_iters",
      rclcpp::ParameterValue(15));
  node->get_parameter(prefix + "minco_optimizer.time_allocation_iters",
                      minco_config.time_allocation_iters);
  if (minco_config.time_allocation_iters <= 0) {
    throw std::invalid_argument(
        prefix +
        "minco_optimizer.time_allocation_iters must be greater than zero");
  }

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.penalty_weight_time",
      rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_time",
                      minco_config.rho);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.smooth_eps",
      rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.smooth_eps",
                      minco_config.smooth_eps);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.integral_res",
      rclcpp::ParameterValue(16));
  node->get_parameter(prefix + "minco_optimizer.integral_res",
                      minco_config.integral_res);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.opt_accuracy",
      rclcpp::ParameterValue(1.0e-4));
  node->get_parameter(prefix + "minco_optimizer.opt_accuracy",
                      minco_config.opt_accuracy);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.print_optimizer_log",
      rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.print_optimizer_log",
                      minco_config.print_optimizer_log);

  double penalty_weight_pos = 0.0;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.penalty_weight_pos",
      rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_pos",
                      penalty_weight_pos);

  double penalty_weight_vel = 0.0;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.penalty_weight_vel",
      rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_vel",
                      penalty_weight_vel);

  double penalty_weight_acc = 0.0;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.penalty_weight_acc",
      rclcpp::ParameterValue(10000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_acc",
                      penalty_weight_acc);

  double penalty_weight_att = 0.0;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.penalty_weight_att",
      rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_att",
                      penalty_weight_att);

  double penalty_weight_time_barrier = 0.0;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "minco_optimizer.penalty_weight_time_barrier",
      rclcpp::ParameterValue(100.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_time_barrier",
                      penalty_weight_time_barrier);

  minco_config.penaltyWeights.resize(5);
  minco_config.penaltyWeights(0) = penalty_weight_pos;
  minco_config.penaltyWeights(1) = penalty_weight_vel;
  minco_config.penaltyWeights(2) = penalty_weight_acc;
  minco_config.penaltyWeights(3) = penalty_weight_att;
  minco_config.penaltyWeights(4) = penalty_weight_time_barrier;

  minco_config.magnitudeBounds.resize(3);
  minco_config.magnitudeBounds(0) = minco_config.safe_dist;
  minco_config.magnitudeBounds(1) = minco_config.max_vel;
  minco_config.magnitudeBounds(2) = minco_config.max_acc;

  // --- Corridor config -------------------------------------------------------

  double corridor_robot_radius = 0.4;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "corridor.robot_radius",
      rclcpp::ParameterValue(corridor_robot_radius));
  node->get_parameter(prefix + "corridor.robot_radius", corridor_robot_radius);

  double corridor_extra_margin = 0.15;
  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "corridor.extra_margin",
      rclcpp::ParameterValue(corridor_extra_margin));
  node->get_parameter(prefix + "corridor.extra_margin", corridor_extra_margin);

  // --- Recovery server config -----------------------------------------------

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "recovery_server.fail_threshold",
      rclcpp::ParameterValue(3));
  node->get_parameter(prefix + "recovery_server.fail_threshold",
                      recovery_server_config_.fail_threshold);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "recovery_server.cooldown_sec",
      rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "recovery_server.cooldown_sec",
                      recovery_server_config_.cooldown_sec);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "recovery_server.recovery_window_sec",
      rclcpp::ParameterValue(3.0));
  node->get_parameter(prefix + "recovery_server.recovery_window_sec",
                      recovery_server_config_.recovery_window_sec);

  nav2_util::declare_parameter_if_not_declared(
      node, prefix + "recovery_server.escape_speed",
      rclcpp::ParameterValue(0.4));
  node->get_parameter(prefix + "recovery_server.escape_speed",
                      recovery_server_config_.escape_speed);

  // --- Components / publishers / timers -------------------------------------

  const auto global_query =
      mode_context_ ? mode_context_->globalQuery() : nullptr;
  const auto dynamic_query =
      mode_context_ ? mode_context_->dynamicQuery() : nullptr;
  const unsigned int init_size_x = global_query ? global_query->sizeX() : 1U;
  const unsigned int init_size_y = global_query ? global_query->sizeY() : 1U;
  astar_planner_ = std::make_unique<Astar>(init_size_x, init_size_y);
  astar_planner_->setMap(global_query);
  astar_planner_->setESDFQuery(
    mode_context_ && mode_context_->dynamicGlobalObstacleEnabled() ? dynamic_query : nullptr);
  astar_planner_->setCollisionDistance(
    mode_context_ && mode_context_->dynamicGlobalObstacleEnabled() ?
    mode_context_->dynamicGlobalCollisionDistance() : 0.0);

  if (use_smac_) {
    smac_planner_ =
        std::make_unique<minco_planner::smac::SmacPlanner2DSimple>();
    smac_planner_->configure(node, costmap_ros_, prefix);
    smac_planner_->setParameters(allow_unknown_, 1000000, tolerance_);
    smac_planner_->setMap(global_query);
    smac_planner_->setESDFQuery(
      mode_context_ && mode_context_->dynamicGlobalObstacleEnabled() ? dynamic_query : nullptr);
    smac_planner_->setCollisionDistance(
      mode_context_ && mode_context_->dynamicGlobalObstacleEnabled() ?
      mode_context_->dynamicGlobalCollisionDistance() : 0.0);
  }

  RCLCPP_INFO(
      logger_,
      "[MincoPlanner] Global search dynamic ROG hard mask: enabled=%s "
      "collision_distance=%.3f m; UNKNOWN/frontier cells are excluded.",
      mode_context_ && mode_context_->dynamicGlobalObstacleEnabled() ? "true" : "false",
      mode_context_ ? mode_context_->dynamicGlobalCollisionDistance() : 0.0);

  global_path_searcher_ = std::make_unique<GlobalPathSearcher>();
  global_path_searcher_->configure(tf_, astar_planner_.get(),
                                   smac_planner_.get(), use_smac_,
                                   allow_unknown_, tolerance_, logger_, clock_);
  global_path_searcher_->setQuery(global_query);

  local_path_processor_ = std::make_unique<LocalPathProcessor>();
  local_path_processor_->configure(lookahead_dist_, minco_config.max_vel,
                                   minco_config.max_acc, traj_goal_tolerance_,
                                   shortcut_peak_cost_slack,
                                   shortcut_mean_cost_slack,
                                   logger_, clock_);

  safety_checker_ = std::make_unique<TrajectorySafetyChecker>();
  safety_checker_->configure(safety_config, logger_, clock_);
  safety_checker_->setQuery(dynamic_query);
  RCLCPP_INFO(logger_,
              "[MincoPlanner] Safety footprint: length=%.3f width=%.3f "
              "margin_per_side=%.3f "
              "sample_dt=%.3f map_timeout=%.3f future_tolerance=%.3f "
              "check_horizon=%.3f "
              "planning_frame=%s rog_frame=%s",
              safety_config.footprint_length, safety_config.footprint_width,
              safety_config.footprint_margin, safety_config.sample_dt,
              safety_config.map_timeout, safety_config.future_tolerance,
              safety_check_horizon_,
              safety_config.planning_frame.c_str(),
              safety_config.rog_frame.c_str());

  opt_path_pub_ =
      node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
          trajectory_topic, rclcpp::QoS(rclcpp::KeepLast(1)));

  backup_path_pub_ =
      node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
          backup_trajectory_topic, rclcpp::QoS(rclcpp::KeepLast(1)));

  odom_callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fsm_callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  safety_callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  request_lease_callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto odom_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  rclcpp::SubscriptionOptions odom_options;
  odom_options.callback_group = odom_callback_group_;
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, odom_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg) {
          return;
        }

        auto node = node_.lock();
        if (node) {
          planner_perf_monitor_.recordOdomCallback(node->now(),
                                                   msg->header.stamp);
        }

        {
          std::lock_guard<std::mutex> lk(odom_mutex_);
          latest_odom_ = *msg;
          has_latest_odom_ = true;
        }
      },
      odom_options);

  visualizer_ = std::make_unique<Visualizer>();
  visualizer_->configure(parent, output_frame_, costmap_ros_);

  minco_optimizer_ = std::make_unique<MincoOptimizer>(minco_config);
  minco_optimizer_->setMap(mode_context_ ? mode_context_->dynamicQuery()
                                         : nullptr);

  corridor_gen_ = std::make_shared<SimpleCorridorGenerator>();
  corridor_gen_->setMap(mode_context_ ? mode_context_->dynamicQuery()
                                      : nullptr);
  corridor_gen_->setSafetyMargins(corridor_robot_radius, corridor_extra_margin);

  backup_opt_ = std::make_unique<traj_opt::BackupTrajOpt>();
  yaw_opt_ = std::make_unique<traj_opt::YawTrajOpt>(max_yaw_dot);

  recovery_server_ = std::make_shared<RecoverServer>();
  recovery_server_->configure(recovery_server_config_);

  // Planner handle for FSM (non-owning; lifetime managed by pluginlib).
  planner_handle_ = MincoPlanner::Ptr(this, [](MincoPlanner *) {});

  // High-level FSM @ 20Hz.
  fsm_ = std::make_unique<MincoFsm>(
      planner_handle_, recovery_server_, failed_replan_retry_period_,
      successful_replan_period_);
  fsm_timer_ = node->create_timer(
      std::chrono::duration<double>(1.0 / 20.0),
      [this]() {
        std::lock_guard<std::mutex> execution_lock(fsm_execution_mutex_);
        if (isLifecycleActive() && fsm_) {
          fsm_->callMainFsmOnce();
        }
      },
      fsm_callback_group_);

  // Asynchronous safety monitor @ 20Hz.
  safety_timer_ =
      node->create_timer(std::chrono::duration<double>(1.0 / 20.0),
                         std::bind(&MincoPlanner::safetyTimerCallback, this),
                         safety_callback_group_);
  if (request_lease_.enabled()) {
    const auto min_period = std::chrono::duration<double>(0.001);
    const auto max_period = std::chrono::duration<double>(0.050);
    const auto check_period =
        std::clamp(request_lease_.timeout() / 4.0, min_period, max_period);
    request_lease_timer_ = node->create_wall_timer(
        check_period, std::bind(&MincoPlanner::requestLeaseTimerCallback, this),
        request_lease_callback_group_);
    request_lease_timer_->cancel();
  }
  fsm_timer_->cancel();
  safety_timer_->cancel();

  RCLCPP_INFO(logger_,
              "[MincoPlanner] Planning-request heartbeat lease: %s "
              "(timeout=%.3f s, steady clock).",
              request_lease_.enabled() ? "enabled" : "disabled",
              request_lease_timeout_);

  on_set_parameters_callback_handle_ = node->add_on_set_parameters_callback(
      std::bind(&MincoPlanner::onSetParameters, this, std::placeholders::_1));
}

void MincoPlanner::setMap(
    const std::shared_ptr<rog_map::MapQueryInterface> &map) {
  rog_query_raw_ = map;
  rebuildModeDependentQueries();
}

void MincoPlanner::activate() {
  // The static layer can receive its transient-local map after plugin
  // configuration. Refresh immutable global overlays now that the costmap has
  // been activated, before any planning timer is allowed to run.
  rebuildModeDependentQueries();
  {
    std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
    planning_session_.activate();
    request_lease_.reset();
    advancePlanningStampLocked();
    invalidateTrajectoryLocked();
    has_active_goal_ = false;
    active_goal_session_ = 0U;
    has_pending_goal_ = false;
    pending_goal_session_ = 0U;
    latest_global_path_.clear();
    latest_global_path_session_ = 0U;
    if (visualizer_) {
      visualizer_->updateGlobalPath(nav_msgs::msg::Path{});
    }
    publishBlockCommandLocked();
  }
  if (fsm_) {
    fsm_->cancelGoal();
  }
  if (fsm_timer_) {
    fsm_timer_->reset();
  }
  if (safety_timer_) {
    safety_timer_->reset();
  }
  if (request_lease_timer_) {
    request_lease_timer_->reset();
  }
}

void MincoPlanner::deactivate() {
  if (fsm_timer_) {
    fsm_timer_->cancel();
  }
  if (safety_timer_) {
    safety_timer_->cancel();
  }
  if (request_lease_timer_) {
    request_lease_timer_->cancel();
  }
  {
    std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
    planning_session_.deactivate();
    request_lease_.reset();
    advancePlanningStampLocked();
    invalidateTrajectoryLocked();
    has_active_goal_ = false;
    active_goal_session_ = 0U;
    has_pending_goal_ = false;
    pending_goal_session_ = 0U;
    latest_global_path_.clear();
    latest_global_path_session_ = 0U;
    if (visualizer_) {
      visualizer_->updateGlobalPath(nav_msgs::msg::Path{});
    }
    publishBlockCommandLocked();
  }
  // A callback that passed its initial active check must finish observing the
  // invalidated generation before lifecycle teardown proceeds.
  { std::lock_guard<std::mutex> lock(fsm_execution_mutex_); }
  { std::lock_guard<std::mutex> lock(safety_execution_mutex_); }
  { std::lock_guard<std::mutex> lock(request_lease_execution_mutex_); }
  if (fsm_) {
    fsm_->cancelGoal();
  }
}

void MincoPlanner::cleanup() {
  planner_perf_monitor_.close();

  on_set_parameters_callback_handle_.reset();

  if (fsm_timer_) {
    fsm_timer_->cancel();
  }
  if (safety_timer_) {
    safety_timer_->cancel();
  }
  if (request_lease_timer_) {
    request_lease_timer_->cancel();
  }

  // Invalidate a safety check that may already be in flight and make sure a
  // later configure cycle cannot observe or reuse trajectories from this one.
  {
    std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
    planning_session_.deactivate();
    request_lease_.reset();
    advancePlanningStampLocked();
    invalidateTrajectoryLocked();
    has_active_goal_ = false;
    active_goal_session_ = 0U;
    has_pending_goal_ = false;
    pending_goal_session_ = 0U;
    latest_global_path_.clear();
    latest_global_path_session_ = 0U;
    if (visualizer_) {
      visualizer_->updateGlobalPath(nav_msgs::msg::Path{});
    }
    publishBlockCommandLocked();
  }
  { std::lock_guard<std::mutex> lock(fsm_execution_mutex_); }
  { std::lock_guard<std::mutex> lock(safety_execution_mutex_); }
  { std::lock_guard<std::mutex> lock(request_lease_execution_mutex_); }
  fsm_timer_.reset();
  safety_timer_.reset();
  request_lease_timer_.reset();

  fsm_.reset();
  recovery_server_.reset();
  planner_handle_.reset();

  if (visualizer_) {
    visualizer_->cleanup();
    visualizer_.reset();
  }

  astar_planner_.reset();
  smac_planner_.reset();
  global_path_searcher_.reset();
  local_path_processor_.reset();
  safety_checker_.reset();
  mode_context_.reset();
  minco_optimizer_.reset();
  corridor_gen_.reset();
  backup_opt_.reset();
  yaw_opt_.reset();
  opt_path_pub_.reset();
  backup_path_pub_.reset();
  odom_sub_.reset();
  odom_callback_group_.reset();
  fsm_callback_group_.reset();
  safety_callback_group_.reset();
  request_lease_callback_group_.reset();
  costmap_ros_.reset();
  if (rog_map_ros_) {
    rog_map_ros_->shutdownRosCallbacks();
  }
  map_.reset();
  rog_query_raw_.reset();
  rog_map_ros_.reset();
}

void MincoPlanner::invalidateTrajectoryLocked() {
  last_traj_.clear();
  last_yaw_traj_.clear();
  has_last_traj_ = false;
  has_last_yaw_traj_ = false;
  last_trajectory_session_ = 0U;
  ++trajectory_generation_;
  emergency_stop_latched_ = false;
  is_traj_safe_.store(true);
}

void MincoPlanner::publishBlockCommandLocked(bool mirror_to_backup) {
  if (!opt_path_pub_ || !clock_) {
    return;
  }
  ros_interfaces::msg::MpcPositionCommand block;
  block.header.frame_id = output_frame_;
  block.header.stamp = rosNow();
  block.planning_stamp = planning_stamp_;
  block.command_flag = ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND;
  block.mpc_horizon = 0U;
  opt_path_pub_->publish(block);
  if (mirror_to_backup && backup_path_pub_) {
    backup_path_pub_->publish(block);
  }
}

void MincoPlanner::advancePlanningStampLocked() {
  const builtin_interfaces::msg::Time now = rosNow();
  planning_stamp_ =
      planning_contract::monotonicPlanningStamp(now, planning_stamp_);
}

bool MincoPlanner::expirePlanningRequestLeaseLocked(
    PlanningRequestLease::TimePoint now) {
  const auto expired_session = request_lease_.consumeExpired(now);
  if (!expired_session || !planning_session_.accepts(*expired_session)) {
    return false;
  }

  (void)planning_session_.beginSession();
  advancePlanningStampLocked();
  invalidateTrajectoryLocked();
  active_goal_ = geometry_msgs::msg::PoseStamped{};
  has_active_goal_ = false;
  active_goal_session_ = 0U;
  pending_goal_ = geometry_msgs::msg::PoseStamped{};
  has_pending_goal_ = false;
  pending_goal_session_ = 0U;
  latest_global_path_.clear();
  latest_global_path_session_ = 0U;
  if (visualizer_) {
    visualizer_->updateGlobalPath(nav_msgs::msg::Path{});
  }
  emergency_stop_latched_ = true;
  is_traj_safe_.store(false);
  publishBlockCommandLocked(true);

  RCLCPP_WARN(
      logger_,
      "[MincoPlanner] Planning-request heartbeat lease expired for session "
      "%llu; "
      "old work invalidated and mirrored BLOCK published with token %d.%09u.",
      static_cast<unsigned long long>(*expired_session), planning_stamp_.sec,
      planning_stamp_.nanosec);
  return true;
}

void MincoPlanner::requestLeaseTimerCallback() {
  std::lock_guard<std::mutex> execution_lock(request_lease_execution_mutex_);
  std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
  (void)expirePlanningRequestLeaseLocked(PlanningRequestLease::Clock::now());
}

MincoPlanner::PlanningSessionHandle MincoPlanner::acceptPlanningGoal(
    const geometry_msgs::msg::PoseStamped &normalized_goal) {
  std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
  const PlanningRequestLease::TimePoint steady_now =
      PlanningRequestLease::Clock::now();
  (void)expirePlanningRequestLeaseLocked(steady_now);
  if (!planning_session_.active()) {
    PlanningSessionHandle rejected;
    return rejected;
  }

  if (has_active_goal_ && planning_session_.accepts(active_goal_session_) &&
      planning_contract::samePlanningGoal(active_goal_, normalized_goal)) {
    request_lease_.refresh(active_goal_session_, steady_now);
    return {active_goal_session_, planning_stamp_};
  }

  const uint64_t session = planning_session_.beginSession();
  advancePlanningStampLocked();
  invalidateTrajectoryLocked();

  active_goal_ = normalized_goal;
  active_goal_session_ = session;
  has_active_goal_ = true;
  pending_goal_ = normalized_goal;
  pending_goal_session_ = session;
  has_pending_goal_ = true;
  latest_global_path_.clear();
  latest_global_path_session_ = 0U;
  if (visualizer_) {
    visualizer_->updateGlobalPath(nav_msgs::msg::Path{});
  }
  request_lease_.refresh(session, steady_now);
  publishBlockCommandLocked();

  return {session, planning_stamp_};
}

uint64_t MincoPlanner::beginPlanningSession() {
  uint64_t session = 0U;
  {
    std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
    session = planning_session_.beginSession();
    request_lease_.reset();
    advancePlanningStampLocked();
    invalidateTrajectoryLocked();
    has_active_goal_ = false;
    active_goal_session_ = 0U;
    has_pending_goal_ = false;
    pending_goal_session_ = 0U;
    latest_global_path_.clear();
    latest_global_path_session_ = 0U;
    if (visualizer_) {
      visualizer_->updateGlobalPath(nav_msgs::msg::Path{});
    }
    publishBlockCommandLocked();
  }
  return session;
}

bool MincoPlanner::isLifecycleActive() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return planning_session_.active();
}

bool MincoPlanner::isPlanningSessionCurrent(uint64_t session) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return planning_session_.accepts(session);
}

rcl_interfaces::msg::SetParametersResult MincoPlanner::onSetParameters(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const std::string planner_mode_param = name_ + ".planner_mode";
  const auto is_configure_time_param =
      [this, &planner_mode_param](const std::string &param_name) {
        return param_name == planner_mode_param ||
               param_name == name_ + ".frames.map_frame" ||
               param_name == name_ + ".frames.rog_frame" ||
               param_name == name_ + ".frames.physical_base_frame" ||
               param_name == name_ + ".priormap.use_nav2_global_search" ||
               param_name == name_ + ".priormap.dynamic_global_obstacle.enable" ||
               param_name == name_ + ".priormap.dynamic_global_obstacle.collision_distance" ||
               param_name == name_ + ".priormap.clip_seed_by_rog_boundary" ||
               param_name == name_ + ".priormap.rog_boundary_margin" ||
               param_name == name_ + ".priormap.rog_boundary_sample_step" ||
               param_name ==
                   name_ + ".priormap.ground_edge_avoidance.enable" ||
               param_name ==
                   name_ + ".priormap.ground_edge_avoidance.max_step" ||
               param_name ==
                   name_ + ".priormap.ground_edge_avoidance.max_slope_deg" ||
               param_name ==
                   name_ + ".priormap.ground_edge_avoidance.clearance_radius" ||
               param_name == name_ +
                                 ".priormap.ground_edge_avoidance."
                                 "lethal_clearance_radius" ||
               param_name ==
                   name_ + ".priormap.ground_edge_avoidance.clearance_cost" ||
               param_name == name_ + ".exploration.boundary_margin" ||
               param_name == name_ + ".exploration.boundary_sample_step" ||
               param_name == name_ + ".exploration.unknown_as_occupied" ||
               param_name == name_ + ".exploration.prefer_goal_direction" ||
               param_name == name_ + ".local_path.shortcut_peak_cost_slack" ||
               param_name == name_ + ".local_path.shortcut_mean_cost_slack" ||
               param_name == name_ + ".minco_optimizer.safe_dist" ||
               param_name == name_ + ".minco_optimizer.collision_dist" ||
               param_name == name_ + ".safety.footprint_length" ||
               param_name == name_ + ".safety.footprint_width" ||
               param_name == name_ + ".safety.footprint_margin" ||
               param_name == name_ + ".safety.sample_dt" ||
               param_name == name_ + ".safety.map_timeout" ||
               param_name == name_ + ".safety.future_tolerance" ||
               param_name == name_ + ".safety.check_horizon" ||
               param_name ==
                   name_ + ".safety.collision_cache_reuse_max_duration" ||
               param_name == name_ + ".request_lease_timeout" ||
               param_name ==
                   name_ + ".minco_optimizer.failed_replan_retry_period" ||
               param_name ==
                   name_ + ".minco_optimizer.successful_replan_period" ||
               param_name == name_ + ".minco_optimizer.time_allocation_iters";
      };
  const std::string max_vel_param = name_ + ".minco_optimizer.max_velocity";
  const std::string max_acc_param = name_ + ".minco_optimizer.max_acceleration";
  const std::string penalty_pos_param =
      name_ + ".minco_optimizer.penalty_weight_pos";
  const std::string penalty_vel_param =
      name_ + ".minco_optimizer.penalty_weight_vel";
  const std::string penalty_acc_param =
      name_ + ".minco_optimizer.penalty_weight_acc";
  const std::string penalty_att_param =
      name_ + ".minco_optimizer.penalty_weight_att";
  const std::string penalty_time_barrier_param =
      name_ + ".minco_optimizer.penalty_weight_time_barrier";

  double next_max_vel = minco_config.max_vel;
  double next_max_acc = minco_config.max_acc;
  double next_penalty_pos = (minco_config.penaltyWeights.size() > 0)
                                ? minco_config.penaltyWeights(0)
                                : 1000.0;
  double next_penalty_vel = (minco_config.penaltyWeights.size() > 1)
                                ? minco_config.penaltyWeights(1)
                                : 1000.0;
  double next_penalty_acc = (minco_config.penaltyWeights.size() > 2)
                                ? minco_config.penaltyWeights(2)
                                : 10000.0;
  double next_penalty_att = (minco_config.penaltyWeights.size() > 3)
                                ? minco_config.penaltyWeights(3)
                                : 1000.0;
  double next_penalty_time_barrier = (minco_config.penaltyWeights.size() > 4)
                                         ? minco_config.penaltyWeights(4)
                                         : 100.0;
  bool optimizer_config_changed = false;

  for (const auto &param : parameters) {
    const auto &param_name = param.get_name();

    auto parse_numeric = [&](double &out) -> bool {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        out = param.as_double();
        return true;
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        out = static_cast<double>(param.as_int());
        return true;
      }
      return false;
    };

    if (is_configure_time_param(param_name)) {
      result.successful = false;
      result.reason = "Parameter is configure-time only; restart "
                      "planner_server to apply: " +
                      param_name;
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
      return result;
    }

    if (param_name == max_vel_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) ||
          candidate <= 0.0) {
        result.successful = false;
        result.reason = "Parameter must be a positive number: " + max_vel_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }
      next_max_vel = candidate;
      optimizer_config_changed = true;
      continue;
    }

    if (param_name == max_acc_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) ||
          candidate <= 0.0) {
        result.successful = false;
        result.reason = "Parameter must be a positive number: " + max_acc_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }
      next_max_acc = candidate;
      optimizer_config_changed = true;
      continue;
    }

    if (param_name == penalty_pos_param || param_name == penalty_vel_param ||
        param_name == penalty_acc_param || param_name == penalty_att_param ||
        param_name == penalty_time_barrier_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) ||
          candidate < 0.0) {
        result.successful = false;
        result.reason =
            "Penalty weight must be a non-negative number: " + param_name;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }

      if (param_name == penalty_pos_param) {
        next_penalty_pos = candidate;
      } else if (param_name == penalty_vel_param) {
        next_penalty_vel = candidate;
      } else if (param_name == penalty_acc_param) {
        next_penalty_acc = candidate;
      } else if (param_name == penalty_att_param) {
        next_penalty_att = candidate;
      } else {
        next_penalty_time_barrier = candidate;
      }

      optimizer_config_changed = true;
      continue;
    }
  }

  if (optimizer_config_changed) {
    minco_config.max_vel = next_max_vel;
    minco_config.max_acc = next_max_acc;

    minco_config.penaltyWeights.resize(5);
    minco_config.penaltyWeights(0) = next_penalty_pos;
    minco_config.penaltyWeights(1) = next_penalty_vel;
    minco_config.penaltyWeights(2) = next_penalty_acc;
    minco_config.penaltyWeights(3) = next_penalty_att;
    minco_config.penaltyWeights(4) = next_penalty_time_barrier;

    minco_config.magnitudeBounds.resize(3);
    minco_config.magnitudeBounds(0) = minco_config.safe_dist;
    minco_config.magnitudeBounds(1) = minco_config.max_vel;
    minco_config.magnitudeBounds(2) = minco_config.max_acc;

    if (minco_optimizer_) {
      minco_optimizer_->setConfig(minco_config);
    }
    if (local_path_processor_) {
      local_path_processor_->updateLimits(
          minco_config.max_vel, minco_config.max_acc, traj_goal_tolerance_);
    }

    RCLCPP_INFO(logger_,
                "[MincoPlanner] Updated optimizer params: vmax=%.3f, "
                "amax=%.3f, w=[%.3f %.3f %.3f %.3f %.3f]",
                minco_config.max_vel, minco_config.max_acc,
                minco_config.penaltyWeights(0), minco_config.penaltyWeights(1),
                minco_config.penaltyWeights(2), minco_config.penaltyWeights(3),
                minco_config.penaltyWeights(4));
  }

  return result;
}

// -----------------------------------------------------------------------------
// 3) Core business interface
// -----------------------------------------------------------------------------

bool MincoPlanner::normalizePoseToFrame(
    const geometry_msgs::msg::PoseStamped &in,
    const std::string &fallback_frame, const std::string &target_frame,
    const std::string &context, geometry_msgs::msg::PoseStamped &out) const {
  out = in;
  if (out.header.frame_id.empty()) {
    out.header.frame_id = fallback_frame;
    RCLCPP_WARN(logger_,
                "[MincoPlanner] %s pose frame is empty, treating it as %s.",
                context.c_str(), fallback_frame.c_str());
  }

  if (out.header.frame_id == target_frame) {
    out.header.frame_id = target_frame;
    return true;
  }

  if (!tf_) {
    RCLCPP_ERROR(logger_,
                 "[MincoPlanner] Cannot transform %s pose from %s to %s: TF "
                 "buffer is null.",
                 context.c_str(), out.header.frame_id.c_str(),
                 target_frame.c_str());
    return false;
  }

  try {
    out = tf_->transform(out, target_frame);
    out.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(logger_,
                 "[MincoPlanner] Failed to transform %s pose from %s to %s: %s",
                 context.c_str(), in.header.frame_id.c_str(),
                 target_frame.c_str(), ex.what());
    return false;
  }
}

nav_msgs::msg::Path
MincoPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                         const geometry_msgs::msg::PoseStamped &goal,
                         std::function<bool()> cancel_checker) {
  const rclcpp::Time request_stamp = rosNow();
  nav_msgs::msg::Path path;
  path.header.stamp = request_stamp;
  path.header.frame_id = output_frame_;

  if (!isLifecycleActive()) {
    RCLCPP_WARN(
        logger_,
        "[MincoPlanner] createPlan rejected while planner is inactive.");
    return path;
  }

  if (cancel_checker && cancel_checker()) {
    cancelGoal();
    return path;
  }

  geometry_msgs::msg::PoseStamped normalized_start;
  geometry_msgs::msg::PoseStamped normalized_goal;
  const bool start_ok =
      normalizePoseToFrame(start, planning_frame_, planning_frame_,
                           "createPlan start", normalized_start);
  const bool goal_ok =
      normalizePoseToFrame(goal, planning_frame_, planning_frame_,
                           "createPlan goal", normalized_goal);
  if (!start_ok || !goal_ok) {
    RCLCPP_ERROR(logger_,
                 "[MincoPlanner] Failed to normalize createPlan pose(s) to "
                 "planning frame %s; reject pending goal.",
                 planning_frame_.c_str());
    cancelGoal();
    return path;
  }

  if (cancel_checker && cancel_checker()) {
    cancelGoal();
    return path;
  }

  const PlanningSessionHandle session = acceptPlanningGoal(normalized_goal);
  if (session.session == 0U) {
    RCLCPP_WARN(
        logger_,
        "[MincoPlanner] createPlan goal rejected while planner is inactive.");
    return path;
  }

  // Path.header.stamp is the stable planning token consumed by Nav2's
  // controller contract. Pose stamps remain the time of this individual
  // createPlan call for TF/debug freshness.
  path.header.stamp = session.planning_stamp;
  std_msgs::msg::Header pose_header;
  pose_header.frame_id = output_frame_;
  pose_header.stamp = request_stamp;

  // Keep a minimal path for Nav2 callers (e.g., visualization/debug).
  normalized_start.header = pose_header;
  normalized_goal.header = pose_header;
  path.poses.push_back(normalized_start);
  path.poses.push_back(normalized_goal);

  {
    std::lock_guard<std::mutex> session_lock(mutex_);
    if (!planning_session_.accepts(session.session)) {
      path.poses.clear();
      return path;
    }
  }

  return path;
}

bool MincoPlanner::PlanGlobalPath(const geometry_msgs::msg::PoseStamped &start,
                                  const geometry_msgs::msg::PoseStamped &goal,
                                  uint64_t expected_session) {
  const bool record_perf = planner_perf_monitor_.detailedCsvEnabled();
  const auto search_start = record_perf
                                ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};
  auto record_search_time = [&]() {
    if (!record_perf) {
      return;
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - search_start)
            .count();
    std::lock_guard<std::mutex> perf_lock(perf_mutex_);
    last_global_search_time_ms_ = elapsed_ms;
    has_fresh_global_search_time_ = true;
  };

  if (!isPlanningSessionCurrent(expected_session) || !global_path_searcher_ ||
      !mode_context_) {
    return false;
  }
  std::vector<geometry_msgs::msg::PoseStamped> planned_path;
  if (!global_path_searcher_->plan(start, goal, *mode_context_, planned_path)) {
    record_search_time();
    return false;
  }
  record_search_time();

  {
    std::scoped_lock lock(mutex_, path_mutex_);
    if (!planning_session_.accepts(expected_session)) {
      return false;
    }
    latest_global_path_ = std::move(planned_path);
    latest_global_path_session_ = expected_session;
    if (latest_global_path_.size() < 2U) {
      return false;
    }

    if (visualizer_) {
      nav_msgs::msg::Path global_path_msg;
      global_path_msg.header.stamp = rosNow();
      global_path_msg.header.frame_id = output_frame_;
      global_path_msg.poses = latest_global_path_;
      visualizer_->updateGlobalPath(global_path_msg);
    }
  }
  return true;
}

bool MincoPlanner::ReplanLocal(
    const geometry_msgs::msg::PoseStamped &current_pose,
    uint64_t expected_session) {
  const bool record_perf = planner_perf_monitor_.detailedCsvEnabled();
  const auto replan_start = record_perf
                                ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};
  std::optional<MincoPerfSample> perf;
  if (record_perf) {
    perf.emplace();
    perf->stamp_ros = rosNow().seconds();
    perf->stamp_steady_ns = PlannerPerformanceMonitor::steadyNowNs();
    perf->planner_mode = mode_params_.planner_mode;
    std::lock_guard<std::mutex> perf_lock(perf_mutex_);
    if (has_fresh_global_search_time_) {
      perf->global_search_time_ms = last_global_search_time_ms_;
      has_fresh_global_search_time_ = false;
    }
  }
  auto finish = [&](bool success, const std::string &reason) {
    std::string effective_reason = reason;
    if (success && !isPlanningSessionCurrent(expected_session)) {
      success = false;
      effective_reason = "SESSION_INVALIDATED";
    }
    // ReplanLocal is a public entry point, so fail-safe behavior must not
    // depend on every caller remembering to recheck the cached trajectory.
    if (!success) {
      (void)evaluateCachedTrajectorySafety(&current_pose);
    }
    if (perf) {
      perf->success = success;
      perf->failure_reason = success ? "NONE" : effective_reason;
      perf->total_replan_time_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - replan_start)
              .count();
      {
        std::lock_guard<std::mutex> perf_lock(perf_mutex_);
        if (last_minco_perf_stamp_ns_ > 0 &&
            perf->stamp_steady_ns > last_minco_perf_stamp_ns_) {
          const double dt_sec = static_cast<double>(perf->stamp_steady_ns -
                                                    last_minco_perf_stamp_ns_) *
                                1.0e-9;
          if (dt_sec > 1.0e-9) {
            perf->planner_hz = 1.0 / dt_sec;
          }
        }
        last_minco_perf_stamp_ns_ = perf->stamp_steady_ns;
      }
      planner_perf_monitor_.recordPlannerSample(*perf);
    }
    return success;
  };

  if (!isPlanningSessionCurrent(expected_session) || !minco_optimizer_ ||
      !mode_context_ || !local_path_processor_) {
    return finish(false, "OPTIMIZER_FAILED");
  }

  // Snapshot the global goal for end-state logic.
  Eigen::Vector3d global_goal(0.0, 0.0, 0.0);
  double goal_yaw = 0.0;
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  bool has_current_global_path = false;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!latest_global_path_.empty() &&
        latest_global_path_session_ == expected_session) {
      global_path_snapshot = latest_global_path_;
      global_goal.x() = latest_global_path_.back().pose.position.x;
      global_goal.y() = latest_global_path_.back().pose.position.y;
      global_goal.z() = 0.0;
      goal_yaw =
          utils::quaternionToYaw(latest_global_path_.back().pose.orientation);
      has_current_global_path = true;
    }
  }
  if (!has_current_global_path) {
    return finish(false, "OPTIMIZER_FAILED");
  }

  const LocalPathSeed seed = local_path_processor_->buildSeed(
      global_path_snapshot, current_pose, *mode_context_,
      [this](const Eigen::Vector3d &position, double yaw) {
        return safety_checker_ &&
               safety_checker_->checkFootprint(position, yaw);
      });
  if (visualizer_) {
    if (!seed.dense_path.empty()) {
      visualizer_->updateLocalEndPoint(seed.dense_path.back(),
                                       seed.local_end_is_goal);
    } else {
      visualizer_->clearLocalEndPoint();
    }
  }
  if (!seed.valid) {
    if (republishSafeCachedTrajectory(current_pose, expected_session,
                                      "LOCAL_SEED_COLLISION")) {
      return finish(true, "CACHED_TRAJECTORY_REPUBLISHED");
    }
    return finish(false, "COLLISION");
  }
  std::vector<Eigen::Vector3d> sparse_path = seed.sparse_waypoints;
  const bool stop_at_local_end = seed.stop_at_local_end;

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;
  header_msg.stamp = rosNow();

  // 4. Determine state (HOT/COLD).
  PlanningState state = PlanningState::COLD_START;
  traj_opt::Trajectory last_traj_snapshot;
  bool has_last_traj_snapshot = false;
  double last_traj_start_WT = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = determinePlanningState(current_pose.pose, sparse_path);
    if (has_last_traj_) {
      last_traj_snapshot = last_traj_;
      has_last_traj_snapshot = true;
      last_traj_start_WT = last_traj_.start_WT;
    }
  }

  if (state == PlanningState::EMERGENCY_STOP) {
    return finish(false, "RECOVERY_TRIGGERED");
  }

  // 5. Prepare start state.
  Eigen::Matrix3d start_state;
  vec_Vec3f shifted_waypoints;
  VecDf shifted_durations;
  bool has_shifted_seed = false;
  if (state == PlanningState::HOT_START) {
    const double now = rosNow().seconds() + 0.005; // small buffer
    const double t_dur = now - last_traj_start_WT;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      prepareHotStart(current_pose.pose, start_state);
    }

    // Extract remaining trajectory segment as shifted warm-start seed.
    if (has_last_traj_snapshot) {
      traj_opt::Trajectory remain;
      const double total = last_traj_snapshot.getTotalDuration();
      if (std::isfinite(t_dur) && t_dur > 0.0 && total > t_dur + 1e-3 &&
          last_traj_snapshot.getPartialTrajectoryByTime(t_dur, total, remain)) {
        shifted_waypoints = remain.getWaypoints();
        shifted_durations = remain.getDurations();
        has_shifted_seed =
            (!shifted_waypoints.empty() && shifted_durations.size() > 0);
      }
    }
  } else {
    prepareColdStart(current_pose.pose, start_state, sparse_path);
    // Avoid reusing stale warm-start guesses.
    minco_optimizer_->setInitPsAndTs(vec_Vec3f{}, VecDf{});
  }
  // P1b: If the robot is inside an obstacle (ESDF dist < 0), project the
  // start position to the nearest free space along the ESDF gradient.
  if (safety_checker_) {
    constexpr double kMargin = 0.05;
    Eigen::Vector3d start_pos = start_state.col(0);
    if (safety_checker_->projectOutOfObstacle(start_pos, kMargin)) {
      start_state.col(0) = start_pos;
    }
  }

  // 6. Generate backup trajectory (safety).
  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);

  // 7. Prepare MINCO optimization.
  traj_opt::Trajectory opt_traj;
  Eigen::Matrix3d end_state;
  end_state.setZero();
  end_state.col(0) = sparse_path.back();

  // End state logic.
  const double dist_to_goal = (end_state.col(0) - global_goal).head<2>().norm();
  if (!stop_at_local_end && dist_to_goal > 1.0) {
    Eigen::Vector3d tangent(1.0, 0.0, 0.0);
    if (sparse_path.size() >= 2) {
      tangent = sparse_path.back() - sparse_path[sparse_path.size() - 2];
      tangent.z() = 0.0;
      const double n = tangent.head<2>().norm();
      if (n > 0.1) {
        tangent /= n;
      } else {
        tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
      }
    }
    const double v_curr = std::max(0.0, start_state.col(1).head<2>().norm());
    const double amax = std::max(0.0, minco_config.max_acc);
    const double v_max_kinematic =
        std::sqrt(std::max(0.0, v_curr * v_curr + 2.0 * amax * dist_to_goal));
    double local_end_vmax = minco_config.max_vel;
    if (sparse_path.size() >= 3) {
      local_end_vmax = utils::LimitLocalVel(
          sparse_path, sparse_path.size() - 3, minco_config.max_vel,
          minco_config.turn_angle_deadzone, minco_config.turn_angle_saturation,
          minco_config.min_turn_vel, minco_config.decay_power);
    }
    const double v_cmd = std::min(
        {minco_config.max_vel, v_max_kinematic, dist_to_goal, local_end_vmax});
    end_state.col(1) = tangent * v_cmd;
    end_state.col(2).setZero();
  } else {
    end_state.col(1).setZero();
    end_state.col(2).setZero();
  }

  // Remove near-start redundant points from sparse_path.
  while (sparse_path.size() > 2) {
    if ((sparse_path[1] - start_state.col(0)).norm() < 0.2) {
      sparse_path.erase(sparse_path.begin() + 1);
    } else {
      break;
    }
  }

  // 7.5 Initial guess Ps/Ts for optimizer (all cases).
  const int N = static_cast<int>(sparse_path.size()) - 1;
  VecDf local_vmaxs(N);
  if (N > 0) {
    vec_Vec3f init_ps;
    VecDf init_ts(N);
    PTAllocation(sparse_path, start_state, stop_at_local_end, state,
                 has_shifted_seed, shifted_waypoints, shifted_durations,
                 init_ps, init_ts, local_vmaxs);

    minco_optimizer_->setInitPsAndTs(init_ps, init_ts);
  }

  // 8. Optimize.
  const auto opt_start_steady = record_perf
                                    ? std::chrono::steady_clock::now()
                                    : std::chrono::steady_clock::time_point{};
  if (perf) {
    perf->local_search_time_ms = std::chrono::duration<double, std::milli>(
                                     opt_start_steady - replan_start)
                                     .count();
  }
  auto opt_start_time = rosNow().seconds();
  double final_cost = minco_optimizer_->optimize(
      sparse_path, start_state, end_state, local_vmaxs, opt_traj);
  if (perf) {
    perf->optimizer_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - opt_start_steady)
            .count();
  }

  // const double max_allowed_cost = 6000.0;
  // if (!std::isfinite(final_cost) || final_cost > max_allowed_cost) {
  if (!std::isfinite(final_cost)) {
    Eigen::Vector3d first_segment = Eigen::Vector3d::Zero();
    if (sparse_path.size() >= 2U) {
      first_segment = sparse_path[1] - sparse_path[0];
    }
    const double first_segment_length = first_segment.head<2>().norm();
    const double start_speed = start_state.col(1).head<2>().norm();
    const double direction_cosine =
      (first_segment_length > 1.0e-6 && start_speed > 1.0e-6)
        ? start_state.col(1).head<2>().dot(first_segment.head<2>()) /
          (start_speed * first_segment_length)
        : 1.0;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[MincoPlanner] MINCO optimizer failed: ret=%d iterations=%d query_failures=%llu "
      "state=%s waypoints=%zu stop_at_end=%s start=(%.3f,%.3f) speed=%.3f "
      "first_segment=%.3f velocity_direction_cos=%.3f peak_v=%.3f peak_a=%.3f "
      "retime_iters=%d.",
      minco_optimizer_->lastReturnCode(), minco_optimizer_->lastIterationCount(),
      static_cast<unsigned long long>(minco_optimizer_->lastQueryFailureCount()),
      state == PlanningState::HOT_START ? "HOT" : "COLD", sparse_path.size(),
      stop_at_local_end ? "true" : "false", start_state(0, 0), start_state(1, 0),
      start_speed, first_segment_length, direction_cosine,
      minco_optimizer_->lastPeakVelocity(), minco_optimizer_->lastPeakAcceleration(),
      minco_optimizer_->lastTimeAllocationIterations());

    if (visualizer_) {
      visualizer_->clearCandidateTrajectory("OPTIMIZER_FAILED");
    }

    if (republishSafeCachedTrajectory(current_pose, expected_session,
                                      "OPTIMIZER_FAILED")) {
      return finish(true, "CACHED_TRAJECTORY_REPUBLISHED");
    }
    return finish(false, "OPTIMIZER_FAILED");
  }

  auto opt_end_time = rosNow().seconds();
  double opt_duration = opt_end_time - opt_start_time;
  std::cout << GREEN
            << "[MincoPlanner] Minco optimization time: " << opt_duration
            << " seconds, " << "raw cost: " << final_cost << ", retime_iters: "
            << minco_optimizer_->lastTimeAllocationIterations()
            << ", peak |v|: " << minco_optimizer_->lastPeakVelocity()
            << ", peak |a|: " << minco_optimizer_->lastPeakAcceleration()
            << RESET << std::endl;

  // 8.5 Quality gating (hard validation) before publishing.
  const bool validation_ok = validateTrajectory(opt_traj, end_state.col(0));
  if (visualizer_) {
    visualizer_->updateCandidateTrajectory(
        opt_traj, opt_duration, validation_ok,
        validation_ok ? "NONE" : last_validation_failure_reason_);
  }

  if (!validation_ok) {
    std::cout << RED
              << "[MincoPlanner] Trajectory validation failed! Rejecting."
              << RESET << std::endl;
    if (republishSafeCachedTrajectory(
            current_pose, expected_session,
            last_validation_failure_reason_.c_str())) {
      return finish(true, "CACHED_TRAJECTORY_REPUBLISHED");
    }
    return finish(false, last_validation_failure_reason_);
  }

  double fallback_yaw = std::numeric_limits<double>::quiet_NaN();
  if (!utils::quaternionToYawChecked(current_pose.pose.orientation,
                                     fallback_yaw)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "[MincoPlanner] Cannot build yaw trajectory from an "
                         "invalid physical robot orientation.");
    return finish(false, "INVALID_PHYSICAL_POSE");
  }

  traj_opt::Trajectory yaw_traj;
  // The observed-prefix gate above validates a clipped seed with the measured
  // chassis yaw.  Rotating toward the path tangent afterward can sweep a
  // rectangular footprint back into the UNKNOWN cells that caused the clip,
  // even though an omnidirectional translation is executable.  Keep yaw fixed
  // for this short stop trajectory; the next rolling replan may resume normal
  // yaw optimization once the newly exposed volume is observed.
  if (shouldOptimizeYawForSeed(use_yaw_opt_, seed)) {
    const bool yaw_success = optimizeYaw(start_state, opt_traj, yaw_traj, state,
                                         current_pose.pose, goal_yaw);
    if (!yaw_success) {
      std::cout << YELLOW
                << "[MincoPlanner] Yaw optimization failed. Falling back to "
                   "constant yaw trajectory."
                << RESET << std::endl;

      Eigen::MatrixXd cMat(3, 6);
      cMat.setZero();
      cMat(0, 5) = std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
      const double yaw_dur = std::max(0.02, opt_traj.getTotalDuration());
      yaw_traj.clear();
      yaw_traj.emplace_back(yaw_dur, cMat);
      yaw_traj.start_WT = opt_traj.start_WT;
    }
  } else {
    Eigen::MatrixXd cMat(3, 6);
    cMat.setZero();
    cMat(0, 5) = std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
    const double yaw_dur = std::max(0.02, opt_traj.getTotalDuration());
    yaw_traj.clear();
    yaw_traj.emplace_back(yaw_dur, cMat);
    yaw_traj.start_WT = opt_traj.start_WT;
  }

  // The rectangular body can collide after yaw rotates even when the position
  // trajectory's centerline is clear. This is the final gate before publish.
  if (!checkExecutableTrajectory(opt_traj, yaw_traj)) {
    last_validation_failure_reason_ = "FOOTPRINT_COLLISION";
    if (visualizer_) {
      visualizer_->updateCandidateTrajectory(opt_traj, opt_duration, false,
                                             last_validation_failure_reason_);
    }
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
                         "[MincoPlanner] Post-yaw footprint validation failed; "
                         "trajectory rejected.");
    if (republishSafeCachedTrajectory(current_pose, expected_session,
                                      "FOOTPRINT_COLLISION")) {
      return finish(true, "CACHED_TRAJECTORY_REPUBLISHED");
    }
    return finish(false, last_validation_failure_reason_);
  }

  // 9. Publish and cache. Cache position/yaw and publish under the same lock so
  // the safety timer cannot publish a stop for an older generation afterward.
  const double t_step = 0.05;
  int steps =
      static_cast<int>(std::ceil(opt_traj.getTotalDuration() / t_step)) + 1;
  steps = std::max(2, steps);
  bool published_for_current_session = false;
  {
    std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
    (void)expirePlanningRequestLeaseLocked(PlanningRequestLease::Clock::now());
    if (planning_session_.accepts(expected_session)) {
      const rclcpp::Time trajectory_stamp = rosNow();
      const double trajectory_start = trajectory_stamp.seconds();
      header_msg.stamp = trajectory_stamp;
      opt_traj.start_WT = trajectory_start;
      yaw_traj.start_WT = trajectory_start;
      last_traj_ = opt_traj;
      last_yaw_traj_ = yaw_traj;
      has_last_traj_ = true;
      has_last_yaw_traj_ = true;
      last_trajectory_session_ = expected_session;
      ++trajectory_generation_;
      emergency_stop_latched_ = false;
      is_traj_safe_.store(true);
      utils::publishOptimizedTrajectory(opt_traj, yaw_traj, opt_path_pub_,
                                        opt_trajectory_id_, header_msg,
                                        planning_stamp_, steps, t_step);
      published_for_current_session = true;
    }
  }
  if (!published_for_current_session) {
    return finish(false, "SESSION_INVALIDATED");
  }

  if (visualizer_) {
    nav_msgs::msg::Path astar_path_msg;
    {
      std::lock_guard<std::mutex> path_lock(path_mutex_);
      astar_path_msg.header.stamp = rosNow();
      astar_path_msg.header.frame_id = output_frame_;
      astar_path_msg.poses = latest_global_path_;
    }
    visualizer_->update(sparse_path, backup_traj, opt_traj, opt_duration,
                        astar_path_msg);
  }

  return finish(true, "NONE");
}

void MincoPlanner::PTAllocation(const std::vector<Eigen::Vector3d> &sparse_path,
                                const Eigen::Matrix3d &start_state,
                                bool stop_at_local_end, PlanningState state,
                                bool has_shifted_seed,
                                const vec_Vec3f &shifted_waypoints,
                                const VecDf &shifted_durations,
                                vec_Vec3f &init_ps, VecDf &init_ts,
                                VecDf &local_vmaxs) const {
  const int N = static_cast<int>(sparse_path.size()) - 1;
  if (N <= 0) {
    init_ps.clear();
    init_ts.resize(0);
    local_vmaxs.resize(0);
    return;
  }

  const double global_vmax = std::max(0.0, minco_config.max_vel);
  const double amax = std::max(1e-3, minco_config.max_acc);
  const double kMinSegTime = 0.1;
  const double kBrakeSafety = 1.2;
  const double max_brake_dist = (global_vmax * global_vmax) / (2.0 * amax);

  local_vmaxs.resize(N);
  local_vmaxs.setConstant(global_vmax);
  init_ts.resize(N);

  init_ps.clear();
  init_ps.reserve(static_cast<size_t>(std::max(0, N - 1)));
  int copyPs = 0;
  if (state == PlanningState::HOT_START && has_shifted_seed) {
    const int oldWp = static_cast<int>(shifted_waypoints.size());
    const int oldPs = std::max(0, oldWp - 2);
    copyPs = std::min(std::max(0, N - 1), oldPs);
  }
  for (int j = 0; j < copyPs; ++j) {
    init_ps.emplace_back(shifted_waypoints[static_cast<size_t>(j + 1)]);
  }
  for (int j = copyPs; j < (N - 1); ++j) {
    init_ps.emplace_back(sparse_path[static_cast<size_t>(j + 1)]);
  }

  std::vector<double> seg_len(static_cast<size_t>(N), 0.0);
  for (int i = 0; i < N; ++i) {
    const double dis = (sparse_path[static_cast<size_t>(i + 1)] -
                        sparse_path[static_cast<size_t>(i)])
                           .head<2>()
                           .norm();
    seg_len[static_cast<size_t>(i)] =
        (std::isfinite(dis) && dis > 0.0) ? dis : 0.0;
  }

  std::vector<double> remain_after(static_cast<size_t>(N), 0.0);
  for (int i = N - 2; i >= 0; --i) {
    remain_after[static_cast<size_t>(i)] =
        remain_after[static_cast<size_t>(i + 1)] +
        seg_len[static_cast<size_t>(i + 1)];
  }

  std::vector<double> local_vmax_vec(static_cast<size_t>(N), global_vmax);
  for (int i = 0; i < N - 1; ++i) {
    local_vmax_vec[static_cast<size_t>(i)] = utils::LimitLocalVel(
        sparse_path, i, global_vmax, minco_config.turn_angle_deadzone,
        minco_config.turn_angle_saturation, minco_config.min_turn_vel,
        minco_config.decay_power);
  }
  utils::VelPropogation(seg_len, amax, local_vmax_vec);
  for (int i = 0; i < N; ++i) {
    local_vmaxs(i) = local_vmax_vec[static_cast<size_t>(i)];
  }

  double v_curr = start_state.col(1).head<2>().norm();
  if (!std::isfinite(v_curr) || v_curr < 0.0) {
    v_curr = 0.0;
  }

  const double local_goal_remain = stop_at_local_end ? 0.0 : max_brake_dist;
  for (int i = 0; i < N; ++i) {
    const bool is_last = (i == N - 1);
    const double L = seg_len[static_cast<size_t>(i)];
    const double remain =
        remain_after[static_cast<size_t>(i)] + local_goal_remain;

    if (L <= 1e-6) {
      init_ts(i) = kMinSegTime;
      continue;
    }

    if (is_last && stop_at_local_end) {
      const double t_stop = v_curr / amax;
      const double t_dist = L / std::max(v_curr, 0.1);
      init_ts(i) = std::max({kMinSegTime, t_dist, kBrakeSafety * t_stop});
      v_curr = 0.0;
      continue;
    }

    const double local_vmax = local_vmax_vec[static_cast<size_t>(i)];
    const double v_next =
        utils::ComputeNextSpeed(v_curr, L, remain, amax, local_vmax);
    init_ts(i) = utils::ComputeSegmentTime(L, v_curr, v_next, local_vmax, amax,
                                           kMinSegTime);
    v_curr = v_next;
  }

  if (state == PlanningState::HOT_START && has_shifted_seed) {
    const int oldN = std::min(N, static_cast<int>(shifted_durations.size()));
    for (int i = 0; i < oldN; ++i) {
      const double t_seed = shifted_durations(i);
      if (std::isfinite(t_seed) && t_seed > 0.02) {
        init_ts(i) = std::max(init_ts(i), t_seed);
      }
    }
  }
}

bool MincoPlanner::makePlan(const geometry_msgs::msg::Pose &start,
                            const geometry_msgs::msg::Pose &goal,
                            double tolerance,
                            std::function<bool()> cancel_checker,
                            nav_msgs::msg::Path &plan) {
  if (!global_path_searcher_ || !mode_context_) {
    return false;
  }
  if (!global_path_searcher_->makePlan(start, goal, *mode_context_, tolerance,
                                       cancel_checker, plan)) {
    return false;
  }
  std::lock_guard<std::mutex> path_lock(path_mutex_);
  latest_global_path_ = plan.poses;
  return true;
}

std::vector<Eigen::Vector3d>
MincoPlanner::extractLocalPath(const Eigen::Vector3d &cur_pos) {
  if (!local_path_processor_ || !mode_context_) {
    return {};
  }
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.pose.position.x = cur_pos.x();
  current_pose.pose.position.y = cur_pos.y();
  current_pose.pose.position.z = cur_pos.z();
  const double current_yaw = getCurrentYawFromOdom();
  current_pose.pose.orientation.z = std::sin(0.5 * current_yaw);
  current_pose.pose.orientation.w = std::cos(0.5 * current_yaw);
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_snapshot = latest_global_path_;
  }
  return local_path_processor_
      ->buildSeed(global_path_snapshot, current_pose, *mode_context_,
                  [this](const Eigen::Vector3d &position, double yaw) {
                    return safety_checker_ &&
                           safety_checker_->checkFootprint(position, yaw);
                  })
      .dense_path;
}

MincoPlanner::PlanningState MincoPlanner::determinePlanningState(
    const geometry_msgs::msg::Pose &start_pose,
    const std::vector<Eigen::Vector3d> &new_path) {
  if (!has_last_traj_) {
    return PlanningState::COLD_START;
  }

  double now = rosNow().seconds() + 0.005;
  double t_dur = now - last_traj_.start_WT;
  if (t_dur <= 0.0 || t_dur >= last_traj_.getTotalDuration()) {
    std::cout
        << YELLOW
        << "[MincoPlanner] Hot Start Rejected: Invalid time duration (t_dur="
        << t_dur << "s)" << RESET << std::endl;
    return PlanningState::COLD_START;
  }

  Eigen::Vector3d current_pos(start_pose.position.x, start_pose.position.y,
                              0.0);
  Eigen::Vector3d pred_pos = last_traj_.getPos(t_dur);
  Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
  double tracking_error = (current_pos - pred_pos).norm();
  Eigen::Vector3d current_speed = getCurrentSpeed();
  // A shifted polynomial is only an optimizer seed. Once the physical robot
  // falls behind on a slope, accepting a metre of position error or nearly a
  // metre per second of velocity error feeds stale momentum into every rolling
  // replan and can keep the reference pointed away from the new local seed.
  const double dynamic_error_threshold =
      0.30 + 0.20 * current_speed.head<2>().norm();
  double vel_error = (current_speed - pred_vel).norm();
  if (tracking_error > dynamic_error_threshold) {
    std::cout << YELLOW << "[MincoPlanner] Large tracking error ("
              << tracking_error << "m). Downgrading to COLD_START." << RESET
              << std::endl;
    return PlanningState::COLD_START;
  }

  if (vel_error > 0.35) {
    std::cout << YELLOW << "[MincoPlanner] Large velocity error (" << vel_error
              << "m/s). Downgrading to COLD_START." << RESET << std::endl;
    return PlanningState::COLD_START;
  }

  if (new_path.size() >= 2) {
    Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
    if (pred_vel.norm() > 0.1) {
      Eigen::Vector3d path_dir = (new_path[1] - new_path[0]).normalized();
      Eigen::Vector3d vel_dir = pred_vel.normalized();
      double dot = vel_dir.dot(path_dir);

      if (dot < 0.5) {
        std::cout
            << YELLOW
            << "[MincoPlanner] Hot Start Rejected: Direction mismatch (dot="
            << dot << ", angle=" << std::acos(dot) * 180.0 / M_PI << " deg)"
            << RESET << std::endl;
        return PlanningState::COLD_START;
      }
    }
  }

  return PlanningState::HOT_START;
}

void MincoPlanner::prepareColdStart(
    const geometry_msgs::msg::Pose &start_pose, Eigen::Matrix3d &start_state,
    const std::vector<Eigen::Vector3d> &sparse_path) {
  start_state.setZero();
  start_state.col(0) =
      Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  Eigen::Vector3d real_speed = getCurrentSpeed();
  real_speed.z() = 0.0;
  Eigen::Vector2d speed_xy = real_speed.head<2>();
  double planar_speed = speed_xy.norm();
  if (!speed_xy.allFinite()) {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[MincoPlanner] Cold-start odometry velocity is non-finite; "
        "starting the optimizer from rest.");
    real_speed.head<2>().setZero();
    planar_speed = 0.0;
  }

  // A cold start is used precisely after the previous optimizer seed was
  // rejected.  Feeding its measured velocity through unchanged can preserve
  // downhill slip or a reverse component that points against the new local
  // path, producing the large path deviation seen in simulation.  Keep a
  // trustworthy forward component, but discard a clearly opposing one.
  if (sparse_path.size() >= 2U && planar_speed > 0.05) {
    const Eigen::Vector2d first_segment =
        (sparse_path[1] - sparse_path[0]).head<2>();
    const double segment_length = first_segment.norm();
    if (std::isfinite(segment_length) && segment_length > 1.0e-3) {
      const Eigen::Vector2d path_direction = first_segment / segment_length;
      const double longitudinal_speed = speed_xy.dot(path_direction);
      const double direction_cos = longitudinal_speed / planar_speed;
      if (!std::isfinite(direction_cos) || direction_cos < 0.25) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "[MincoPlanner] Cold-start velocity rejected against local path "
            "(speed=%.3f m/s, direction_cos=%.3f); starting from rest.",
            planar_speed, direction_cos);
        real_speed.head<2>().setZero();
        planar_speed = 0.0;
      }
    }
  }

  if (std::isfinite(planar_speed) && planar_speed > minco_config.max_vel) {
    real_speed.head<2>() *= minco_config.max_vel / planar_speed;
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[MincoPlanner] Cold-start odometry speed %.3f m/s exceeds the %.3f "
        "m/s trajectory boundary; clamped the planar optimizer state.",
        planar_speed, minco_config.max_vel);
  }
  start_state.col(1) = real_speed;
}

void MincoPlanner::prepareHotStart(const geometry_msgs::msg::Pose &start_pose,
                                   Eigen::Matrix3d &start_state) {
  start_state.setZero();
  start_state.col(0) =
      Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  Eigen::Vector3d measured_speed = getCurrentSpeed();
  measured_speed.z() = 0.0;
  const double planar_speed = measured_speed.head<2>().norm();
  if (std::isfinite(planar_speed) && planar_speed > minco_config.max_vel) {
    measured_speed.head<2>() *= minco_config.max_vel / planar_speed;
  }
  start_state.col(1) = measured_speed;
  // The odometry contract does not provide a trustworthy planar acceleration.
  // Zero is a truthful boundary; the shifted trajectory remains available only
  // as the optimizer's initial guess.
  start_state.col(2).setZero();
}

bool MincoPlanner::optimizeYaw(const Eigen::Matrix3d &start_state,
                               const traj_opt::Trajectory &pos_traj,
                               traj_opt::Trajectory &out_yaw_traj,
                               PlanningState state,
                               const geometry_msgs::msg::Pose &current_pose,
                               double goal_yaw) {
  (void)start_state;
  (void)state;

  if (!yaw_opt_) {
    return false;
  }

  const double pos_dur = pos_traj.getTotalDuration();
  if (!(std::isfinite(pos_dur) && pos_dur > 1e-6)) {
    return false;
  }

  Eigen::Vector4d init_yaw_state = Eigen::Vector4d::Zero();
  Eigen::Vector4d goal_yaw_state = Eigen::Vector4d::Zero();

  // This omnidirectional model controls global vx/vy independently from yaw.
  // Lateral motion or downhill slip therefore says nothing about chassis
  // heading. Every replacement trajectory starts at the measured physical
  // orientation so replanning cannot introduce an artificial in-place turn.
  if (!utils::quaternionToYawChecked(current_pose.orientation,
                                     init_yaw_state(0))) {
    return false;
  }
  init_yaw_state(1) = 0.0;

  if (!std::isfinite(goal_yaw)) {
    goal_yaw = init_yaw_state(0);
  }
  const double yaw_err = std::atan2(std::sin(goal_yaw - init_yaw_state(0)),
                                    std::cos(goal_yaw - init_yaw_state(0)));
  goal_yaw_state(0) = init_yaw_state(0) + yaw_err;
  goal_yaw_state(1) = 0.0;

  // The ground chassis is omnidirectional, so the explicitly selected target
  // heading is authoritative. Allowing a free goal here replaces it with the
  // final position-trajectory tangent, which can make grid-scale path turns
  // produce an unnecessary high-rate yaw command.
  return yaw_opt_->optimize(init_yaw_state, goal_yaw_state, pos_traj,
                            out_yaw_traj, 5, false, false);
}

// -----------------------------------------------------------------------------
// 5) Helpers / callbacks / getters
// -----------------------------------------------------------------------------

bool MincoPlanner::validateTrajectory(const traj_opt::Trajectory &traj,
                                      const Eigen::Vector3d &expected_end_pos) {
  last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
  constexpr double kDt = 0.05;
  constexpr double kSevereScale = 1.5;

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    std::cout << YELLOW
              << "[MincoPlanner] validateTrajectory: invalid duration." << RESET
              << std::endl;
    last_validation_failure_reason_ = "OPTIMIZER_FAILED";
    return false;
  }

  const double vmax = minco_config.max_vel;
  const double amax = minco_config.max_acc;
  if (!(std::isfinite(vmax) && std::isfinite(amax) && vmax > 1e-6 &&
        amax > 1e-6)) {
    std::cout << YELLOW
              << "[MincoPlanner] validateTrajectory: invalid vmax/amax config."
              << RESET << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  const double vmax_severe = kSevereScale * vmax;
  const double amax_severe = kSevereScale * amax;

  // 1) Dynamic feasibility (severe violation gate).
  for (double t = 0.0; t <= dur; t += kDt) {
    const Eigen::Vector3d v = traj.getVel(t);
    const Eigen::Vector3d a = traj.getAcc(t);
    if (!(v.allFinite() && a.allFinite())) {
      std::cout << YELLOW
                << "[MincoPlanner] validateTrajectory: non-finite v/a." << RESET
                << std::endl;
      last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
      return false;
    }
    if (v.norm() > vmax_severe || a.norm() > amax_severe) {
      std::cout
          << YELLOW
          << "[MincoPlanner] validateTrajectory: severe dynamics violation."
          << " |v|=" << v.norm() << " (limit=" << vmax_severe << ")"
          << ", |a|=" << a.norm() << " (limit=" << amax_severe << ")" << RESET
          << std::endl;
      last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
      return false;
    }
  }

  // 2) Goal reachability.
  const Eigen::Vector3d end_pos = traj.getPos(dur);
  if (!end_pos.allFinite()) {
    std::cout << YELLOW
              << "[MincoPlanner] validateTrajectory: non-finite end position."
              << RESET << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  const double goal_err = (end_pos - expected_end_pos).norm();
  if (!(std::isfinite(goal_err) && goal_err <= traj_goal_tolerance_)) {
    std::cout << YELLOW
              << "[MincoPlanner] validateTrajectory: goal not reached. err="
              << goal_err << " tol=" << traj_goal_tolerance_ << RESET
              << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  // 3) Collision safety.
  if (!checkCollision(traj)) {
    last_validation_failure_reason_ = "COLLISION";
    return false;
  }

  return true;
}

bool MincoPlanner::checkCollision() {
  traj_opt::Trajectory position_snapshot;
  traj_opt::Trajectory yaw_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_last_traj_) {
      return true;
    }
    if (!has_last_yaw_traj_) {
      return false;
    }
    position_snapshot = last_traj_;
    yaw_snapshot = last_yaw_traj_;
  }

  return checkExecutableTrajectory(position_snapshot, yaw_snapshot);
}

bool MincoPlanner::checkCollision(const traj_opt::Trajectory &traj) {
  if (!safety_checker_) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return false;
  }

  const double check_end = std::min(dur, safety_check_horizon_);
  if (check_end >= dur - 1.0e-6) {
    return safety_checker_->checkTrajectory(traj);
  }

  return safety_checker_->checkTrajectoryFromTime(traj, 0.0, check_end);
}

bool MincoPlanner::checkCollision(const traj_opt::Trajectory &position_traj,
                                  const traj_opt::Trajectory &yaw_traj) {
  if (!safety_checker_ || position_traj.empty() || yaw_traj.empty()) {
    return false;
  }

  const double position_duration = position_traj.getTotalDuration();
  const double yaw_duration = yaw_traj.getTotalDuration();
  if (!(std::isfinite(position_duration) && position_duration > 1.0e-6) ||
      !(std::isfinite(yaw_duration) && yaw_duration + 1.0e-6 >= position_duration)) {
    return false;
  }

  const double check_end = std::min(position_duration, safety_check_horizon_);
  if (check_end >= position_duration - 1.0e-6) {
    return safety_checker_->checkTrajectory(position_traj, yaw_traj);
  }

  return safety_checker_->checkTrajectoryFromTime(position_traj, yaw_traj, 0.0,
                                                  check_end);
}

bool MincoPlanner::checkExecutableTrajectory(
    const traj_opt::Trajectory &position_traj,
    const traj_opt::Trajectory &yaw_traj) {
  geometry_msgs::msg::PoseStamped actual_pose;
  return getRobotPose(actual_pose) &&
         checkExecutableTrajectory(position_traj, yaw_traj, actual_pose);
}

bool MincoPlanner::checkExecutableTrajectory(
    const traj_opt::Trajectory &position_traj,
    const traj_opt::Trajectory &yaw_traj,
    const geometry_msgs::msg::PoseStamped &actual_pose) {
  if (!safety_checker_ || yaw_traj.empty()) {
    return false;
  }

  const Eigen::Vector3d actual_position(actual_pose.pose.position.x,
                                        actual_pose.pose.position.y,
                                        actual_pose.pose.position.z);
  double actual_yaw = std::numeric_limits<double>::quiet_NaN();
  double start_time = std::numeric_limits<double>::quiet_NaN();
  if (!utils::quaternionToYawChecked(actual_pose.pose.orientation,
                                     actual_yaw) ||
      !safety_checker_->computeSpatialCheckStartTime(
          position_traj, actual_position, start_time)) {
    return false;
  }

  const double yaw_duration = yaw_traj.getTotalDuration();
  if (!std::isfinite(yaw_duration) ||
      yaw_duration + 1e-6 < position_traj.getTotalDuration()) {
    return false;
  }
  const Eigen::Vector3d join_position = position_traj.getPos(start_time);
  const Eigen::Vector3d join_yaw_state =
      yaw_traj.getPos(std::min(start_time, yaw_duration));
  if (!join_position.allFinite() || !join_yaw_state.allFinite() ||
      !safety_checker_->checkSweptFootprint(
          actual_position, actual_yaw, join_position, join_yaw_state.x())) {
    return false;
  }

  return checkCollisionFromTime(position_traj, yaw_traj, start_time);
}

bool MincoPlanner::checkCollisionFromTime(
    const traj_opt::Trajectory &position_traj,
    const traj_opt::Trajectory &yaw_traj, double start_time) {
  if (!safety_checker_ || position_traj.empty() || yaw_traj.empty()) {
    return false;
  }

  const double position_duration = position_traj.getTotalDuration();
  const double yaw_duration = yaw_traj.getTotalDuration();
  if (!(std::isfinite(position_duration) && position_duration > 1.0e-6) ||
      !(std::isfinite(yaw_duration) && yaw_duration + 1.0e-6 >= position_duration) ||
      !std::isfinite(start_time)) {
    return false;
  }

  const double first_time = std::clamp(start_time, 0.0, position_duration);
  const double check_end =
      std::min(position_duration, first_time + safety_check_horizon_);
  if (check_end <= first_time + 1.0e-6) {
    return true;
  }

  if (first_time <= 1.0e-6 && check_end >= position_duration - 1.0e-6) {
    return safety_checker_->checkTrajectory(position_traj, yaw_traj);
  }

  return safety_checker_->checkTrajectoryFromTime(
      position_traj, yaw_traj, first_time, check_end);
}

bool MincoPlanner::evaluateCachedTrajectorySafety(
    const geometry_msgs::msg::PoseStamped *fallback_stop_pose) {
  traj_opt::Trajectory position_snapshot;
  traj_opt::Trajectory yaw_snapshot;
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_last_traj_ ||
        !planning_session_.accepts(last_trajectory_session_)) {
      return true;
    }
    if (emergency_stop_latched_) {
      is_traj_safe_.store(false);
      return false;
    }
    position_snapshot = last_traj_;
    if (has_last_yaw_traj_) {
      yaw_snapshot = last_yaw_traj_;
    }
    generation = trajectory_generation_;
  }

  geometry_msgs::msg::PoseStamped actual_pose;
  // ReplanLocal can finish after the pose passed by its caller is already
  // stale. Only a pose looked up at safety-check time may define the current
  // footprint and progress; the caller's pose is only a BLOCK-pose fallback.
  const bool has_actual_pose = getRobotPose(actual_pose);

  const double elapsed = nowSeconds() - position_snapshot.start_WT;
  const bool safe =
      has_actual_pose &&
      checkExecutableTrajectory(position_snapshot, yaw_snapshot, actual_pose);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != trajectory_generation_ ||
        !planning_session_.accepts(last_trajectory_session_)) {
      return true;
    }
    is_traj_safe_.store(safe);
  }
  if (safe) {
    return true;
  }

  geometry_msgs::msg::PoseStamped stop_pose;
  if (has_actual_pose) {
    stop_pose = actual_pose;
  } else if (fallback_stop_pose) {
    stop_pose = *fallback_stop_pose;
  } else {
    stop_pose.header.frame_id = output_frame_;
    stop_pose.header.stamp = rosNow();
    double stop_time = 0.0;
    if (!position_snapshot.empty()) {
      const double duration = position_snapshot.getTotalDuration();
      if (std::isfinite(duration) && duration > 0.0 && std::isfinite(elapsed)) {
        stop_time = std::clamp(elapsed, 0.0, duration);
      }
      const Eigen::Vector3d pos = position_snapshot.getPos(stop_time);
      stop_pose.pose.position.x = pos.x();
      stop_pose.pose.position.y = pos.y();
      stop_pose.pose.position.z = pos.z();
    }
    const double yaw_duration =
        yaw_snapshot.empty() ? 0.0 : yaw_snapshot.getTotalDuration();
    const double yaw =
        (!yaw_snapshot.empty() && std::isfinite(yaw_duration) &&
         yaw_duration >= 0.0)
            ? yaw_snapshot.getPos(std::min(stop_time, yaw_duration)).x()
            : 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::isfinite(yaw) ? yaw : 0.0);
    stop_pose.pose.orientation = tf2::toMsg(q);
  }

  publishEmergencyStopImpl(stop_pose, generation, true);
  return false;
}

bool MincoPlanner::republishSafeCachedTrajectory(
    const geometry_msgs::msg::PoseStamped &fallback_stop_pose,
    uint64_t expected_session, const char *rejection_reason) {
  // This also obtains a fresh physical pose, validates the complete remaining
  // position/yaw footprint against the current ROG snapshot, and publishes a
  // BLOCK if that validation fails.
  if (!evaluateCachedTrajectorySafety(&fallback_stop_pose)) {
    return false;
  }

  traj_opt::Trajectory position_snapshot;
  traj_opt::Trajectory yaw_snapshot;
  uint64_t generation = 0U;
  double elapsed = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_last_traj_ || !has_last_yaw_traj_ ||
        last_trajectory_session_ != expected_session ||
        !planning_session_.accepts(expected_session) ||
        emergency_stop_latched_) {
      return false;
    }
    position_snapshot = last_traj_;
    yaw_snapshot = last_yaw_traj_;
    generation = trajectory_generation_;
    elapsed = nowSeconds() - last_traj_.start_WT;
  }

  constexpr double command_step = 0.05;
  const double position_duration = position_snapshot.getTotalDuration();
  const double yaw_duration = yaw_snapshot.getTotalDuration();
  if (!std::isfinite(elapsed) || elapsed < 0.0 ||
      !std::isfinite(position_duration) || !std::isfinite(yaw_duration) ||
      position_duration <= elapsed + 2.0 * command_step ||
      yaw_duration <= elapsed + 2.0 * command_step) {
    return false;
  }

  const std::string_view reason = rejection_reason ? rejection_reason : "UNKNOWN";
  const double remaining_duration = position_duration - elapsed;
  if (!cached_trajectory_policy::reuseDurationAllowed(
          reason, remaining_duration,
          collision_cache_reuse_max_duration_)) {
    geometry_msgs::msg::PoseStamped stop_pose = fallback_stop_pose;
    (void)getRobotPose(stop_pose);
    publishEmergencyStopImpl(stop_pose, generation, true);
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[MincoPlanner] New trajectory rejected (%s) with %.2f s cached "
        "motion remaining; collision-cache limit is %.2f s, so BLOCK was "
        "published instead of extending the old trajectory.",
        reason.data(), remaining_duration,
        collision_cache_reuse_max_duration_);
    return false;
  }

  traj_opt::Trajectory remaining_position;
  traj_opt::Trajectory remaining_yaw;
  if (!position_snapshot.getPartialTrajectoryByTime(elapsed, position_duration,
                                                    remaining_position) ||
      !yaw_snapshot.getPartialTrajectoryByTime(elapsed, yaw_duration,
                                               remaining_yaw) ||
      remaining_position.empty() || remaining_yaw.empty()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped actual_pose;
  if (!getRobotPose(actual_pose) ||
      !checkExecutableTrajectory(remaining_position, remaining_yaw,
                                 actual_pose)) {
    return false;
  }

  bool published = false;
  {
    std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
    (void)expirePlanningRequestLeaseLocked(PlanningRequestLease::Clock::now());
    if (trajectory_generation_ != generation ||
        last_trajectory_session_ != expected_session ||
        !planning_session_.accepts(expected_session) ||
        emergency_stop_latched_) {
      return false;
    }

    std_msgs::msg::Header header_msg;
    header_msg.frame_id = output_frame_;
    const rclcpp::Time trajectory_stamp = rosNow();
    header_msg.stamp = trajectory_stamp;
    remaining_position.start_WT = trajectory_stamp.seconds();
    remaining_yaw.start_WT = trajectory_stamp.seconds();
    last_traj_ = remaining_position;
    last_yaw_traj_ = remaining_yaw;
    ++trajectory_generation_;
    is_traj_safe_.store(true);
    const int steps =
        std::max(2, static_cast<int>(std::ceil(
                        remaining_position.getTotalDuration() / command_step)) +
                        1);
    utils::publishOptimizedTrajectory(
        remaining_position, remaining_yaw, opt_path_pub_, opt_trajectory_id_,
        header_msg, planning_stamp_, steps, command_step);
    published = true;
  }

  if (published) {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[MincoPlanner] New trajectory rejected (%s); republished %.2f s of "
        "the current trajectory after fresh ROG footprint validation.",
        rejection_reason ? rejection_reason : "UNKNOWN",
        remaining_position.getTotalDuration());
  }
  return published;
}

bool MincoPlanner::ensureTrajectorySafe(
    const geometry_msgs::msg::PoseStamped &current_pose) {
  return evaluateCachedTrajectorySafety(&current_pose);
}

void MincoPlanner::safetyTimerCallback() {
  std::lock_guard<std::mutex> execution_lock(safety_execution_mutex_);
  if (!isLifecycleActive()) {
    return;
  }
  if (!evaluateCachedTrajectorySafety(nullptr)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "[MincoPlanner] Unsafe position/yaw trajectory; "
                         "emergency stop published.");
  }
}

void MincoPlanner::publishEmergencyStop(
    const geometry_msgs::msg::PoseStamped &current_pose) {
  publishEmergencyStopImpl(current_pose, 0U, false);
}

void MincoPlanner::publishEmergencyStopImpl(
    const geometry_msgs::msg::PoseStamped &current_pose,
    uint64_t expected_generation, bool require_generation_match) {
  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;

  Eigen::Matrix3d start_state;
  prepareColdStart(current_pose.pose, start_state,
                   std::vector<Eigen::Vector3d>{});
  double current_yaw = 0.0;
  if (!utils::quaternionToYawChecked(current_pose.pose.orientation,
                                     current_yaw)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "[MincoPlanner] Emergency-stop pose has invalid "
                         "physical orientation; publishing zero yaw.");
    current_yaw = 0.0;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (require_generation_match &&
      expected_generation != trajectory_generation_) {
    return;
  }
  if (emergency_stop_latched_) {
    return;
  }
  if (has_last_traj_) {
    const double t_dur = nowSeconds() - last_traj_.start_WT;
    const double total = last_traj_.getTotalDuration();
    if (std::isfinite(t_dur) && std::isfinite(total) && t_dur >= 0.0 &&
        t_dur <= total) {
      start_state.col(1) = last_traj_.getVel(t_dur);
      start_state.col(2) = last_traj_.getAcc(t_dur);
    }
  }

  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);
  emergency_stop_latched_ = true;
  is_traj_safe_.store(false);
  header_msg.stamp = rosNow();
  utils::publishBackupTrajectory(backup_traj, opt_path_pub_, backup_path_pub_,
                                 opt_trajectory_id_, header_msg,
                                 planning_stamp_, 20, 0.1, current_yaw);
}

traj_opt::Trajectory
MincoPlanner::generateBackupTraj(const Eigen::Matrix3d &start_state) {
  std::lock_guard<std::mutex> backup_lock(backup_mutex_);
  auto make_stop_traj = [&start_state]() -> traj_opt::Trajectory {
    traj_opt::Trajectory stop_traj;
    const Eigen::Vector3d p = start_state.col(0);

    Eigen::MatrixXd cMat(3, 6);
    cMat.setZero();
    cMat.col(5) = p;

    // Two very short constant pieces ("2 points" semantics).
    stop_traj.emplace_back(0.2, cMat);
    stop_traj.emplace_back(0.2, cMat);
    return stop_traj;
  };

  if (!corridor_gen_ || !backup_opt_) {
    std::cout << RED << "[MincoPlanner] Backup optimizer not initialized!"
              << RESET << std::endl;
    return make_stop_traj();
  }

  // Step 1: Generate SFC (safe box).
  auto safe_poly = corridor_gen_->generateSafeBox(start_state.col(0), 1.0);

  // Step 2: Setup backup optimizer.
  backup_opt_->setInitState(start_state);
  backup_opt_->setStopConstraints();
  backup_opt_->setPolygons({safe_poly});

  // Step 3: Optimize.
  traj_opt::Trajectory backup_traj;
  bool success = backup_opt_->optimize(backup_traj);

  // Step 4: Return.
  if (success) {
    return backup_traj;
  }

  std::cout << RED
            << "[MincoPlanner] Backup trajectory optimization failed, fallback "
               "to stop."
            << RESET << std::endl;
  return make_stop_traj();
}

bool MincoPlanner::consumePendingGoal(geometry_msgs::msg::PoseStamped &goal_out,
                                      uint64_t &session_out) {
  std::lock_guard<std::mutex> session_lock(mutex_);
  std::lock_guard<std::mutex> goal_lock(goal_mutex_);
  if (!has_pending_goal_ || !planning_session_.accepts(pending_goal_session_)) {
    has_pending_goal_ = false;
    pending_goal_session_ = 0U;
    return false;
  }
  goal_out = pending_goal_;
  session_out = pending_goal_session_;
  has_pending_goal_ = false;
  pending_goal_session_ = 0U;
  return true;
}

void MincoPlanner::cancelGoal() {
  (void)beginPlanningSession();
  // Do not mutate MincoFsm from the planner action callback while its timer may
  // be optimizing. The generation is invalid immediately; the serialized FSM
  // callback reconciles the stale token on its next tick.
}

void MincoPlanner::completeGoal(uint64_t expected_session) {
  std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
  if (!planning_session_.accepts(expected_session)) {
    return;
  }

  (void)planning_session_.beginSession();
  request_lease_.reset();
  advancePlanningStampLocked();
  invalidateTrajectoryLocked();
  has_active_goal_ = false;
  active_goal_session_ = 0U;
  has_pending_goal_ = false;
  pending_goal_session_ = 0U;
  latest_global_path_.clear();
  latest_global_path_session_ = 0U;
  if (visualizer_) {
    visualizer_->updateGlobalPath(nav_msgs::msg::Path{});
  }
  publishBlockCommandLocked();
}

double MincoPlanner::nowSeconds() const { return rosNow().seconds(); }

rclcpp::Time MincoPlanner::rosNow() const { return clock_->now(); }

double MincoPlanner::getTrajectoryRemainTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_traj_) {
    return 0.0;
  }
  double passed_time = nowSeconds() - last_traj_.start_WT;
  return std::max(0.0, last_traj_.getTotalDuration() - passed_time);
}

bool MincoPlanner::getRobotPose(geometry_msgs::msg::PoseStamped &pose) const {
  const bool direct_odom_pose =
      mode_context_ && mode_context_->directOdomPose();
  geometry_msgs::msg::PoseStamped physical_robot_pose;
  if (!getPhysicalRobotPoseInPlanningFrame(physical_robot_pose)) {
    return false;
  }

  if (!direct_odom_pose && costmap_ros_) {
    if (costmap_ros_->getRobotPose(pose)) {
      // The legacy Nav2 costmaps intentionally track gimbal_yaw_fake, whose
      // yaw is cancelled for world-frame velocity control. MINCO footprint
      // checks must use the physical chassis yaw while retaining the costmap's
      // control-point position.
      pose.pose.orientation = physical_robot_pose.pose.orientation;
      pose.header.frame_id = planning_frame_;
      return true;
    }
  }

  pose = physical_robot_pose;
  return true;
}

bool MincoPlanner::getPhysicalRobotPoseInPlanningFrame(
    geometry_msgs::msg::PoseStamped &pose) const {
  if (!tf_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                         "[MincoPlanner] Cannot look up physical robot pose %s "
                         "-> %s: TF buffer is null.",
                         physical_base_frame_.c_str(), planning_frame_.c_str());
    return false;
  }

  try {
    const auto transform = tf_->lookupTransform(
        planning_frame_, physical_base_frame_, tf2::TimePointZero);
    pose.header = transform.header;
    pose.header.frame_id = planning_frame_;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    double yaw = std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(pose.pose.position.x) ||
        !std::isfinite(pose.pose.position.y) ||
        !std::isfinite(pose.pose.position.z) ||
        !utils::quaternionToYawChecked(pose.pose.orientation, yaw)) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                           "[MincoPlanner] Physical robot TF %s -> %s contains "
                           "a non-finite pose.",
                           physical_base_frame_.c_str(),
                           planning_frame_.c_str());
      return false;
    }
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "[MincoPlanner] Failed to look up physical robot pose %s -> %s: %s",
        physical_base_frame_.c_str(), planning_frame_.c_str(), ex.what());
    return false;
  }
}

bool MincoPlanner::checkGoalReached(
    const geometry_msgs::msg::PoseStamped &current_pose) {
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (latest_global_path_.empty()) {
    return false;
  }

  const auto &goal = latest_global_path_.back().pose.position;
  const double dx = current_pose.pose.position.x - goal.x;
  const double dy = current_pose.pose.position.y - goal.y;
  const double dist = std::hypot(dx, dy);
  return std::isfinite(dist) && dist <= traj_goal_tolerance_;
}

Eigen::Vector3d MincoPlanner::getCurrentSpeed() const {
  std::lock_guard<std::mutex> lk(odom_mutex_);
  if (has_latest_odom_) {
    const auto &twist = latest_odom_.twist.twist;
    const double yaw =
        utils::quaternionToYaw(latest_odom_.pose.pose.orientation);

    double vx_global = 0.0;
    double vy_global = 0.0;
    double omega_global = 0.0;
    utils::compensateLeverArm(twist.linear.x, twist.linear.y, twist.angular.z,
                              yaw, lidar_offset_x_, lidar_offset_y_, vx_global,
                              vy_global, omega_global);

    // std::cout << "[MincoPlanner] Lever-arm compensation: raw_v=(" <<
    // twist.linear.x << ", "
    //           << twist.linear.y << ") wz=" << twist.angular.z << " yaw=" <<
    //           yaw << " -> v=("
    //           << vx_global << ", " << vy_global << ")" << std::endl;

    // This stack plans a 2.5D ground path but optimizes a z=0 polynomial. A
    // vertical/body-frame slope component would make the boundary state
    // inconsistent with that polynomial and can invalidate continuous dynamic
    // feasibility before retiming starts.
    return Eigen::Vector3d(vx_global, vy_global, 0.0);
  }
  return Eigen::Vector3d::Zero();
}

double MincoPlanner::getCurrentYawFromOdom() const {
  std::lock_guard<std::mutex> lk(odom_mutex_);
  if (!has_latest_odom_) {
    return 0.0;
  }
  return utils::quaternionToYaw(latest_odom_.pose.pose.orientation);
}

bool MincoPlanner::isTrajectoryTimeExpired(double now_s) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_traj_) {
    return true;
  }
  const double end_s = last_traj_.start_WT + last_traj_.getTotalDuration();
  return now_s > end_s;
}

double MincoPlanner::getEsdfDistance(const Eigen::Vector3d &pos) const {
  return safety_checker_ ? safety_checker_->getDistance(pos)
                         : std::numeric_limits<double>::quiet_NaN();
}

bool MincoPlanner::publishEscapeCommand(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const Eigen::Vector2d &escape_vel, uint64_t expected_session) {
  uint64_t expected_generation = 0U;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!planning_session_.accepts(expected_session)) {
      return false;
    }
    if (emergency_stop_latched_) {
      publishBlockCommandLocked();
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                           "[MincoPlanner] Escape command suppressed while "
                           "emergency stop is latched.");
      return false;
    }
    expected_generation = trajectory_generation_;
  }

  auto reject_escape = [this, expected_session,
                        expected_generation](const char *reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!planning_session_.accepts(expected_session) ||
        trajectory_generation_ != expected_generation) {
      return false;
    }
    last_traj_.clear();
    last_yaw_traj_.clear();
    has_last_traj_ = false;
    has_last_yaw_traj_ = false;
    last_trajectory_session_ = 0U;
    ++trajectory_generation_;
    emergency_stop_latched_ = true;
    is_traj_safe_.store(false);
    publishBlockCommandLocked();
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[MincoPlanner] Escape trajectory rejected (%s); BLOCK published.",
        reason);
    return false;
  };

  if (current_pose.header.frame_id != planning_frame_) {
    return reject_escape("POSE_FRAME_MISMATCH");
  }

  double current_yaw = std::numeric_limits<double>::quiet_NaN();
  if (!utils::quaternionToYawChecked(current_pose.pose.orientation,
                                     current_yaw)) {
    return reject_escape("INVALID_CURRENT_YAW");
  }

  constexpr double escape_duration = 0.5;
  constexpr double command_step = 0.05;
  traj_opt::Trajectory escape_traj;
  traj_opt::Trajectory yaw_traj;
  if (!utils::makeEscapeTrajectories(current_pose, escape_vel, current_yaw,
                                     escape_duration, escape_traj, yaw_traj)) {
    return reject_escape("INVALID_ESCAPE_TRAJECTORY");
  }

  // This is the same full body + yaw + ROG snapshot freshness gate used for
  // optimized trajectories. It samples the current footprint, prediction, and
  // endpoint.
  if (!checkCollision(escape_traj, yaw_traj)) {
    return reject_escape("TRAJECTORY_SAFETY_CHECK_FAILED");
  }

  bool published = false;
  {
    std::scoped_lock lock(mutex_, goal_mutex_, path_mutex_);
    (void)expirePlanningRequestLeaseLocked(PlanningRequestLease::Clock::now());
    if (!planning_session_.accepts(expected_session) ||
        trajectory_generation_ != expected_generation ||
        emergency_stop_latched_) {
      return false;
    }

    std_msgs::msg::Header header_msg;
    header_msg.frame_id = output_frame_;
    const rclcpp::Time trajectory_stamp = rosNow();
    header_msg.stamp = trajectory_stamp;
    escape_traj.start_WT = trajectory_stamp.seconds();
    yaw_traj.start_WT = escape_traj.start_WT;
    last_traj_ = escape_traj;
    last_yaw_traj_ = yaw_traj;
    has_last_traj_ = true;
    has_last_yaw_traj_ = true;
    last_trajectory_session_ = expected_session;
    ++trajectory_generation_;
    emergency_stop_latched_ = false;
    is_traj_safe_.store(true);
    const int steps =
        static_cast<int>(std::ceil(escape_duration / command_step)) + 1;
    utils::publishOptimizedTrajectory(escape_traj, yaw_traj, opt_path_pub_,
                                      opt_trajectory_id_, header_msg,
                                      planning_stamp_, steps, command_step);
    published = true;
  }

  if (published && visualizer_) {
    visualizer_->publishRecoveryDebug(current_pose, escape_vel,
                                      escape_duration);
  }
  return published;
}

void MincoPlanner::clearRecoveryDebugVisualization() {
  if (visualizer_) {
    visualizer_->clearRecoveryDebug();
  }
}

} // namespace minco_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(minco_planner::MincoPlanner, nav2_core::GlobalPlanner)
