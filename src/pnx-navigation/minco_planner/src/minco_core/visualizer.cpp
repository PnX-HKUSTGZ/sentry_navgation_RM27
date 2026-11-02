#include "minco_core/visualizer.hpp"

// C++ standard library
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>

#include "nav2_costmap_2d/costmap_layer.hpp"

namespace minco_planner {

void Visualizer::configure(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & global_frame,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = parent;
  global_frame_ = global_frame;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  if (!node) {
    return;
  }

  // --- Visualization publishers ---------------------------------------------

  // Backup
  backup_path_vis_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("/backup_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)));

  // Optimized
  opt_path_vis_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("/opt_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)));

  candidate_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/minco_candidate_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // A* guide path
  astar_path_vis_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/astar_path_vis", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // Recovery debug
  recover_path_vis_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("/recover_path", rclcpp::QoS(rclcpp::KeepLast(1)));
  recover_goal_vis_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "/recover_goal", rclcpp::QoS(rclcpp::KeepLast(1)));

  // Markers
  control_points_vis_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "/minco_control_points_vis", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // This is the final master costmap after all plugins are merged. Publishing
  // soft cells as XYZ+intensity bypasses RViz's indexed Map texture shader,
  // which is unreliable on some integrated-GPU OpenGL drivers.
  global_costmap_soft_costs_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/minco/global_costmap_soft_costs",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  dynamic_costmap_inflation_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/minco/dynamic_costmap_inflation",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  // 15Hz visualization timer
  visual_callback_group_ =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  visual_timer_ = node->create_timer(
    std::chrono::milliseconds(66),
    std::bind(&Visualizer::visualTimerCallback, this),
    visual_callback_group_);
}

void Visualizer::cleanup()
{
  visual_timer_.reset();
  visual_callback_group_.reset();

  auto node = node_.lock();
  if (node && control_points_vis_pub_) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = node->now();
    marker.header.frame_id = global_frame_;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.ns = "minco_local_end";
    marker.id = 0;
    control_points_vis_pub_->publish(marker);
    marker.id = 1;
    control_points_vis_pub_->publish(marker);
    marker.ns = "minco_candidate_status";
    marker.id = 0;
    control_points_vis_pub_->publish(marker);
  }
  if (node && candidate_path_vis_pub_) {
    nav_msgs::msg::Path path;
    path.header.stamp = node->now();
    path.header.frame_id = global_frame_;
    candidate_path_vis_pub_->publish(path);
  }
  if (node && astar_path_vis_pub_) {
    nav_msgs::msg::Path path;
    path.header.stamp = node->now();
    path.header.frame_id = global_frame_;
    astar_path_vis_pub_->publish(path);
  }
  if (node && global_costmap_soft_costs_pub_) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = node->now();
    const auto costmap_ros = costmap_ros_.lock();
    cloud.header.frame_id = costmap_ros ? costmap_ros->getGlobalFrameID() : global_frame_;
    global_costmap_soft_costs_pub_->publish(cloud);
    if (dynamic_costmap_inflation_pub_) {
      dynamic_costmap_inflation_pub_->publish(cloud);
    }
  }

  backup_path_vis_pub_.reset();
  opt_path_vis_pub_.reset();
  candidate_path_vis_pub_.reset();
  astar_path_vis_pub_.reset();
  recover_path_vis_pub_.reset();
  recover_goal_vis_pub_.reset();
  control_points_vis_pub_.reset();
  global_costmap_soft_costs_pub_.reset();
  dynamic_costmap_inflation_pub_.reset();
  costmap_ros_.reset();

  std::lock_guard<std::mutex> lock(vis_mutex_);
  vis_control_points_.clear();
  vis_astar_path_ = nav_msgs::msg::Path();
  astar_path_update_pending_ = false;
  vis_opt_time_ = -1.0;
  has_vis_opt_traj_ = false;
  has_vis_backup_traj_ = false;
  has_vis_local_end_ = false;
  vis_local_end_is_goal_ = false;
  has_vis_candidate_traj_ = false;
  vis_candidate_valid_ = false;
  vis_candidate_reason_.clear();
  vis_candidate_opt_time_ = -1.0;
}

void Visualizer::publishRecoveryDebug(const geometry_msgs::msg::PoseStamped & current_pose,
  const Eigen::Vector2d & escape_vel,
  double preview_sec)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  if (!recover_goal_vis_pub_ || !recover_path_vis_pub_) {
    return;
  }
  if (!(std::isfinite(current_pose.pose.position.x) && std::isfinite(current_pose.pose.position.y) &&
        escape_vel.allFinite())) {
    return;
  }

  const double dt = (std::isfinite(preview_sec) && preview_sec > 0.0) ? preview_sec : 0.5;
  const double goal_x = current_pose.pose.position.x + escape_vel.x() * dt;
  const double goal_y = current_pose.pose.position.y + escape_vel.y() * dt;

  visualization_msgs::msg::Marker goal_mk;
  goal_mk.header.stamp = node->now();
  goal_mk.header.frame_id = global_frame_;
  goal_mk.ns = "recover_goal";
  goal_mk.id = 0;
  goal_mk.type = visualization_msgs::msg::Marker::SPHERE;
  goal_mk.action = visualization_msgs::msg::Marker::ADD;
  goal_mk.pose.position.x = goal_x;
  goal_mk.pose.position.y = goal_y;
  goal_mk.pose.position.z = 0.05;
  goal_mk.pose.orientation.w = 1.0;
  goal_mk.scale.x = 0.20;
  goal_mk.scale.y = 0.20;
  goal_mk.scale.z = 0.20;
  goal_mk.color.r = 1.0f;
  goal_mk.color.g = 0.2f;
  goal_mk.color.b = 0.2f;
  goal_mk.color.a = 1.0f;
  recover_goal_vis_pub_->publish(goal_mk);

  nav_msgs::msg::Path path_msg;
  path_msg.header = goal_mk.header;
  path_msg.poses.resize(2);
  path_msg.poses[0] = current_pose;
  path_msg.poses[0].header = path_msg.header;
  path_msg.poses[1].header = path_msg.header;
  path_msg.poses[1].pose.position.x = goal_x;
  path_msg.poses[1].pose.position.y = goal_y;
  path_msg.poses[1].pose.position.z = 0.0;
  path_msg.poses[1].pose.orientation.w = 1.0;
  recover_path_vis_pub_->publish(path_msg);
}

void Visualizer::clearRecoveryDebug()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  if (!recover_goal_vis_pub_ || !recover_path_vis_pub_) {
    return;
  }

  visualization_msgs::msg::Marker goal_mk;
  goal_mk.header.stamp = node->now();
  goal_mk.header.frame_id = global_frame_;
  goal_mk.ns = "recover_goal";
  goal_mk.id = 0;
  goal_mk.action = visualization_msgs::msg::Marker::DELETE;
  recover_goal_vis_pub_->publish(goal_mk);

  nav_msgs::msg::Path path_msg;
  path_msg.header = goal_mk.header;
  recover_path_vis_pub_->publish(path_msg);
}

void Visualizer::updateLocalEndPoint(const Eigen::Vector3d & point, bool local_end_is_goal)
{
  std::lock_guard<std::mutex> lock(vis_mutex_);
  vis_local_end_ = point;
  has_vis_local_end_ = point.allFinite();
  vis_local_end_is_goal_ = local_end_is_goal;
}

void Visualizer::clearLocalEndPoint()
{
  std::lock_guard<std::mutex> lock(vis_mutex_);
  has_vis_local_end_ = false;
  vis_local_end_is_goal_ = false;
}

void Visualizer::updateCandidateTrajectory(const traj_opt::Trajectory & trajectory,
  double opt_time,
  bool validation_ok,
  const std::string & validation_reason)
{
  std::lock_guard<std::mutex> lock(vis_mutex_);
  vis_candidate_traj_ = trajectory;
  has_vis_candidate_traj_ = trajectory.getTotalDuration() > 1e-3;
  vis_candidate_valid_ = validation_ok;
  vis_candidate_reason_ = validation_reason;
  vis_candidate_opt_time_ = opt_time;
}

void Visualizer::clearCandidateTrajectory(const std::string & failure_reason)
{
  std::lock_guard<std::mutex> lock(vis_mutex_);
  has_vis_candidate_traj_ = false;
  vis_candidate_valid_ = false;
  vis_candidate_reason_ = failure_reason;
  vis_candidate_opt_time_ = -1.0;
}

void Visualizer::update(const std::vector<Eigen::Vector3d> & control_points,
  const traj_opt::Trajectory & backup_traj,
  const traj_opt::Trajectory & opt_traj,
  double opt_time_seconds,
  const nav_msgs::msg::Path & astar_path)
{
  std::lock_guard<std::mutex> lock(vis_mutex_);
  vis_control_points_ = control_points;

  vis_backup_traj_ = backup_traj;
  has_vis_backup_traj_ = (backup_traj.getTotalDuration() > 1e-3);

  vis_opt_traj_ = opt_traj;
  vis_opt_time_ = opt_time_seconds;
  has_vis_opt_traj_ = (opt_traj.getTotalDuration() > 1e-3);

  vis_astar_path_ = astar_path;
  astar_path_update_pending_ = true;
}

void Visualizer::updateGlobalPath(const nav_msgs::msg::Path & astar_path)
{
  std::lock_guard<std::mutex> lock(vis_mutex_);
  vis_astar_path_ = astar_path;
  astar_path_update_pending_ = true;
}

void Visualizer::visualTimerCallback()
{
  std_msgs::msg::Header header;
  auto node = node_.lock();
  if (node) {
    header.stamp = node->now();
    header.frame_id = global_frame_;
  } else {
    return;
  }

  publishGlobalCostmapSoftCosts(header);

  std::lock_guard<std::mutex> lock(vis_mutex_);

  // 1. A* Path
  if (astar_path_vis_pub_ &&
      (astar_path_update_pending_ || !vis_astar_path_.poses.empty())) {
    vis_astar_path_.header = header;
    for (auto & p : vis_astar_path_.poses) {
      p.header = header;
    }
    astar_path_vis_pub_->publish(vis_astar_path_);
    astar_path_update_pending_ = false;
  }

  // 2. Project A* sampled control points to optimized trajectory + straight line connections
  if (control_points_vis_pub_ && has_vis_opt_traj_ && vis_opt_traj_.getTotalDuration() > 1e-3 &&
      !vis_control_points_.empty()) {
    const double t_step = 0.02;
    const double total_duration = vis_opt_traj_.getTotalDuration();
    const int sample_steps = static_cast<int>(std::ceil(total_duration / t_step)) + 1;

    std::vector<Eigen::Vector3d> traj_samples;
    traj_samples.reserve(static_cast<size_t>(sample_steps));
    for (int i = 0; i < sample_steps; ++i) {
      double t = i * t_step;
      if (t > total_duration) {
        t = total_duration;
      }
      traj_samples.push_back(vis_opt_traj_.getPos(t));
    }

    std::vector<geometry_msgs::msg::Point> vis_waypoints;
    vis_waypoints.reserve(vis_control_points_.size());

    size_t search_start = 0;
    for (const auto & cp : vis_control_points_) {
      if (search_start >= traj_samples.size()) {
        break;
      }

      size_t best_idx = search_start;
      double best_dist_sq = std::numeric_limits<double>::max();
      for (size_t i = search_start; i < traj_samples.size(); ++i) {
        const double dx = traj_samples[i].x() - cp.x();
        const double dy = traj_samples[i].y() - cp.y();
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_idx = i;
        }
      }

      geometry_msgs::msg::Point pt;
      pt.x = traj_samples[best_idx].x();
      pt.y = traj_samples[best_idx].y();
      pt.z = 0.05;
      vis_waypoints.push_back(pt);

      search_start = best_idx;
    }

    visualization_msgs::msg::Marker points_mk;
    points_mk.header = header;
    points_mk.ns = "minco_opt_waypoints";
    points_mk.id = 0;
    points_mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_mk.action = visualization_msgs::msg::Marker::ADD;
    points_mk.pose.orientation.w = 1.0;
    points_mk.scale.x = 0.25;
    points_mk.scale.y = 0.25;
    points_mk.scale.z = 0.25;
    points_mk.color.r = 1.0f;
    points_mk.color.g = 0.55f;
    points_mk.color.b = 0.0f;
    points_mk.color.a = 1.0f;
    if (!vis_waypoints.empty()) {
      points_mk.points = vis_waypoints;
      control_points_vis_pub_->publish(points_mk);
    }

    if (vis_waypoints.size() >= 2U) {
      visualization_msgs::msg::Marker line_mk;
      line_mk.header = header;
      line_mk.ns = "minco_opt_waypoints";
      line_mk.id = 1;
      line_mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_mk.action = visualization_msgs::msg::Marker::ADD;
      line_mk.pose.orientation.w = 1.0;
      line_mk.scale.x = 0.04;
      line_mk.color.r = 1.0f;
      line_mk.color.g = 0.35f;
      line_mk.color.b = 0.0f;
      line_mk.color.a = 1.0f;
      line_mk.points = vis_waypoints;
      control_points_vis_pub_->publish(line_mk);
    }
  }

  // 3. Backup Path
  if (backup_path_vis_pub_ && has_vis_backup_traj_ && vis_backup_traj_.getTotalDuration() > 1e-3) {
    const double t_step = 0.05;
    const int steps = static_cast<int>(std::ceil(vis_backup_traj_.getTotalDuration() / t_step)) + 1;
    auto path_msg = convertTrajectoryToPath(vis_backup_traj_, header, steps, t_step);
    backup_path_vis_pub_->publish(path_msg);
  }

  // 4. Optimized Path
  if (opt_path_vis_pub_ && has_vis_opt_traj_ && vis_opt_traj_.getTotalDuration() > 1e-3) {
    const double t_step = 0.05;
    const int steps = static_cast<int>(std::ceil(vis_opt_traj_.getTotalDuration() / t_step)) + 1;
    auto path_msg = convertTrajectoryToPath(vis_opt_traj_, header, steps, t_step);
    opt_path_vis_pub_->publish(path_msg);
  }

  // 5. Opt time text
  if (control_points_vis_pub_ && has_vis_opt_traj_ && vis_opt_traj_.getTotalDuration() > 1e-3 &&
      vis_opt_time_ > 0.0) {
    visualization_msgs::msg::Marker mk;
    mk.header = header;
    mk.ns = "opt_time";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    mk.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d start_pos = vis_opt_traj_.getPos(0.0);
    mk.pose.position.x = start_pos(0);
    mk.pose.position.y = start_pos(1);
    mk.pose.position.z = 1.0;
    mk.pose.orientation.w = 1.0;
    mk.scale.z = 0.3;
    mk.color.r = 1.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0f;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << (vis_opt_time_ * 1000.0) << " ms";
    mk.text = ss.str();
    control_points_vis_pub_->publish(mk);
  }

  // 6. Local path clipping endpoint (independent of accepted trajectory state).
  if (control_points_vis_pub_) {
    visualization_msgs::msg::Marker mk;
    mk.header = header;
    mk.ns = "minco_local_end";
    mk.pose.orientation.w = 1.0;
    if (has_vis_local_end_) {
      mk.id = 0;
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = vis_local_end_.x();
      mk.pose.position.y = vis_local_end_.y();
      mk.pose.position.z = vis_local_end_.z() + 0.15;
      mk.scale.x = 0.28;
      mk.scale.y = 0.28;
      mk.scale.z = 0.28;
      mk.color.r = vis_local_end_is_goal_ ? 0.0f : 0.65f;
      mk.color.g = vis_local_end_is_goal_ ? 1.0f : 0.0f;
      mk.color.b = vis_local_end_is_goal_ ? 0.0f : 1.0f;
      mk.color.a = 1.0f;
      control_points_vis_pub_->publish(mk);

      mk.id = 1;
      mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      mk.pose.position.z = vis_local_end_.z() + 0.55;
      mk.scale.z = 0.25;
      mk.text = vis_local_end_is_goal_ ? "GLOBAL GOAL" : "LOCAL END";
      control_points_vis_pub_->publish(mk);
    } else {
      mk.action = visualization_msgs::msg::Marker::DELETE;
      mk.id = 0;
      control_points_vis_pub_->publish(mk);
      mk.id = 1;
      control_points_vis_pub_->publish(mk);
    }
  }

  // 7. Pre-validation optimizer candidate path.
  if (candidate_path_vis_pub_) {
    nav_msgs::msg::Path path_msg;
    path_msg.header = header;
    if (has_vis_candidate_traj_ && vis_candidate_traj_.getTotalDuration() > 1e-3) {
      const double t_step = 0.05;
      const int steps =
        static_cast<int>(std::ceil(vis_candidate_traj_.getTotalDuration() / t_step)) + 1;
      path_msg = convertTrajectoryToPath(vis_candidate_traj_, header, steps, t_step);
    }
    candidate_path_vis_pub_->publish(path_msg);
  }

  // 8. Candidate validation status.
  if (control_points_vis_pub_) {
    visualization_msgs::msg::Marker mk;
    mk.header = header;
    mk.ns = "minco_candidate_status";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    mk.pose.orientation.w = 1.0;
    if (has_vis_candidate_traj_ || (!vis_candidate_reason_.empty() && has_vis_local_end_)) {
      const Eigen::Vector3d anchor =
        has_vis_candidate_traj_ ? vis_candidate_traj_.getPos(0.0) : vis_local_end_;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = anchor.x();
      mk.pose.position.y = anchor.y();
      mk.pose.position.z = anchor.z() + 1.0;
      mk.scale.z = 0.3;
      mk.color.a = 1.0f;

      const std::string status = vis_candidate_valid_ ? "VALID" : vis_candidate_reason_;
      mk.text = "CANDIDATE: " + status;
      if (status == "VALID") {
        mk.color.g = 1.0f;
      } else if (status == "COLLISION") {
        mk.color.r = 1.0f;
      } else if (status == "KINEMATIC_VIOLATION") {
        mk.color.r = 1.0f;
        mk.color.g = 1.0f;
      } else {
        mk.color.r = 1.0f;
        mk.color.g = 0.5f;
      }
      control_points_vis_pub_->publish(mk);
    } else {
      mk.action = visualization_msgs::msg::Marker::DELETE;
      control_points_vis_pub_->publish(mk);
    }
  }
}

void Visualizer::publishGlobalCostmapSoftCosts(const std_msgs::msg::Header & header)
{
  if (!global_costmap_soft_costs_pub_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (last_costmap_soft_costs_publish_.time_since_epoch().count() != 0 &&
      now - last_costmap_soft_costs_publish_ < std::chrono::seconds(1)) {
    return;
  }
  last_costmap_soft_costs_publish_ = now;

  const auto costmap_ros = costmap_ros_.lock();
  auto * costmap = costmap_ros ? costmap_ros->getCostmap() : nullptr;
  if (!costmap) {
    return;
  }

  unsigned int size_x = 0U;
  unsigned int size_y = 0U;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<unsigned char> costs;
  std::vector<uint8_t> dynamic_neighborhood;
  size_t dynamic_sources = 0U;
  size_t missing_dynamic_cores = 0U;
  double inflation_radius = 0.0;
  if (dynamic_costmap_inflation_pub_ && costmap_ros->get_parameter(
      "inflation_layer.inflation_radius", inflation_radius) &&
      std::isfinite(inflation_radius) && inflation_radius > 0.0)
  {
    // Read the active Nav2 inflation radius, not a separate visualization knob.
  } else {
    inflation_radius = 0.0;
  }
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap->getMutex());
    size_x = costmap->getSizeInCellsX();
    size_y = costmap->getSizeInCellsY();
    resolution = costmap->getResolution();
    origin_x = costmap->getOriginX();
    origin_y = costmap->getOriginY();
    const size_t cell_count = static_cast<size_t>(size_x) * size_y;
    const unsigned char * charmap = costmap->getCharMap();
    if (!charmap || cell_count == 0U) {
      return;
    }
    costs.assign(charmap, charmap + cell_count);

    // The ROG layer contains only measured source cells. Sample the final
    // master inside their inflation radius, so this display never invents
    // costs that Nav2's planner did not actually receive.
    if (dynamic_costmap_inflation_pub_ && inflation_radius > 0.0) {
      auto * layered = costmap_ros->getLayeredCostmap();
      nav2_costmap_2d::CostmapLayer * rog_layer = nullptr;
      for (const auto & plugin : *layered->getPlugins()) {
        if (plugin->getName().find("rog_dynamic_obstacle_layer") != std::string::npos) {
          rog_layer = dynamic_cast<nav2_costmap_2d::CostmapLayer *>(plugin.get());
          break;
        }
      }
      if (rog_layer && rog_layer->getSizeInCellsX() == size_x &&
          rog_layer->getSizeInCellsY() == size_y &&
          std::abs(rog_layer->getOriginX() - origin_x) < 1e-6 &&
          std::abs(rog_layer->getOriginY() - origin_y) < 1e-6 &&
          std::abs(rog_layer->getResolution() - resolution) < 1e-6)
      {
        // RogDynamicObstacleLayer commits its rolling snapshot on the
        // costmap update thread.  Lock the layer while sampling it; without
        // this, the diagnostic cloud can mix a newly cleared layer with an
        // old master and falsely report missing dynamic cores.
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> rog_lock(
          *rog_layer->getMutex());
        dynamic_neighborhood.assign(cell_count, 0U);
        const auto * sources = rog_layer->getCharMap();
        const int radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution));
        for (unsigned int my = 0U; my < size_y; ++my) {
          for (unsigned int mx = 0U; mx < size_x; ++mx) {
            const size_t source_index = static_cast<size_t>(my) * size_x + mx;
            if (sources[source_index] != nav2_costmap_2d::LETHAL_OBSTACLE) {
              continue;
            }
            ++dynamic_sources;
            if (costs[source_index] != nav2_costmap_2d::LETHAL_OBSTACLE) {
              ++missing_dynamic_cores;
            }
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
              for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (std::hypot(dx, dy) * resolution > inflation_radius) {
                  continue;
                }
                const int x = static_cast<int>(mx) + dx;
                const int y = static_cast<int>(my) + dy;
                if (x >= 0 && y >= 0 && x < static_cast<int>(size_x) &&
                    y < static_cast<int>(size_y)) {
                  dynamic_neighborhood[static_cast<size_t>(y) * size_x + x] = 1U;
                }
              }
            }
          }
        }
      }
    }
  }

  const auto is_visible_cost = [](const unsigned char cost) {
    return cost > nav2_costmap_2d::FREE_SPACE &&
           cost != nav2_costmap_2d::NO_INFORMATION;
  };
  const size_t visible_cell_count = static_cast<size_t>(
    std::count_if(costs.begin(), costs.end(), is_visible_cost));

  const std::string costmap_frame = costmap_ros->getGlobalFrameID();
  if (global_costmap_soft_costs_pub_->get_subscription_count() > 0U) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = costmap_frame;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(visible_cell_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    for (unsigned int my = 0U; my < size_y; ++my) {
      for (unsigned int mx = 0U; mx < size_x; ++mx) {
        const unsigned char cost = costs[static_cast<size_t>(my) * size_x + mx];
        if (!is_visible_cost(cost)) {
          continue;
        }
        *iter_x = static_cast<float>(origin_x + (static_cast<double>(mx) + 0.5) * resolution);
        *iter_y = static_cast<float>(origin_y + (static_cast<double>(my) + 0.5) * resolution);
        *iter_z = 0.08F;
        *iter_intensity = static_cast<float>(cost);
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
      }
    }
    cloud.is_dense = true;
    global_costmap_soft_costs_pub_->publish(cloud);
  }

  if (dynamic_costmap_inflation_pub_) {
    size_t dynamic_cells = 0U;
    for (size_t index = 0U; index < dynamic_neighborhood.size(); ++index) {
      dynamic_cells += dynamic_neighborhood[index] != 0U && is_visible_cost(costs[index]);
    }
    sensor_msgs::msg::PointCloud2 dynamic_cloud;
    dynamic_cloud.header = header;
    dynamic_cloud.header.frame_id = costmap_frame;
    sensor_msgs::PointCloud2Modifier dynamic_modifier(dynamic_cloud);
    dynamic_modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    dynamic_modifier.resize(dynamic_cells);
    sensor_msgs::PointCloud2Iterator<float> dynamic_x(dynamic_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> dynamic_y(dynamic_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> dynamic_z(dynamic_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> dynamic_intensity(dynamic_cloud, "intensity");
    size_t dynamic_soft_cells = 0U;
    for (unsigned int my = 0U; my < size_y; ++my) {
      for (unsigned int mx = 0U; mx < size_x; ++mx) {
        const size_t index = static_cast<size_t>(my) * size_x + mx;
        if (dynamic_neighborhood.empty() || !dynamic_neighborhood[index] ||
            !is_visible_cost(costs[index])) {
          continue;
        }
        *dynamic_x = static_cast<float>(origin_x + (static_cast<double>(mx) + 0.5) * resolution);
        *dynamic_y = static_cast<float>(origin_y + (static_cast<double>(my) + 0.5) * resolution);
        *dynamic_z = 0.17F;
        *dynamic_intensity = static_cast<float>(costs[index]);
        dynamic_soft_cells += costs[index] < nav2_costmap_2d::LETHAL_OBSTACLE;
        ++dynamic_x;
        ++dynamic_y;
        ++dynamic_z;
        ++dynamic_intensity;
      }
    }
    dynamic_cloud.is_dense = true;
    dynamic_costmap_inflation_pub_->publish(dynamic_cloud);
    auto node = node_.lock();
    if (node) {
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000,
        "[MincoVisualizer] dynamic final costmap: sources=%zu missing_cores=%zu "
        "inflated_soft=%zu visible=%zu radius=%.2f subscribers=%zu",
        dynamic_sources, missing_dynamic_cores, dynamic_soft_cells, dynamic_cells,
        inflation_radius, dynamic_costmap_inflation_pub_->get_subscription_count());
    }
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "[MincoVisualizer] final costmap cloud: cells=%zu subscribers=%zu "
      "frame='%s' (cost 1..254 includes inflation and lethal obstacle cores)",
      visible_cell_count, global_costmap_soft_costs_pub_->get_subscription_count(),
      costmap_frame.c_str());
  }
}

nav_msgs::msg::Path Visualizer::convertTrajectoryToPath(
  const traj_opt::Trajectory & traj, const std_msgs::msg::Header & header, int steps, double t_step) const
{
  nav_msgs::msg::Path path_msg;
  path_msg.header = header;

  if (steps <= 0 || t_step <= 0.0) {
    return path_msg;
  }

  path_msg.poses.resize(static_cast<size_t>(steps));

  const double total_duration = traj.getTotalDuration();
  for (int i = 0; i < steps; ++i) {
    double t = i * t_step;
    if (t > total_duration) {
      t = total_duration;
    }

    Eigen::Vector3d pos = traj.getPos(t);
    Eigen::Vector3d vel = traj.getVel(t);

    pos.z() = 0.0;
    vel.z() = 0.0;

    double yaw = 0.0;
    if (vel.head<2>().norm() > 1e-4) {
      yaw = std::atan2(vel(1), vel(0));
    } else if (i > 0) {
      const auto & last_q = path_msg.poses[static_cast<size_t>(i - 1)].pose.orientation;
      yaw = 2.0 * std::atan2(last_q.z, last_q.w);
    }

    auto & pose = path_msg.poses[static_cast<size_t>(i)];
    pose.header = header;
    pose.pose.position.x = pos(0);
    pose.pose.position.y = pos(1);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
  }

  return path_msg;
}

}  // namespace minco_planner
