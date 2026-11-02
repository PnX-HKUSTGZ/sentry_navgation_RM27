#pragma once

#include "rog_map/map_query_interface.hpp"
#include "minco_core/components/footprint_geometry.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace minco_planner {

class OverlapEscapeGuard {
public:
  struct Config {
    std::vector<Eigen::Vector2d> footprint;
    StaticOverlapPolicy static_overlap;
    double max_speed{0.20};
    double max_duration{1.0};
    double max_distance{0.20};
    double max_penetration{0.05};
    double max_overlap_ratio{0.10};
    // Do not enter recovery for a trajectory rejection that has no meaningful
    // current body overlap. Those cases are usually a near-wall guide error;
    // restarting the same escape only creates a stop/replan oscillation.
    double min_overlap_ratio{0.01};
    double tracking_tolerance{0.03};
    double map_timeout{0.50};
    double future_tolerance{0.05};
    bool allow_unknown_motion{false};
  };
  struct Context {
    Eigen::Vector3d start;
    Eigen::Vector2d velocity;
    double yaw{0.0};
    // The validated horizon can be shorter than the configured maximum when
    // a narrow corridor only has room for a small corrective motion.
    double duration{0.0};
    double distance{0.0};
    std::vector<Eigen::Vector2d> allowed_cells;
    std::vector<double> last_cell_areas;
    double last_overlap_area{0.0};
    std::shared_ptr<rog_map::MapQueryInterface> static_query;
    std::shared_ptr<Context> static_context;
    std::chrono::steady_clock::time_point started;
    std::mutex mutex;
  };

  explicit OverlapEscapeGuard(Config config);
  std::shared_ptr<Context> begin(const Eigen::Vector3d & start, double yaw,
    const Eigen::Vector2d & velocity,
    const std::shared_ptr<rog_map::MapQueryInterface> & query, double now,
    std::string * reason = nullptr,
    std::shared_ptr<rog_map::MapQueryInterface> static_query = nullptr) const;
  bool check(Context & context, const Eigen::Vector3d & actual, double yaw,
    const std::shared_ptr<rog_map::MapQueryInterface> & query, double now) const;

private:
  bool validate(Context & context, const Eigen::Vector3d & start, double yaw,
    double duration, const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double now, bool initialize, std::string * reason = nullptr,
    bool dynamic_only = false, bool static_grid = false) const;
  Config config_;
};
}  // namespace minco_planner
