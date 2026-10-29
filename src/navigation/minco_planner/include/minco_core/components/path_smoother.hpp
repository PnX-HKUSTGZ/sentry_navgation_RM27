#pragma once

#include <Eigen/Core>
#include <functional>
#include <vector>
#include "rog_map/map_query_interface.hpp"

namespace minco_planner {

struct PathSmoothingOptions
{
  bool enabled{true};
  bool enable_shortcuts{false};  // bounded, collision/cost-checked long-span straightening
  int max_iterations{60};
  double data_weight{0.05};
  double smooth_weight{0.3};
  double max_deviation{0.25};  // metres from the original guide vertex
  double tolerance{1e-4};     // metres per sweep
  double max_time_ms{10.0};   // cooperative iteration budget; checks finish atomically
  double peak_cost_slack{10.0};
  double mean_cost_slack{5.0};
  void validate() const;
};

// In-place, bounded coordinate descent on squared second differences plus
// reference attraction. Endpoints and vertex count are preserved. Rejected
// moves retain the previous guide; this is not a substitute for trajectory
// validation. Unknown/invalid reference edges are left untouched.
class PathSmoother
{
public:
  using PointCheck = std::function<bool(const Eigen::Vector3d &)>;
  using CellCost = std::function<unsigned char(unsigned int, unsigned int)>;
  static bool smooth(std::vector<Eigen::Vector3d> & path,
    const rog_map::MapQueryInterface & map,
    const PathSmoothingOptions & options,
    const PointCheck & point_is_safe = {},
    const CellCost & cell_cost = {},
    const std::function<bool()> & canceled = {});
};

}  // namespace minco_planner
