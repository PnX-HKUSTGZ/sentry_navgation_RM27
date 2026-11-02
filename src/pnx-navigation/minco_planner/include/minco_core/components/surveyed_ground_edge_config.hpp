#ifndef MINCO_PLANNER__SURVEYED_GROUND_EDGE_CONFIG_HPP_
#define MINCO_PLANNER__SURVEYED_GROUND_EDGE_CONFIG_HPP_

#include <cstdint>

namespace minco_planner
{

struct SurveyedGroundEdgeConfig
{
  double max_step{0.06};
  double max_slope_deg{28.0};
  double lethal_clearance_radius{0.0};
  double clearance_radius{0.20};
  uint8_t clearance_cost{240U};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__SURVEYED_GROUND_EDGE_CONFIG_HPP_
