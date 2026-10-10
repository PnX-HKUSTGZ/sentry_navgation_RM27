#include "minco_core/components/footprint_geometry.hpp"

#include <boost/geometry.hpp>

namespace minco_planner {
namespace {
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

Polygon makePolygon(const std::vector<Eigen::Vector2d> & points) {
  Polygon result;
  for (const auto & point : points) {bg::append(result.outer(), Point(point.x(), point.y()));}
  bg::correct(result);
  return result;
}
}

double polygonArea(const std::vector<Eigen::Vector2d> & points) {
  return std::abs(bg::area(makePolygon(points)));
}

double footprintCellOverlapArea(const std::vector<Eigen::Vector2d> & body,
  const Eigen::Vector2d & center, const Eigen::Vector2d & ax, const Eigen::Vector2d & ay)
{
  std::vector<Polygon> intersections;
  bg::intersection(makePolygon(body), makePolygon({center - 0.5 * ax - 0.5 * ay,
    center + 0.5 * ax - 0.5 * ay, center + 0.5 * ax + 0.5 * ay,
    center - 0.5 * ax + 0.5 * ay}), intersections);
  double area = 0.0;
  for (const auto & intersection : intersections) {area += std::abs(bg::area(intersection));}
  return area;
}

std::vector<Eigen::Vector2d> insetFootprint(
  const std::vector<Eigen::Vector2d> & body, double depth)
{
  bg::model::multi_polygon<Polygon> inset;
  bg::buffer(makePolygon(body), inset, bg::strategy::buffer::distance_symmetric<double>(-depth),
    bg::strategy::buffer::side_straight(), bg::strategy::buffer::join_miter(),
    bg::strategy::buffer::end_flat(), bg::strategy::buffer::point_circle(8));
  if (inset.size() != 1U || !bg::is_valid(inset.front())) {return {};}
  std::vector<Eigen::Vector2d> result;
  const auto & ring = inset.front().outer();
  for (size_t i = 0; i + 1U < ring.size(); ++i) {
    result.emplace_back(bg::get<0>(ring[i]), bg::get<1>(ring[i]));
  }
  return result;
}
}  // namespace minco_planner
