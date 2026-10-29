#include "minco_core/components/path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include "nav2_costmap_2d/cost_values.hpp"

namespace minco_planner {
namespace {
struct Profile
{
  bool valid{false};
  double peak{0.0};
  double mean{0.0};
};

// Visit every cell crossed by a segment, including both side cells at an
// exact grid-corner crossing. Half-cell point sampling alone can miss a
// arbitrarily short intersection with an occupied cell.
Profile profile(const Eigen::Vector3d & a, const Eigen::Vector3d & b,
  const rog_map::MapQueryInterface & map, const PathSmoother::CellCost & cost,
  const PathSmoother::PointCheck & safe, bool guard_samples = false)
{
  Profile result;
  unsigned int ax, ay, bx, by;
  if (!a.allFinite() || !b.allFinite() ||
    !map.worldToMap(a.x(), a.y(), ax, ay) ||
    !map.worldToMap(b.x(), b.y(), bx, by)) {return result;}
  const double resolution = map.resolution();
  const Eigen::Vector2d delta = (b - a).head<2>();
  const int sx = (delta.x() > 0.0) - (delta.x() < 0.0);
  const int sy = (delta.y() > 0.0) - (delta.y() < 0.0);
  int x = static_cast<int>(ax), y = static_cast<int>(ay);
  const double inf = std::numeric_limits<double>::infinity();
  const double dx = sx ? resolution / std::abs(delta.x()) : inf;
  const double dy = sy ? resolution / std::abs(delta.y()) : inf;
  double tx = sx ? (map.originX() + (x + (sx > 0)) * resolution - a.x()) / delta.x() : inf;
  double ty = sy ? (map.originY() + (y + (sy > 0)) * resolution - a.y()) / delta.y() : inf;
  auto sample = [&](int cx, int cy, double weight) {
      if (cx < 0 || cy < 0 || !map.isValid(cx, cy)) {return false;}
      const auto c = cost(cx, cy);
      if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return false;}
      result.peak = std::max(result.peak, static_cast<double>(c));
      result.mean += weight * c;
      return true;
    };
  double t = 0.0;
  // A grid segment crosses at most width + height boundaries.
  const size_t budget = static_cast<size_t>(map.sizeX()) + map.sizeY() + 2U;
  for (size_t step = 0; step < budget; ++step) {
    const double next = std::clamp(std::min(tx, ty), t, 1.0);
    if (!sample(x, y, next - t)) {return result;}
    if (next >= 1.0) {
      if (!sample(bx, by, 0.0)) {return result;}
      result.valid = true;
      break;
    }
    if (std::abs(tx - ty) <= 1e-12) {
      if (!sample(x + sx, y, 0.0) || !sample(x, y + sy, 0.0)) {return result;}
      x += sx; y += sy; tx += dx; ty += dy;
    } else if (tx < ty) {
      x += sx; tx += dx;
    } else {
      y += sy; ty += dy;
    }
    t = next;
  }
  if (result.valid && safe) {
    const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / (0.5 * resolution))));
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    if (guard_samples && delta.norm() > 1e-9) {
      // A boolean footprint query has no continuous clearance bound. Reserve
      // an extra half cell on either side of a NEW edge so smoothing cannot
      // shave a corner between translation samples. Existing edges retain
      // their original safety policy if no guarded improvement is feasible.
      normal = Eigen::Vector3d(-delta.y(), delta.x(), 0.0).normalized() * (0.5 * resolution);
    }
    for (int i = 0; i <= samples; ++i) {
      const Eigen::Vector3d point = a + (b - a) * (static_cast<double>(i) / samples);
      if (!safe(point) || (guard_samples && (!safe(point + normal) || !safe(point - normal)))) {
        result.valid = false;
        break;
      }
    }
  }
  return result;
}
}  // namespace

void PathSmoothingOptions::validate() const
{
  if (max_iterations < 1 || max_iterations > 1000 ||
    !std::isfinite(data_weight) || data_weight <= 0.0 ||
    !std::isfinite(smooth_weight) || smooth_weight <= 0.0 ||
    !std::isfinite(max_deviation) || max_deviation <= 0.0 ||
    !std::isfinite(tolerance) || tolerance <= 0.0 ||
    !std::isfinite(max_time_ms) || max_time_ms <= 0.0 ||
    !std::isfinite(peak_cost_slack) || peak_cost_slack < 0.0 ||
    !std::isfinite(mean_cost_slack) || mean_cost_slack < 0.0)
  {
    throw std::invalid_argument("path_smoothing: invalid iteration, weight, distance or cost limit");
  }
}

bool PathSmoother::smooth(std::vector<Eigen::Vector3d> & path,
  const rog_map::MapQueryInterface & map, const PathSmoothingOptions & options,
  const PointCheck & point_is_safe, const CellCost & cell_cost,
  const std::function<bool()> & canceled)
{
  options.validate();
  if (!options.enabled || path.size() < 3U ||
    !std::isfinite(map.resolution()) || map.resolution() <= 0.0) {return false;}
  for (const auto & p : path) {if (!p.allFinite()) {return false;}}
  const auto original = path;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double, std::milli>(options.max_time_ms);
  // Cache static costs for this invocation only; never carry dynamic evidence
  // across replans. Local smoothing touches only the small guide neighbourhood.
  std::unordered_map<size_t, unsigned char> costs;
  const CellCost cost = [&](unsigned int x, unsigned int y) {
      const size_t key = static_cast<size_t>(y) * map.sizeX() + x;
      const auto found = costs.find(key);
      if (found != costs.end()) {return found->second;}
      const auto value = cell_cost ? cell_cost(x, y) : map.value(x, y);
      costs.emplace(key, value);
      return value;
    };
  std::vector<Profile> reference;
  reference.reserve(path.size() - 1U);
  for (size_t i = 1; i < path.size(); ++i) {
    if ((canceled && canceled()) || std::chrono::steady_clock::now() >= deadline) {return false;}
    reference.push_back(profile(path[i - 1], path[i], map, cost, point_is_safe));
  }
  auto allowed = [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b, size_t edge) {
      const bool moved = (a - original[edge]).squaredNorm() +
        (b - original[edge + 1]).squaredNorm() > 1e-18;
      const auto candidate = profile(a, b, map, cost, point_is_safe, moved);
      return candidate.valid &&
             candidate.peak <= reference[edge].peak + options.peak_cost_slack &&
             candidate.mean <= reference[edge].mean + options.mean_cost_slack;
    };
  bool changed = false;
  if (options.enable_shortcuts) {
    // Local second-difference descent rounds individual 8-connected grid
    // corners but converges too slowly to remove metre-long zigzags. Try
    // coarse spans first, preserving every vertex, height and endpoint.
    // Cost and displacement are always compared to the ORIGINAL guide, so
    // repeated accepted shortcuts cannot accumulate clearance/cost drift.
    for (const size_t span : {64U, 32U, 16U, 8U, 4U, 2U}) {
      for (size_t begin = 0; begin + 2 < path.size(); begin += span / 2) {
        if (canceled && canceled()) {path = original; return false;}
        if (std::chrono::steady_clock::now() >= deadline) {break;}
        const size_t end = std::min(begin + span, path.size() - 1);
        if (end - begin < 2 ||
          (path[end] - path[begin]).head<2>().norm() < 1e-6) {continue;}
        std::vector<Eigen::Vector3d> candidate;
        candidate.reserve(end - begin + 1);
        bool valid = true;
        double max_move = 0.;
        for (size_t j = begin; j <= end; ++j) {
          Eigen::Vector3d p = path[begin] + (path[end] - path[begin]) *
            (static_cast<double>(j - begin) / (end - begin));
          p.z() = original[j].z();
          if ((p - original[j]).norm() > options.max_deviation + 1e-12) {
            valid = false; break;
          }
          max_move = std::max(max_move, (p - path[j]).norm());
          candidate.push_back(p);
        }
        if (!valid || max_move < options.tolerance) {continue;}
        const auto proposed = [&](size_t j) -> Eigen::Vector3d {
            return j >= begin && j <= end ? candidate[j - begin] : path[j];
          };
        double before = 0., after = 0.;
        for (size_t j = std::max(size_t{1}, begin);
          j <= std::min(path.size() - 2, end); ++j) {
          before += (path[j-1] - 2.*path[j] + path[j+1]).head<2>().squaredNorm();
          after += (proposed(j-1) - 2.*proposed(j) + proposed(j+1)).head<2>().squaredNorm();
        }
        if (after >= before - 1e-12) {continue;}
        for (size_t j = begin; j < end; ++j) {
          if (canceled && canceled()) {path = original; return false;}
          if (std::chrono::steady_clock::now() >= deadline || !reference[j].valid ||
            !allowed(candidate[j-begin], candidate[j-begin+1], j)) {
            valid = false; break;
          }
        }
        if (valid) {
          std::copy(candidate.begin(), candidate.end(), path.begin() + begin);
          changed = true;
        }
      }
    }
  }
  // Refine the accepted geometry, rather than pulling it back toward the grid
  // zigzags. The original remains the hard displacement/cost reference below.
  const auto attraction_reference = path;
  for (int iteration = 0; iteration < options.max_iterations &&
    std::chrono::steady_clock::now() < deadline; ++iteration) {
    double max_move = 0.0;
    for (size_t k = 1; k + 1 < path.size(); ++k) {
      if (canceled && canceled()) {path = original; return false;}
      if (std::chrono::steady_clock::now() >= deadline) {break;}
      // Alternate sweep direction to avoid a persistent start/goal bias.
      const size_t i = iteration % 2 ? path.size() - 1U - k : k;
      if (!reference[i - 1].valid || !reference[i].valid) {continue;}
      Eigen::Vector3d numerator = options.data_weight * attraction_reference[i];
      double denominator = options.data_weight;
      for (size_t j = std::max(size_t{1}, i - 1); j <= std::min(path.size() - 2, i + 1); ++j) {
        const double coefficient = j == i ? -2.0 : 1.0;
        const Eigen::Vector3d residual = path[j - 1] - 2.0 * path[j] + path[j + 1] - coefficient * path[i];
        numerator -= options.smooth_weight * coefficient * residual;
        denominator += options.smooth_weight * coefficient * coefficient;
      }
      Eigen::Vector3d target = numerator / denominator;
      target.z() = original[i].z();
      const Eigen::Vector3d offset = target - original[i];
      if (offset.norm() > options.max_deviation) {
        target = original[i] + offset.normalized() * options.max_deviation;
      }
      const Eigen::Vector3d delta = target - path[i];
      for (double scale = 1.0; scale >= 1.0 / 32.0; scale *= 0.5) {
        const Eigen::Vector3d candidate = path[i] + scale * delta;
        const double move = (candidate - path[i]).norm();
        if (move < options.tolerance * 0.1) {break;}
        if ((candidate - path[i - 1]).head<2>().norm() < 1e-6 ||
          (candidate - path[i + 1]).head<2>().norm() < 1e-6) {continue;}
        if (allowed(path[i - 1], candidate, i - 1) && allowed(candidate, path[i + 1], i)) {
          path[i] = candidate;
          max_move = std::max(max_move, move);
          changed = true;
          break;
        }
      }
    }
    if (max_move < options.tolerance) {break;}
  }
  // A live footprint query may have changed during optimisation. Never keep
  // a partially invalidated result; the caller still owns final safety checks.
  for (size_t i = 1; i < path.size(); ++i) {
    if ((canceled && canceled()) ||
      (reference[i - 1].valid && !allowed(path[i - 1], path[i], i - 1)))
    {path = original; return false;}
  }
  return changed;
}
}  // namespace minco_planner
