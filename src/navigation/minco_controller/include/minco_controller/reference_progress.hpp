#pragma once

#include <algorithm>
#include <cmath>

namespace minco_controller {

inline bool computeReferenceIndex(
    double nearest_index, bool same_trajectory, double tracked_spatial_index,
    double tracked_reference_index,
    double elapsed_since_update, double sample_dt, double maximum_index,
    double maximum_lead_time, double &spatial_index,
    double &reference_index) {
  if (!std::isfinite(nearest_index) || nearest_index < 0.0 ||
      !std::isfinite(maximum_index) || maximum_index < 0.0 ||
      !std::isfinite(sample_dt) || sample_dt <= 0.0 ||
      !std::isfinite(maximum_lead_time) || maximum_lead_time < 0.0) {
    return false;
  }

  spatial_index = nearest_index;
  double progress_index = nearest_index;
  if (same_trajectory) {
    if (!std::isfinite(tracked_spatial_index) ||
        tracked_spatial_index < 0.0 ||
        !std::isfinite(tracked_reference_index) ||
        tracked_reference_index < 0.0 ||
        !std::isfinite(elapsed_since_update) || elapsed_since_update < 0.0) {
      return false;
    }
    spatial_index = std::max(nearest_index, tracked_spatial_index);
    const double time_progress =
        tracked_reference_index + elapsed_since_update / sample_dt;
    const double spatial_limit =
        spatial_index + maximum_lead_time / sample_dt;
    progress_index = std::max(
        tracked_reference_index, std::min(time_progress, spatial_limit));
  }

  spatial_index = std::clamp(spatial_index, 0.0, maximum_index);
  reference_index = std::clamp(
      std::max(spatial_index, progress_index), 0.0, maximum_index);
  return std::isfinite(spatial_index) && std::isfinite(reference_index);
}

}  // namespace minco_controller
