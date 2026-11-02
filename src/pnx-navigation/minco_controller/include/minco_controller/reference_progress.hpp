#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace minco_controller {

inline bool computeReferenceIndex(
    double nearest_index, bool same_trajectory, double tracked_spatial_index,
    double tracked_reference_index, double elapsed_since_update,
    double sample_dt, double maximum_index, double maximum_lead_time,
    double &spatial_index, double &reference_index) {
  if (!std::isfinite(nearest_index) || nearest_index < 0.0 ||
      !std::isfinite(maximum_index) || maximum_index < 0.0 ||
      !std::isfinite(sample_dt) || sample_dt <= 0.0 ||
      !std::isfinite(maximum_lead_time) || maximum_lead_time < 0.0) {
    return false;
  }

  spatial_index = nearest_index;
  double progress_index = nearest_index;
  if (same_trajectory) {
    if (!std::isfinite(tracked_spatial_index) || tracked_spatial_index < 0.0 ||
        !std::isfinite(tracked_reference_index) ||
        tracked_reference_index < 0.0 || !std::isfinite(elapsed_since_update) ||
        elapsed_since_update < 0.0) {
      return false;
    }
    spatial_index = std::max(nearest_index, tracked_spatial_index);
    const double time_progress =
        tracked_reference_index + elapsed_since_update / sample_dt;
    const double spatial_limit = spatial_index + maximum_lead_time / sample_dt;
    progress_index = std::max(tracked_reference_index,
                              std::min(time_progress, spatial_limit));
  }

  spatial_index = std::clamp(spatial_index, 0.0, maximum_index);
  reference_index =
      std::clamp(std::max(spatial_index, progress_index), 0.0, maximum_index);
  return std::isfinite(spatial_index) && std::isfinite(reference_index);
}

inline bool applyStationaryStartupReferenceFloor(
    const std::vector<double> &reference_speeds, double measured_speed,
    double stationary_speed_threshold, double target_reference_speed,
    double sample_dt, double maximum_lead_time, double spatial_index,
    double maximum_index, double &reference_index) {
  if (reference_speeds.empty() || !std::isfinite(measured_speed) ||
      measured_speed < 0.0 || !std::isfinite(stationary_speed_threshold) ||
      stationary_speed_threshold < 0.0 ||
      !std::isfinite(target_reference_speed) || target_reference_speed <= 0.0 ||
      !std::isfinite(sample_dt) || sample_dt <= 0.0 ||
      !std::isfinite(maximum_lead_time) || maximum_lead_time < 0.0 ||
      !std::isfinite(spatial_index) || spatial_index < 0.0 ||
      !std::isfinite(maximum_index) || maximum_index < 0.0 ||
      !std::isfinite(reference_index) || reference_index < 0.0) {
    return false;
  }

  if (measured_speed > stationary_speed_threshold) {
    return true;
  }

  const double bounded_maximum_index = std::min(
      maximum_index, static_cast<double>(reference_speeds.size() - 1U));
  const double search_end = std::min(
      bounded_maximum_index, spatial_index + maximum_lead_time / sample_dt);
  const size_t first_index = static_cast<size_t>(
      std::clamp(std::ceil(spatial_index), 0.0, bounded_maximum_index));
  const size_t last_index = static_cast<size_t>(
      std::clamp(std::floor(search_end + 1.0e-9), 0.0, bounded_maximum_index));

  if (first_index > last_index) {
    return true;
  }
  for (size_t index = first_index; index <= last_index; ++index) {
    const double speed = reference_speeds[index];
    if (!std::isfinite(speed) || speed < 0.0) {
      return false;
    }
    if (speed + 1.0e-9 >= target_reference_speed) {
      reference_index = std::max(reference_index, static_cast<double>(index));
      break;
    }
  }
  reference_index = std::clamp(reference_index, 0.0, bounded_maximum_index);
  return std::isfinite(reference_index);
}

inline bool shouldSuppressPlanarCommand(double command_speed,
                                        double deadzone_speed,
                                        bool stationary_startup_active) {
  return !stationary_startup_active && command_speed < deadzone_speed;
}

inline bool applyStationaryStartupCommandFloor(bool stationary_startup_active,
                                               double minimum_command_speed,
                                               double reference_vx,
                                               double reference_vy,
                                               double &command_vx,
                                               double &command_vy) {
  if (!std::isfinite(minimum_command_speed) || minimum_command_speed < 0.0 ||
      !std::isfinite(reference_vx) || !std::isfinite(reference_vy) ||
      !std::isfinite(command_vx) || !std::isfinite(command_vy)) {
    return false;
  }
  if (!stationary_startup_active || minimum_command_speed <= 0.0) {
    return true;
  }

  const double command_speed = std::hypot(command_vx, command_vy);
  if (command_speed + 1.0e-9 >= minimum_command_speed) {
    return true;
  }

  constexpr double kDirectionEpsilon = 1.0e-9;
  if (command_speed > kDirectionEpsilon) {
    const double scale = minimum_command_speed / command_speed;
    command_vx *= scale;
    command_vy *= scale;
    return true;
  }

  const double reference_speed = std::hypot(reference_vx, reference_vy);
  if (reference_speed > kDirectionEpsilon) {
    command_vx = minimum_command_speed * reference_vx / reference_speed;
    command_vy = minimum_command_speed * reference_vy / reference_speed;
  }
  return true;
}

} // namespace minco_controller
