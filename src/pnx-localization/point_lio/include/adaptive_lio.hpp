#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace point_lio
{

struct PlaneModel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double offset = 0.0;
  double residual = 0.0;
  double surface_variance = 0.0;
  double innovation_variance = 0.0;
  double curvature = 1.0;
  double quality = 0.0;
  double responsibility = 0.0;
  double robust_weight = 0.0;
  std::size_t support = 0;
  std::size_t scale_index = 0;
};

using PlaneModelVector = std::vector<PlaneModel, Eigen::aligned_allocator<PlaneModel>>;

struct SpatialAdaptiveConfig
{
  bool enabled = false;
  std::array<int, 3> neighbors{5, 10, 20};
  std::array<double, 3> max_query_ranges{0.35, 0.70, 1.40};
  double max_curvature = 0.03;
  double max_plane_residual = 0.10;
  double normalized_residual_gate = 3.0;
  double outlier_probability = 0.05;
  double student_t_dof = 5.0;
  double min_responsibility = 0.05;
  double min_surface_variance = 1.0e-5;
  double max_information_scale = 10.0;
  double degeneracy_eigen_ratio = 1.0e-4;
};

inline bool fitPlane(
  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> & points,
  std::size_t count, double max_curvature, double max_plane_residual, PlaneModel & model)
{
  if (count < 3 || points.size() < count) {
    return false;
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < count; ++i) {
    mean += points[i];
  }
  mean /= static_cast<double>(count);

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < count; ++i) {
    const Eigen::Vector3d centered = points[i] - mean;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(count);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return false;
  }

  const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  const double eigenvalue_sum = eigenvalues.sum();
  if (eigenvalue_sum <= 1.0e-12 || eigenvalues(1) <= 1.0e-8) {
    return false;
  }

  model.normal = solver.eigenvectors().col(0).normalized();
  model.offset = -model.normal.dot(mean);
  model.curvature = eigenvalues(0) / eigenvalue_sum;
  model.surface_variance = eigenvalues(0) / static_cast<double>(count);
  model.support = count;

  double largest_residual = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    largest_residual =
      std::max(largest_residual, std::abs(model.normal.dot(points[i]) + model.offset));
  }

  if (model.curvature > max_curvature || largest_residual > max_plane_residual) {
    return false;
  }

  const double curvature_quality =
    std::clamp(1.0 - model.curvature / std::max(max_curvature, 1.0e-9), 0.0, 1.0);
  const double spread_quality = std::clamp(eigenvalues(1) / (eigenvalues(2) + 1.0e-12), 0.0, 1.0);
  model.quality = std::max(1.0e-3, curvature_quality * std::sqrt(spread_quality));
  return true;
}

inline void computeResponsibilities(
  PlaneModelVector & models, double outlier_probability, double normalized_residual_gate,
  double student_t_dof, double min_responsibility)
{
  if (models.empty()) {
    return;
  }

  std::vector<double> likelihoods(models.size(), 0.0);
  double total_likelihood = std::clamp(outlier_probability, 1.0e-9, 0.99);
  const double scale_prior = (1.0 - std::clamp(outlier_probability, 0.0, 0.99)) /
                             static_cast<double>(models.size());

  for (std::size_t i = 0; i < models.size(); ++i) {
    auto & model = models[i];
    const double variance = std::max(model.innovation_variance, 1.0e-12);
    const double nis = model.residual * model.residual / variance;
    if (!std::isfinite(nis) || std::sqrt(nis) > normalized_residual_gate) {
      continue;
    }

    likelihoods[i] = scale_prior * model.quality * std::exp(-0.5 * nis) / std::sqrt(variance);
    total_likelihood += likelihoods[i];
  }

  const double dof = std::max(student_t_dof, 1.0);
  for (std::size_t i = 0; i < models.size(); ++i) {
    auto & model = models[i];
    model.responsibility = likelihoods[i] / std::max(total_likelihood, 1.0e-12);
    const double variance = std::max(model.innovation_variance, 1.0e-12);
    const double nis = model.residual * model.residual / variance;
    model.robust_weight = (dof + 1.0) / (dof + std::max(nis, 0.0));
    if (model.responsibility < min_responsibility) {
      model.responsibility = 0.0;
      model.robust_weight = 0.0;
    }
  }
}

inline double stableLogDet(const Eigen::Matrix<double, 6, 6> & covariance)
{
  const Eigen::Matrix<double, 6, 6> symmetric = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(symmetric, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double result = 0.0;
  for (const double value : solver.eigenvalues()) {
    result += std::log(std::max(value, 1.0e-12));
  }
  return result;
}

struct TemporalAdaptiveConfig
{
  bool enabled = false;
  int min_points = 8;
  int initial_target_points = 32;
  int max_points = 128;
  double min_duration = 0.001;
  double max_duration = 0.010;
  double max_translation = 0.02;
  double max_rotation = 0.8 * M_PI / 180.0;
  double latency_penalty = 0.02;
  double information_ema_alpha = 0.10;
  double hysteresis_ratio = 0.15;
  double max_pose_covariance_trace = 1.0;
};

class TemporalUpdateScheduler
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TemporalUpdateScheduler(TemporalAdaptiveConfig config = {}) : config_(config) { reset(); }

  void setConfig(const TemporalAdaptiveConfig & config)
  {
    config_ = config;
    reset();
  }

  void reset()
  {
    initialized_ = false;
    pending_points_ = 0;
    commanded_points_ = std::clamp(
      config_.initial_target_points, std::max(config_.min_points, 1),
      std::max(config_.max_points, config_.min_points));
    information_per_point_ema_ = 0.0;
    last_update_time_ = 0.0;
    last_update_position_.setZero();
    last_update_rotation_.setIdentity();
  }

  bool shouldUpdate(
    double timestamp, int new_points, const Eigen::Vector3d & position,
    const Eigen::Matrix3d & rotation, double pose_covariance_trace, bool force)
  {
    pending_points_ += std::max(new_points, 0);
    if (!config_.enabled) {
      return true;
    }
    if (!initialized_) {
      initialized_ = true;
      last_update_time_ = timestamp;
      last_update_position_ = position;
      last_update_rotation_ = rotation;
    }

    const double elapsed = std::max(0.0, timestamp - last_update_time_);
    const double translation = (position - last_update_position_).norm();
    const Eigen::AngleAxisd rotation_delta(last_update_rotation_.transpose() * rotation);
    const double angle = std::abs(rotation_delta.angle());
    const int lower_target = std::max(
      config_.min_points,
      static_cast<int>(std::floor(commanded_points_ * (1.0 - config_.hysteresis_ratio))));

    const bool enough_points = pending_points_ >= lower_target && elapsed >= config_.min_duration;
    const bool hard_limit = pending_points_ >= config_.max_points || elapsed >= config_.max_duration;
    const bool excessive_motion =
      translation >= config_.max_translation || angle >= config_.max_rotation;
    const bool covariance_emergency = pose_covariance_trace >= config_.max_pose_covariance_trace;
    return force || hard_limit || excessive_motion || covariance_emergency || enough_points;
  }

  void recordUpdate(
    double timestamp, const Eigen::Vector3d & position, const Eigen::Matrix3d & rotation,
    double information_gain, double effective_points)
  {
    if (std::isfinite(information_gain) && information_gain > 0.0 && effective_points > 0.0) {
      const double sample = information_gain / effective_points;
      if (information_per_point_ema_ <= 0.0) {
        information_per_point_ema_ = sample;
      } else {
        const double alpha = std::clamp(config_.information_ema_alpha, 0.001, 1.0);
        information_per_point_ema_ =
          (1.0 - alpha) * information_per_point_ema_ + alpha * sample;
      }
      // Minimizing J(N) = 1 / (j_hat * N) + lambda * N gives
      // N* = 1 / sqrt(lambda * j_hat).
      const double raw_command = 1.0 / std::sqrt(
        std::max(config_.latency_penalty, 1.0e-12) *
        std::max(information_per_point_ema_, 1.0e-12));
      commanded_points_ = std::clamp(
        static_cast<int>(std::lround(raw_command)), std::max(config_.min_points, 1),
        std::max(config_.max_points, config_.min_points));
    }

    initialized_ = true;
    pending_points_ = 0;
    last_update_time_ = timestamp;
    last_update_position_ = position;
    last_update_rotation_ = rotation;
  }

  int pendingPoints() const { return pending_points_; }
  int commandedPoints() const { return commanded_points_; }
  double informationPerPoint() const { return information_per_point_ema_; }

private:
  TemporalAdaptiveConfig config_;
  bool initialized_ = false;
  int pending_points_ = 0;
  int commanded_points_ = 0;
  double information_per_point_ema_ = 0.0;
  double last_update_time_ = 0.0;
  Eigen::Vector3d last_update_position_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d last_update_rotation_ = Eigen::Matrix3d::Identity();
};

struct FrameDiagnostics
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::size_t raw_points = 0;
  std::size_t filtered_points = 0;
  std::size_t associated_points = 0;
  std::size_t effective_rows = 0;
  std::size_t lidar_updates = 0;
  std::size_t skipped_updates = 0;
  std::array<double, 3> scale_responsibility{0.0, 0.0, 0.0};
  double effective_points = 0.0;
  double residual_abs_sum = 0.0;
  double nis_sum = 0.0;
  double information_gain_sum = 0.0;
  Eigen::Matrix<double, 6, 6> pose_information = Eigen::Matrix<double, 6, 6>::Zero();
  bool imu_saturated = false;

  void reset()
  {
    *this = FrameDiagnostics{};
  }
};

}  // namespace point_lio
