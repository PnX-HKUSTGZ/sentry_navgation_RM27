#include "joint_startup.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include "zlio/inertial.hpp"
#include "zlio/state.hpp"

namespace point_lio
{
zlio::Mat15 startupErrorJacobian(const zlio::State & state)
{
  const auto R = state.pose.rotation.toRotationMatrix();
  zlio::Mat15 J = zlio::Mat15::Zero();
  J.block<3, 3>(0, 0).setIdentity();
  J.block<3, 3>(0, 3) = -zlio::skew(state.pose.position);
  J.block<3, 3>(3, 3) = R.transpose();
  J.block<3, 3>(6, 3) = -zlio::skew(state.velocity);
  J.block<3, 3>(6, 6).setIdentity();
  J.block<3, 3>(9, 9) = R.transpose();
  J.block<3, 3>(12, 9) = -R.transpose() * zlio::skew(state.velocity);
  J.block<3, 3>(12, 12) = R.transpose();
  return J;
}

StartupSeed makeStartupSeed(zlio::InitializationResult result, const zlio::Imu & boundary,
                           double gyro_variance, double accel_variance)
{
  if (boundary.time != result.estimate.state.time ||
      !boundary.angular_velocity.allFinite() || !boundary.acceleration.allFinite() ||
      !std::isfinite(gyro_variance) || gyro_variance <= 0 ||
      !std::isfinite(accel_variance) || accel_variance <= 0) {
    throw std::invalid_argument("invalid startup boundary or IMU variance");
  }
  const auto & state = result.estimate.state;
  const zlio::Mat15 J = startupErrorJacobian(state);
  const zlio::Mat15 covariance = J * result.estimate.covariance * J.transpose();
  Eigen::Matrix<double, 30, 15> A = Eigen::Matrix<double, 30, 15>::Zero();
  A.block<6, 6>(0, 0).setIdentity();
  A.block<3, 3>(12, 6).setIdentity();
  A.block<6, 6>(24, 9).setIdentity();
  A.block<3, 3>(15, 9) = -Eigen::Matrix3d::Identity();
  A.block<3, 3>(18, 12) = -Eigen::Matrix3d::Identity();
  StartupSeed seed;
  seed.covariance_output = A * covariance * A.transpose();
  seed.covariance_output.block<6, 6>(6, 6).diagonal().setConstant(1e-8);
  seed.covariance_output.block<3, 3>(15, 15).diagonal().array() += gyro_variance;
  seed.covariance_output.block<3, 3>(18, 18).diagonal().array() += accel_variance;
  // Gravity is a fixed magnitude/direction in the aligned startup frame. Retain
  // a small runtime gravity prior, as Point-LIO carries it in its state vector.
  seed.covariance_output.block<3, 3>(21, 21).diagonal().setConstant(1e-4);
  seed.covariance_output = (0.5 * seed.covariance_output +
                           0.5 * seed.covariance_output.transpose()).eval();
  const std::array<int, 8> blocks{0, 3, 6, 9, 12, 24, 27, 21};
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      seed.covariance_input.block<3, 3>(i * 3, j * 3) =
        seed.covariance_output.block<3, 3>(blocks[i], blocks[j]);
    }
  }
  if (Eigen::LLT<Eigen::Matrix<double, 30, 30>>(seed.covariance_output).info() != Eigen::Success ||
      !seed.covariance_output.allFinite() || result.map.empty()) {
    throw zlio::NumericalError("invalid startup covariance/map after Point-LIO conversion");
  }
  seed.boundary_imu = boundary;
  seed.angular_velocity = boundary.angular_velocity - state.gyro_bias;
  seed.specific_force = boundary.acceleration - state.accel_bias;
  seed.solution = std::move(result);
  return seed;
}

JointStartup::JointStartup(JointStartupConfig config) : config_(std::move(config))
{
  for (double v : {config_.acceleration_scale, config_.max_imu_gap, config_.max_scan_duration,
                   config_.acceleration_limit, config_.angular_velocity_limit}) {
    if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("invalid joint startup limits");}
  }
  if (config_.min_scan_points < 3 || config_.max_scan_points < config_.min_scan_points ||
      config_.solver.initialization.window_frames > 60) {
    throw std::invalid_argument("joint startup point/window resource bounds are invalid");
  }
  config_.solver = zlio::normalizeConfig(config_.solver);
  reset();
}
void JointStartup::reset()
{
  initializer_ = std::make_unique<zlio::Initializer>(config_.solver);
  zlio::ImuHistoryLimits limits;
  limits.maximum_interval = config_.max_imu_gap;
  limits.maximum_duration = 30.0;
  limits.maximum_samples = 20000;
  history_ = std::make_unique<zlio::ImuHistory>(limits);
  boundary_.reset();
  packets_ = 0;
  compute_ms_ = 0;
  reason_ = "collecting synchronized scans";
  phase_ = "COLLECTING";
}
std::optional<StartupSeed> JointStartup::add(
  const zlio::Scan & raw, zlio::Time begin, zlio::Time end,
  const std::vector<zlio::Imu> & imus)
{
  const auto start = std::chrono::steady_clock::now();
  try {
    if (begin > end || zlio::seconds(end - begin) > config_.max_scan_duration ||
        raw.size() < config_.min_scan_points || (boundary_ && end <= *boundary_)) {
      throw std::invalid_argument("invalid startup scan interval/point count or time rollback");
    }
    std::optional<zlio::Time> previous_sample;
    for (auto sample : imus) {
      if (previous_sample && sample.time <= *previous_sample) {
        throw std::invalid_argument("unordered startup IMU packet");
      }
      previous_sample = sample.time;
      if (!sample.angular_velocity.allFinite() || !sample.acceleration.allFinite()) {
        throw std::invalid_argument("nonfinite startup IMU");
      }
      if (config_.check_saturation &&
          (sample.angular_velocity.cwiseAbs().maxCoeff() >= .99 * config_.angular_velocity_limit ||
           sample.acceleration.cwiseAbs().maxCoeff() >= .99 * config_.acceleration_limit)) {
        throw std::invalid_argument("saturated startup IMU");
      }
      sample.acceleration *= config_.acceleration_scale;
      if (!history_->samples().empty() && sample.time <= history_->samples().back().time) {
        continue;  // Adjacent scan bundles deliberately share bracketing samples.
      }
      history_->push(sample);
    }
    if (!boundary_) {boundary_ = begin;}
    if (end == *boundary_) {
      reason_ = "waiting for next scan boundary";
      return std::nullopt;
    }
    if (!history_->covers(*boundary_) || !history_->covers(end)) {
      boundary_ = end;
      phase_ = "COLLECTING";
      reason_ = "waiting for complete IMU coverage";
      return std::nullopt;
    }
    std::vector<zlio::Imu> interval{history_->at(*boundary_)};
    for (const auto & sample : history_->samples()) {
      if (sample.time > *boundary_ && sample.time < end) {interval.push_back(sample);}
    }
    interval.push_back(history_->at(end));
    zlio::Scan scan;
    // Bound memory while preserving measured points and their individual timestamps.
    const std::size_t stride = std::max<std::size_t>(1,
      (raw.size() + config_.max_scan_points - 1) / config_.max_scan_points);
    for (std::size_t i = 0; i < raw.size(); ++i) {
      const auto & point = raw[i];
      if (!point.position.allFinite() || point.time < begin || point.time > end) {
        throw std::invalid_argument("invalid startup point or timestamp");
      }
      if (i % stride == 0 && point.time >= *boundary_) {scan.push_back(point);}
    }
    if (scan.size() < config_.min_scan_points || end <= *boundary_) {
      throw std::invalid_argument("insufficient causal startup points");
    }
    auto solution = initializer_->add(scan, end, interval);
    ++packets_;
    boundary_ = end;
    history_->discardBefore(end);
    reason_ = initializer_->status().reason;
    phase_ = solution ? "READY" :
      (initializer_->status().state == zlio::InitializationState::Collecting ? "COLLECTING" : "RETRYING");
    compute_ms_ += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    if (solution) {
      return makeStartupSeed(std::move(*solution), interval.back(),
        config_.solver.noise.gyro_sample_variance, config_.solver.noise.accel_sample_variance);
    }
  } catch (const std::exception & error) {
    const std::string reason = error.what();
    reset();
    phase_ = "RETRYING";
    reason_ = reason;
  }
  return std::nullopt;
}
}  // namespace point_lio
