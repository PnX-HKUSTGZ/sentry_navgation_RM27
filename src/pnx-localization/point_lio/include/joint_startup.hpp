#pragma once

#include <memory>
#include <optional>
#include <string>
#include "zlio/initialization.hpp"
#include "zlio/input.hpp"

namespace point_lio
{
struct JointStartupConfig
{
  zlio::Config solver;
  double acceleration_scale = 1.0;  // Raw IMU -> the runtime filter's acceleration units.
  double max_imu_gap = 0.03;
  double max_scan_duration = 0.12;
  double acceleration_limit = 40.0;  // Raw sensor units, matching Point-LIO saturation gates.
  double angular_velocity_limit = 35.0;
  std::size_t max_scan_points = 30000;
  std::size_t min_scan_points = 100;
  bool check_saturation = true;
};

struct StartupSeed
{
  zlio::InitializationResult solution;
  zlio::Imu boundary_imu;
  // Point-LIO ordering: p, right rotation, extrinsic R/T, v, omega, a, g, bg, ba.
  Eigen::Matrix<double, 30, 30> covariance_output;
  // Input mode ordering: p, right rotation, extrinsic R/T, v, bg, ba, g.
  Eigen::Matrix<double, 24, 24> covariance_input;
  zlio::Vec3 angular_velocity;
  zlio::Vec3 specific_force;
};

// Converts ZLIO SD-group errors to Point-LIO additive/right-rotation errors.
zlio::Mat15 startupErrorJacobian(const zlio::State & state);
StartupSeed makeStartupSeed(zlio::InitializationResult result, const zlio::Imu & boundary,
                           double gyro_variance, double accel_variance);

// Owns a bounded startup window. Inputs include the IMU sample just after scan end;
// exact boundaries are interpolated and a previous boundary is retained across scans.
class JointStartup
{
public:
  explicit JointStartup(JointStartupConfig config);
  void reset();
  std::optional<StartupSeed> add(const zlio::Scan & raw, zlio::Time begin, zlio::Time end,
                                const std::vector<zlio::Imu> & imus);
  const std::string & reason() const {return reason_;}
  const std::string & phase() const {return phase_;}
  std::size_t acceptedPackets() const {return packets_;}
  double computeMilliseconds() const {return compute_ms_;}
private:
  JointStartupConfig config_;
  std::unique_ptr<zlio::Initializer> initializer_;
  std::unique_ptr<zlio::ImuHistory> history_;
  std::optional<zlio::Time> boundary_;
  std::string reason_ = "collecting synchronized scans", phase_ = "COLLECTING";
  std::size_t packets_ = 0;
  double compute_ms_ = 0.0;
};
}  // namespace point_lio
