#pragma once
#include <algorithm>
#include <random>
#include <string>
#include "joint_startup.hpp"
#include "zlio/state.hpp"

namespace startup_test
{
struct Scenario
{
  std::string name;
  bool moving = false, tilted = false, plane_only = false, snapshot = false;
  double noise = 0.0;
  zlio::Vec3 gyro_bias = zlio::Vec3::Zero(), accel_bias = zlio::Vec3::Zero();
  zlio::Pose extrinsic;
};
inline std::vector<Scenario> scenarios()
{
  std::vector<Scenario> list;
  list.push_back({"static_level"});
  Scenario tilted{"static_tilted"}; tilted.tilted = true; list.push_back(tilted);
  Scenario biased{"static_bias_noise"}; biased.tilted = true; biased.noise = 1;
  biased.gyro_bias = {0.012, -0.008, 0.01}; biased.accel_bias = {0.06, -0.04, 0.08};
  list.push_back(biased);
  Scenario moving = biased; moving.name = "dynamic_sweep"; moving.moving = true; list.push_back(moving);
  moving.name = "dynamic_snapshot"; moving.snapshot = true; list.push_back(moving);
  moving.name = "dynamic_extrinsics"; moving.snapshot = false;
  moving.extrinsic.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(.18, zlio::Vec3::UnitY()));
  moving.extrinsic.position = {-0.011, -0.02329, 0.04412}; list.push_back(moving);
  Scenario plane{"degenerate_plane"}; plane.plane_only = true; list.push_back(plane);
  return list;
}
inline zlio::State truth(const Scenario & s, double t)
{
  zlio::State x;
  double yaw = 0, pitch = s.tilted ? .18 : 0, roll = s.tilted ? -.5235987756 : 0;
  if (s.moving) {
    yaw = .35 * t + .12 * std::sin(1.3 * t);
    pitch += .15 * std::sin(.9 * t);
    roll += .12 * std::sin(1.1 * t);
    x.pose.position = {.25 * t + .4 * std::sin(1.1 * t), .2 * (1 - std::cos(.8 * t)),
                       .08 * std::sin(.6 * t)};
    x.velocity = {.25 + .44 * std::cos(1.1 * t), .16 * std::sin(.8 * t),
                  .048 * std::cos(.6 * t)};
  }
  x.pose.rotation = Eigen::AngleAxisd(yaw, zlio::Vec3::UnitZ()) *
                    Eigen::AngleAxisd(pitch, zlio::Vec3::UnitY()) *
                    Eigen::AngleAxisd(roll, zlio::Vec3::UnitX());
  return x;
}
inline zlio::Vec3 acceleration(const Scenario & s, double t)
{
  if (!s.moving) {return zlio::Vec3::Zero();}
  return {-.484 * std::sin(1.1 * t), .128 * std::cos(.8 * t), -.0288 * std::sin(.6 * t)};
}
constexpr double epoch = 100.0;
struct Packet
{
  zlio::Scan scan;
  std::vector<zlio::Imu> imus;
  zlio::Time begin, end;
};
inline Packet packet(const Scenario & s, int frame, unsigned seed)
{
  std::mt19937 generator(seed * 10000 + frame);
  std::normal_distribution<double> noise(0.0, 1.0);
  const double begin = frame * .1, end = (frame + 1) * .1;
  Packet p;
  p.begin = zlio::nanoseconds(epoch + (s.snapshot ? end : begin));
  p.end = zlio::nanoseconds(epoch + end);
  for (int j = 0; j <= 20; ++j) {
    const double t = begin + j * .005;
    const auto x = truth(s, t);
    zlio::Imu imu;
    imu.time = zlio::nanoseconds(epoch + t);
    const double h = 1e-5;
    imu.angular_velocity = zlio::logRotation(
      truth(s, t-h).pose.rotation.conjugate() * truth(s, t+h).pose.rotation) / (2*h) + s.gyro_bias;
    imu.acceleration = x.pose.rotation.conjugate() *
      (acceleration(s, t) + zlio::Vec3(0, 0, 9.81)) + s.accel_bias;
    for (int k = 0; k < 3; ++k) {
      imu.angular_velocity[k] += s.noise * .002 * noise(generator);
      imu.acceleration[k] += s.noise * .015 * noise(generator);
    }
    p.imus.push_back(imu);
  }
  std::vector<zlio::Vec3> world;
  for (int i = -20; i <= 20; ++i) {
    for (int j = -20; j <= 20; ++j) {
      const double a = .1*i, b = .1*j;
      if (!s.plane_only) {world.emplace_back(4,a,b); world.emplace_back(a,4,b);}
      world.emplace_back(a,b,-2);
    }
  }
  for (std::size_t i = 0; i < world.size(); ++i) {
    const double t = s.snapshot ? end : begin + .1 * (i + 1.0) / world.size();
    const auto x = truth(s,t);
    zlio::Vec3 raw = s.extrinsic.inverseApply(x.pose.inverseApply(world[i]));
    for (int k = 0; k < 3; ++k) {raw[k] += s.noise * .005 * noise(generator);}
    p.scan.push_back({raw, zlio::nanoseconds(epoch+t), i});
  }
  return p;
}
inline point_lio::JointStartupConfig config(const Scenario & s)
{
  point_lio::JointStartupConfig c;
  c.solver.body_from_lidar = s.extrinsic;
  c.solver.initialization.optimization.thread_count = 2;
  c.solver.noise.gyro_sample_variance = .1;
  c.solver.noise.accel_sample_variance = .1;
  return c;
}
inline double tiltError(const zlio::State & estimate, const zlio::State & actual)
{
  const auto a = estimate.pose.rotation.conjugate() * zlio::Vec3::UnitZ();
  const auto b = actual.pose.rotation.conjugate() * zlio::Vec3::UnitZ();
  return std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * 180 / M_PI;
}
inline double speedError(const zlio::State & estimate, const zlio::State & actual)
{
  return (estimate.pose.rotation.conjugate() * estimate.velocity -
          actual.pose.rotation.conjugate() * actual.velocity).norm();
}
inline double mapRmse(const std::vector<zlio::MapPoint> & map, const zlio::State & estimate,
                      const zlio::State & actual)
{
  double sum = 0;
  for (const auto & point : map) {
    const auto q = actual.pose.apply(estimate.pose.inverseApply(point.point));
    const double distance = std::min({std::abs(q.x()-4), std::abs(q.y()-4), std::abs(q.z()+2)});
    sum += distance * distance;
  }
  return std::sqrt(sum / std::max<std::size_t>(1,map.size()));
}
}  // namespace startup_test
