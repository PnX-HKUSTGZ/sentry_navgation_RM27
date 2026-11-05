#pragma once
#include "common_lib.h"
#include "joint_startup.hpp"

namespace point_lio
{
// All state and covariance values refer to exactly the same scan-end timestamp.
inline void installStartupSeed(const StartupSeed & seed,
                              const Eigen::Matrix3d & lidar_rotation,
                              const Eigen::Vector3d & lidar_translation,
                              esekfom::esekf<state_output, 30, input_ikfom> & output,
                              esekfom::esekf<state_input, 24, input_ikfom> & input)
{
  const auto & x = seed.solution.estimate.state;
  state_output out;
  out.pos = x.pose.position;
  out.rot = x.pose.rotation.toRotationMatrix();
  out.vel = x.velocity;
  out.bg = x.gyro_bias;
  out.ba = x.accel_bias;
  out.omg = seed.angular_velocity;
  out.acc = seed.specific_force;
  out.gravity = seed.solution.gravity;
  out.offset_R_L_I = lidar_rotation;
  out.offset_T_L_I = lidar_translation;
  output.change_x(out);
  auto P_out = seed.covariance_output;
  output.change_P(P_out);
  state_input in;
  in.pos = out.pos;
  in.rot = out.rot;
  in.vel = out.vel;
  in.bg = out.bg;
  in.ba = out.ba;
  in.gravity = out.gravity;
  in.offset_R_L_I = lidar_rotation;
  in.offset_T_L_I = lidar_translation;
  input.change_x(in);
  auto P_in = seed.covariance_input;
  input.change_P(P_in);
}
}  // namespace point_lio
