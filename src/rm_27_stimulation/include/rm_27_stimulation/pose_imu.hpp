#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/imu.pb.h>

namespace rm_27_stimulation {

struct ImuPoseSample {
  std::chrono::nanoseconds time;
  gz::math::Pose3d pose;
};

// Central differences describe the MIDDLE sample, including the acceleration
// of an offset sensor as its parent joint rotates. All outputs use sensor axes.
inline std::optional<gz::msgs::IMU> MakePoseImu(
    ImuPoseSample before, ImuPoseSample middle, ImuPoseSample after,
    const gz::math::Vector3d &gravity, const std::string &frame) {
  if (before.time >= middle.time || middle.time >= after.time ||
      middle.time.count() < 0 || !gravity.IsFinite()) {
    return std::nullopt;
  }
  for (auto *sample : {&before, &middle, &after}) {
    const auto &q = sample->pose.Rot();
    const double norm2 = q.W()*q.W() + q.X()*q.X() + q.Y()*q.Y() + q.Z()*q.Z();
    if (!sample->pose.IsFinite() || norm2 < 1e-12) {
      return std::nullopt;
    }
    sample->pose.Rot().Normalize();
  }
  const double dt0 = std::chrono::duration<double>(middle.time - before.time).count();
  const double dt1 = std::chrono::duration<double>(after.time - middle.time).count();
  const auto acceleration = 2.0 / (dt0 + dt1) *
      ((after.pose.Pos() - middle.pose.Pos()) / dt1 -
       (middle.pose.Pos() - before.pose.Pos()) / dt0);

  auto delta = after.pose.Rot() * before.pose.Rot().Inverse();
  delta.Normalize();
  // q and -q describe the same rotation. Choose the short arc at +/- pi.
  if (delta.W() < 0.0) {
    delta = gz::math::Quaterniond(-delta.W(), -delta.X(), -delta.Y(), -delta.Z());
  }
  const gz::math::Vector3d vector(delta.X(), delta.Y(), delta.Z());
  const double sine = vector.Length();
  const auto omega = vector * (sine < 1e-12 ? 2.0 :
      2.0 * std::atan2(sine, delta.W()) / sine) / (dt0 + dt1);

  gz::msgs::IMU message;
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(middle.time);
  message.mutable_header()->mutable_stamp()->set_sec(seconds.count());
  message.mutable_header()->mutable_stamp()->set_nsec((middle.time - seconds).count());
  auto *id = message.mutable_header()->add_data();
  id->set_key("frame_id");
  id->add_value(frame);
  gz::msgs::Set(message.mutable_orientation(), middle.pose.Rot());
  gz::msgs::Set(message.mutable_angular_velocity(),
               middle.pose.Rot().RotateVectorReverse(omega));
  gz::msgs::Set(message.mutable_linear_acceleration(),
               middle.pose.Rot().RotateVectorReverse(acceleration - gravity));
  return message;
}

} // namespace rm_27_stimulation
