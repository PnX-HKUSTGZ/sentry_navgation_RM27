#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/odometry.pb.h>

namespace rm_27_stimulation {

inline std::optional<gz::msgs::Odometry> MakeGroundTruthOdometry(
    gz::math::Pose3d pose, const gz::math::Vector3d &worldLinear,
    const gz::math::Vector3d &worldAngular, std::chrono::nanoseconds time,
    const std::string &odomFrame, const std::string &baseFrame) {
  const auto &q = pose.Rot();
  const auto norm2 = q.W() * q.W() + q.X() * q.X() + q.Y() * q.Y() +
                     q.Z() * q.Z();
  if (!pose.IsFinite() || !worldLinear.IsFinite() || !worldAngular.IsFinite() ||
      !std::isfinite(norm2) || norm2 < 1e-12 || time.count() < 0) {
    return std::nullopt;
  }

  pose.Rot().Normalize();
  gz::msgs::Odometry message;
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time);
  message.mutable_header()->mutable_stamp()->set_sec(seconds.count());
  message.mutable_header()->mutable_stamp()->set_nsec((time - seconds).count());
  auto *frame = message.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(odomFrame);
  auto *child = message.mutable_header()->add_data();
  child->set_key("child_frame_id");
  child->add_value(baseFrame);
  gz::msgs::Set(message.mutable_pose(), pose);
  gz::msgs::Set(message.mutable_twist()->mutable_linear(),
                pose.Rot().RotateVectorReverse(worldLinear));
  gz::msgs::Set(message.mutable_twist()->mutable_angular(),
                pose.Rot().RotateVectorReverse(worldAngular));
  return message;
}

} // namespace rm_27_stimulation
