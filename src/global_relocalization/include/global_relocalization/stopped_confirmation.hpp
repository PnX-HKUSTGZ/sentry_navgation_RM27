// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <cmath>
#include <cstddef>

namespace global_relocalization {
enum class StoppedScanState { Moving, Settling, Ready };
// Called for every processed scan, before a later result can hide its motion.
// Both elapsed wall time and acquisition time must cover the stopped window.
inline StoppedScanState observeStoppedScan(double linear_speed,double angular_speed,
  std::chrono::steady_clock::time_point wall_now,double ros_now,double scan_stamp,
  double settle_sec,std::chrono::steady_clock::time_point & stable_wall_start,
  double & stable_ros_start) {
  if(!std::isfinite(linear_speed)||!std::isfinite(angular_speed)||linear_speed>.04||angular_speed>.06) {
    stable_wall_start=wall_now;stable_ros_start=ros_now;return StoppedScanState::Moving;
  }
  if(std::chrono::duration<double>(wall_now-stable_wall_start).count()<settle_sec||
    !std::isfinite(scan_stamp)||scan_stamp<stable_ros_start+settle_sec)return StoppedScanState::Settling;
  return StoppedScanState::Ready;
}
// Freeze may drain results that invalidate the earlier pre-freeze decision.
inline bool stoppedConfirmationSnapshotReady(bool canceled,bool stopping,size_t scans,
  size_t required,StoppedScanState state) {
  return !canceled&&stopping&&required>=3&&scans>=required&&state==StoppedScanState::Ready;
}
}
