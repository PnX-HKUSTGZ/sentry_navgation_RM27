// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SENSOR_SCAN_GENERATION__TRANSFORM_SAMPLE_GATE_HPP_
#define SENSOR_SCAN_GENERATION__TRANSFORM_SAMPLE_GATE_HPP_

#include <cstdint>

namespace sensor_scan_generation
{

struct TransformSampleDecision
{
  bool publish_sample{false};
  bool reset_twist_history{true};
  bool recovered{false};
};

// Keeps publication and finite-difference history atomic with respect to the
// transforms needed by one synchronized cloud/odometry sample.
class TransformSampleGate
{
public:
  TransformSampleDecision evaluate(
    bool lidar_to_robot_base_available, bool lidar_to_chassis_available)
  {
    if (!lidar_to_robot_base_available || !lidar_to_chassis_available) {
      state_ = State::BLOCKED;
      return TransformSampleDecision{};
    }

    const bool recovered = state_ == State::BLOCKED;
    const bool cold_start = state_ != State::TRACKING;
    state_ = State::TRACKING;
    return TransformSampleDecision{true, cold_start, recovered};
  }

private:
  enum class State : std::uint8_t { COLD, TRACKING, BLOCKED };

  State state_{State::COLD};
};

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__TRANSFORM_SAMPLE_GATE_HPP_
