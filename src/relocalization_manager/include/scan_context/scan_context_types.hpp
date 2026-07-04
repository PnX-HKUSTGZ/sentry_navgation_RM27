// Copyright 2025 Lihan Chen
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

#ifndef SCAN_CONTEXT__SCAN_CONTEXT_TYPES_HPP_
#define SCAN_CONTEXT__SCAN_CONTEXT_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>

namespace scan_context
{

struct ScanContextParams
{
  std::string mode{"off"};
  std::string database_path;
  std::string input_frame{"base_footprint"};
  std::string build_source{"live"};
  double update_interval{0.1};

  int num_rings{20};
  int num_sectors{60};
  double max_radius{25.0};
  double min_height{-1.0};
  double max_height{3.0};

  int num_candidates{10};
  double score_threshold{0.15};

  double keyframe_min_translation{1.0};
  double keyframe_min_yaw{0.17453292519943295};
  double keyframe_min_interval{1.0};
  int min_points{500};

  double prior_sample_resolution{1.0};
  double prior_leaf_size{0.25};
};

struct ScanContextDescriptor
{
  Eigen::MatrixXf scan_context;
  Eigen::VectorXf ring_key;
};

struct ScanContextKeyframe
{
  int id{-1};
  std::int64_t stamp_nanoseconds{0};
  Eigen::Isometry3d map_to_base{Eigen::Isometry3d::Identity()};
  ScanContextDescriptor descriptor;
};

struct ScanContextCandidate
{
  int keyframe_id{-1};
  std::int64_t stamp_nanoseconds{0};
  Eigen::Isometry3d map_to_base{Eigen::Isometry3d::Identity()};
  double yaw_delta{0.0};
  double score{1.0};
};

}  // namespace scan_context

#endif  // SCAN_CONTEXT__SCAN_CONTEXT_TYPES_HPP_
