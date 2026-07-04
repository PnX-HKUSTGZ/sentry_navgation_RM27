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

#ifndef SCAN_CONTEXT__SCAN_CONTEXT_MANAGER_HPP_
#define SCAN_CONTEXT__SCAN_CONTEXT_MANAGER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "scan_context/scan_context_types.hpp"

namespace scan_context
{

class ScanContextManager
{
public:
  explicit ScanContextManager(const ScanContextParams & params);
  ~ScanContextManager();

  const ScanContextParams & params() const;
  std::size_t size() const;
  void clear();

  ScanContextDescriptor makeDescriptor(const pcl::PointCloud<pcl::PointXYZ> & cloud) const;

  bool addKeyframe(
    const pcl::PointCloud<pcl::PointXYZ> & cloud_in_base, const Eigen::Isometry3d & map_to_base,
    std::int64_t stamp_nanoseconds);

  bool addKeyframeDescriptor(
    const ScanContextDescriptor & descriptor, std::size_t point_count,
    const Eigen::Isometry3d & map_to_base, std::int64_t stamp_nanoseconds,
    bool rebuild_index = true);

  void rebuildIndex();

  std::vector<ScanContextCandidate> query(
    const pcl::PointCloud<pcl::PointXYZ> & cloud_in_base) const;

  bool loadDatabase(const std::string & path);
  bool saveDatabase(const std::string & path) const;

private:
  struct RingKeyIndex;

  void rebuildRingKeyIndex();
  std::vector<std::size_t> findNearestRingKeys(
    const Eigen::VectorXf & query_ring_key, int max_candidates) const;
  Eigen::VectorXf makeRingKey(const Eigen::MatrixXf & descriptor) const;
  double ringKeyDistance(const Eigen::VectorXf & lhs, const Eigen::VectorXf & rhs) const;
  double descriptorDistance(
    const Eigen::MatrixXf & reference, const Eigen::MatrixXf & query, int sector_shift) const;
  ScanContextCandidate makeCandidate(
    const ScanContextKeyframe & keyframe, const ScanContextDescriptor & query_descriptor) const;

  ScanContextParams params_;
  std::vector<ScanContextKeyframe> keyframes_;
  std::unique_ptr<RingKeyIndex> ring_key_index_;
};

}  // namespace scan_context

#endif  // SCAN_CONTEXT__SCAN_CONTEXT_MANAGER_HPP_
