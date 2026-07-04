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

#include "scan_context/scan_context_manager.hpp"

#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <flann/flann.hpp>
#include <fstream>
#include <limits>
#include <string>
#include <utility>

namespace scan_context
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kDatabaseVersion = 2;
constexpr int kMaxDatabaseKeyframes = 100000;
constexpr float kEmptyBinValue = -std::numeric_limits<float>::max();
constexpr char kDatabaseMagic[] = "SCDB";
constexpr std::size_t kDatabaseMagicSize = 4;

bool hasValidDescriptorParams(const ScanContextParams & params)
{
  return params.num_rings > 0 && params.num_sectors > 0 && std::isfinite(params.max_radius) &&
         std::isfinite(params.min_height) && std::isfinite(params.max_height) &&
         params.max_radius > 0.0 && params.min_height <= params.max_height;
}

bool hasEnoughPoints(std::size_t point_count, int min_points)
{
  return min_points <= 0 || point_count >= static_cast<std::size_t>(min_points);
}

bool isFinitePoint(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isFiniteValues(const double * values, std::size_t size)
{
  for (std::size_t i = 0; i < size; ++i) {
    if (!std::isfinite(values[i])) {
      return false;
    }
  }
  return true;
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

template <typename T>
void writeValue(std::ofstream & stream, const T & value)
{
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template <typename T>
bool readValue(std::ifstream & stream, T & value)
{
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

void writeMatrix(std::ofstream & stream, const Eigen::MatrixXf & matrix)
{
  stream.write(
    reinterpret_cast<const char *>(matrix.data()),
    static_cast<std::streamsize>(sizeof(float) * matrix.size()));
}

bool readMatrix(std::ifstream & stream, Eigen::MatrixXf & matrix)
{
  stream.read(
    reinterpret_cast<char *>(matrix.data()),
    static_cast<std::streamsize>(sizeof(float) * matrix.size()));
  return static_cast<bool>(stream);
}

bool isDirectory(const std::string & path)
{
  struct stat path_stat;
  if (stat(path.c_str(), &path_stat) != 0) {
    return false;
  }
  return S_ISDIR(path_stat.st_mode);
}

bool createDirectories(const std::string & directory)
{
  if (directory.empty()) {
    return true;
  }

  std::size_t current_position = directory[0] == '/' ? 1U : 0U;
  while (current_position <= directory.size()) {
    const std::size_t next_separator = directory.find('/', current_position);
    const std::string current_directory = directory.substr(0, next_separator);
    if (!current_directory.empty()) {
      errno = 0;
      if (mkdir(current_directory.c_str(), 0755) != 0) {
        if (errno != EEXIST || !isDirectory(current_directory)) {
          return false;
        }
      }
    }
    if (next_separator == std::string::npos) {
      break;
    }
    current_position = next_separator + 1U;
  }

  return true;
}

bool createParentDirectories(const std::string & file_path)
{
  const std::size_t last_separator = file_path.find_last_of('/');
  if (last_separator == std::string::npos || last_separator == 0U) {
    return true;
  }
  return createDirectories(file_path.substr(0, last_separator));
}

}  // namespace

struct ScanContextManager::RingKeyIndex
{
  std::vector<float> dataset;
  std::unique_ptr<flann::Index<flann::L2<float>>> index;
};

ScanContextManager::ScanContextManager(const ScanContextParams & params) : params_(params) {}

ScanContextManager::~ScanContextManager() = default;

const ScanContextParams & ScanContextManager::params() const { return params_; }

std::size_t ScanContextManager::size() const { return keyframes_.size(); }

void ScanContextManager::clear()
{
  keyframes_.clear();
  ring_key_index_.reset();
}

ScanContextDescriptor ScanContextManager::makeDescriptor(
  const pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
  ScanContextDescriptor descriptor;
  if (!hasValidDescriptorParams(params_)) {
    descriptor.scan_context = Eigen::MatrixXf();
    descriptor.ring_key = Eigen::VectorXf();
    return descriptor;
  }

  descriptor.scan_context =
    Eigen::MatrixXf::Constant(params_.num_rings, params_.num_sectors, kEmptyBinValue);

  const double ring_step = params_.max_radius / static_cast<double>(params_.num_rings);
  const double sector_step = 2.0 * kPi / static_cast<double>(params_.num_sectors);
  int occupied_bins = 0;

  for (const auto & point : cloud.points) {
    if (!isFinitePoint(point)) {
      continue;
    }
    if (point.z < params_.min_height || point.z > params_.max_height) {
      continue;
    }

    const double radius = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (radius <= 0.0 || radius > params_.max_radius) {
      continue;
    }

    int ring_index = static_cast<int>(std::floor(radius / ring_step));
    ring_index = std::max(0, std::min(params_.num_rings - 1, ring_index));

    double angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
    if (angle < 0.0) {
      angle += 2.0 * kPi;
    }
    int sector_index = static_cast<int>(std::floor(angle / sector_step));
    sector_index = std::max(0, std::min(params_.num_sectors - 1, sector_index));

    const float height = static_cast<float>(point.z - params_.min_height);
    float & bin = descriptor.scan_context(ring_index, sector_index);
    if (bin == kEmptyBinValue) {
      ++occupied_bins;
    }
    if (height > bin) {
      bin = height;
    }
  }

  if (occupied_bins == 0) {
    descriptor.scan_context = Eigen::MatrixXf();
    descriptor.ring_key = Eigen::VectorXf();
    return descriptor;
  }

  for (int row = 0; row < descriptor.scan_context.rows(); ++row) {
    for (int col = 0; col < descriptor.scan_context.cols(); ++col) {
      if (descriptor.scan_context(row, col) == kEmptyBinValue) {
        descriptor.scan_context(row, col) = 0.0F;
      }
    }
  }

  descriptor.ring_key = makeRingKey(descriptor.scan_context);
  return descriptor;
}

bool ScanContextManager::addKeyframe(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in_base, const Eigen::Isometry3d & map_to_base,
  std::int64_t stamp_nanoseconds)
{
  return addKeyframeDescriptor(
    makeDescriptor(cloud_in_base), cloud_in_base.size(), map_to_base, stamp_nanoseconds);
}

bool ScanContextManager::addKeyframeDescriptor(
  const ScanContextDescriptor & descriptor, std::size_t point_count,
  const Eigen::Isometry3d & map_to_base, std::int64_t stamp_nanoseconds, bool rebuild_index)
{
  if (!hasEnoughPoints(point_count, params_.min_points)) {
    return false;
  }
  if (
    descriptor.scan_context.rows() != params_.num_rings ||
    descriptor.scan_context.cols() != params_.num_sectors ||
    descriptor.ring_key.size() != params_.num_rings || !descriptor.scan_context.allFinite() ||
    !descriptor.ring_key.allFinite() || !map_to_base.matrix().allFinite()) {
    return false;
  }

  ScanContextKeyframe keyframe;
  keyframe.id = static_cast<int>(keyframes_.size());
  keyframe.stamp_nanoseconds = stamp_nanoseconds;
  keyframe.map_to_base = map_to_base;
  keyframe.descriptor = descriptor;

  keyframes_.push_back(keyframe);
  if (rebuild_index) {
    rebuildRingKeyIndex();
  } else {
    ring_key_index_.reset();
  }
  return true;
}

void ScanContextManager::rebuildIndex() { rebuildRingKeyIndex(); }

std::vector<ScanContextCandidate> ScanContextManager::query(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in_base) const
{
  std::vector<ScanContextCandidate> candidates;
  if (
    keyframes_.empty() || !hasEnoughPoints(cloud_in_base.size(), params_.min_points) ||
    params_.num_candidates <= 0) {
    return candidates;
  }

  const ScanContextDescriptor query_descriptor = makeDescriptor(cloud_in_base);
  if (query_descriptor.scan_context.size() == 0) {
    return candidates;
  }

  const int max_candidates =
    std::max(1, std::min(params_.num_candidates, static_cast<int>(keyframes_.size())));
  const std::vector<std::size_t> nearest_indices =
    findNearestRingKeys(query_descriptor.ring_key, max_candidates);

  for (const std::size_t keyframe_index : nearest_indices) {
    const auto & keyframe = keyframes_[keyframe_index];
    ScanContextCandidate candidate = makeCandidate(keyframe, query_descriptor);
    if (candidate.score <= params_.score_threshold) {
      candidates.push_back(candidate);
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const ScanContextCandidate & lhs, const ScanContextCandidate & rhs) {
      return lhs.score < rhs.score;
    });
  return candidates;
}

bool ScanContextManager::loadDatabase(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    return false;
  }

  char magic[kDatabaseMagicSize];
  stream.read(magic, static_cast<std::streamsize>(kDatabaseMagicSize));
  if (!stream || !std::equal(magic, magic + kDatabaseMagicSize, kDatabaseMagic)) {
    return false;
  }

  int version = 0;
  int num_rings = 0;
  int num_sectors = 0;
  int keyframe_count = 0;
  if (
    !readValue(stream, version) || !readValue(stream, num_rings) ||
    !readValue(stream, num_sectors) || !readValue(stream, keyframe_count)) {
    return false;
  }

  if (
    version != kDatabaseVersion || num_rings != params_.num_rings ||
    num_sectors != params_.num_sectors || keyframe_count < 0 ||
    keyframe_count > kMaxDatabaseKeyframes) {
    return false;
  }

  std::vector<ScanContextKeyframe> loaded_keyframes;
  loaded_keyframes.reserve(static_cast<std::size_t>(keyframe_count));

  for (int i = 0; i < keyframe_count; ++i) {
    ScanContextKeyframe keyframe;
    double translation[3] = {0.0, 0.0, 0.0};
    double quaternion_xyzw[4] = {0.0, 0.0, 0.0, 1.0};

    if (
      !readValue(stream, keyframe.id) || !readValue(stream, keyframe.stamp_nanoseconds) ||
      !readValue(stream, translation[0]) || !readValue(stream, translation[1]) ||
      !readValue(stream, translation[2]) || !readValue(stream, quaternion_xyzw[0]) ||
      !readValue(stream, quaternion_xyzw[1]) || !readValue(stream, quaternion_xyzw[2]) ||
      !readValue(stream, quaternion_xyzw[3])) {
      return false;
    }

    const double quaternion_norm = std::sqrt(
      quaternion_xyzw[0] * quaternion_xyzw[0] + quaternion_xyzw[1] * quaternion_xyzw[1] +
      quaternion_xyzw[2] * quaternion_xyzw[2] + quaternion_xyzw[3] * quaternion_xyzw[3]);
    if (
      !isFiniteValues(translation, 3) || !isFiniteValues(quaternion_xyzw, 4) ||
      quaternion_norm < 1e-9) {
      return false;
    }

    keyframe.map_to_base = Eigen::Isometry3d::Identity();
    keyframe.map_to_base.translation() =
      Eigen::Vector3d(translation[0], translation[1], translation[2]);
    keyframe.map_to_base.linear() =
      Eigen::Quaterniond(
        quaternion_xyzw[3], quaternion_xyzw[0], quaternion_xyzw[1], quaternion_xyzw[2])
        .normalized()
        .toRotationMatrix();

    keyframe.descriptor.scan_context.resize(num_rings, num_sectors);
    if (
      !readMatrix(stream, keyframe.descriptor.scan_context) ||
      !keyframe.descriptor.scan_context.allFinite()) {
      return false;
    }
    keyframe.descriptor.ring_key = makeRingKey(keyframe.descriptor.scan_context);
    loaded_keyframes.push_back(keyframe);
  }

  keyframes_ = loaded_keyframes;
  rebuildRingKeyIndex();
  return true;
}

bool ScanContextManager::saveDatabase(const std::string & path) const
{
  if (path.empty()) {
    return false;
  }

  if (keyframes_.size() > static_cast<std::size_t>(kMaxDatabaseKeyframes)) {
    return false;
  }

  for (const auto & keyframe : keyframes_) {
    if (
      keyframe.descriptor.scan_context.rows() != params_.num_rings ||
      keyframe.descriptor.scan_context.cols() != params_.num_sectors ||
      !keyframe.descriptor.scan_context.allFinite() || !keyframe.map_to_base.matrix().allFinite()) {
      return false;
    }
  }

  if (!createParentDirectories(path)) {
    return false;
  }

  const std::string temp_path = path + ".tmp";
  std::ofstream stream(temp_path, std::ios::binary | std::ios::trunc);
  if (!stream.is_open()) {
    return false;
  }

  stream.write(kDatabaseMagic, static_cast<std::streamsize>(kDatabaseMagicSize));
  writeValue(stream, kDatabaseVersion);
  writeValue(stream, params_.num_rings);
  writeValue(stream, params_.num_sectors);
  const int keyframe_count = static_cast<int>(keyframes_.size());
  writeValue(stream, keyframe_count);

  for (const auto & keyframe : keyframes_) {
    const Eigen::Vector3d translation = keyframe.map_to_base.translation();
    const Eigen::Quaterniond rotation(keyframe.map_to_base.rotation());

    writeValue(stream, keyframe.id);
    writeValue(stream, keyframe.stamp_nanoseconds);
    writeValue(stream, translation.x());
    writeValue(stream, translation.y());
    writeValue(stream, translation.z());
    writeValue(stream, rotation.x());
    writeValue(stream, rotation.y());
    writeValue(stream, rotation.z());
    writeValue(stream, rotation.w());
    writeMatrix(stream, keyframe.descriptor.scan_context);
  }

  stream.close();
  if (!stream) {
    std::remove(temp_path.c_str());
    return false;
  }

  if (std::rename(temp_path.c_str(), path.c_str()) != 0) {
    std::remove(temp_path.c_str());
    return false;
  }

  return true;
}

void ScanContextManager::rebuildRingKeyIndex()
{
  ring_key_index_.reset();

  if (keyframes_.empty() || params_.num_rings <= 0) {
    return;
  }

  std::unique_ptr<RingKeyIndex> new_index(new RingKeyIndex());
  new_index->dataset.resize(keyframes_.size() * static_cast<std::size_t>(params_.num_rings));
  for (std::size_t keyframe_index = 0; keyframe_index < keyframes_.size(); ++keyframe_index) {
    const Eigen::VectorXf & ring_key = keyframes_[keyframe_index].descriptor.ring_key;
    if (ring_key.size() != params_.num_rings) {
      return;
    }

    for (int dim = 0; dim < params_.num_rings; ++dim) {
      new_index->dataset
        [keyframe_index * static_cast<std::size_t>(params_.num_rings) +
         static_cast<std::size_t>(dim)] = ring_key(dim);
    }
  }

  flann::Matrix<float> dataset(
    new_index->dataset.data(), keyframes_.size(), static_cast<std::size_t>(params_.num_rings));
  try {
    new_index->index.reset(
      new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(4)));
    new_index->index->buildIndex();
    ring_key_index_ = std::move(new_index);
  } catch (...) {
    ring_key_index_.reset();
  }
}

std::vector<std::size_t> ScanContextManager::findNearestRingKeys(
  const Eigen::VectorXf & query_ring_key, int max_candidates) const
{
  std::vector<std::size_t> nearest_indices;
  if (query_ring_key.size() != params_.num_rings || max_candidates <= 0 || keyframes_.empty()) {
    return nearest_indices;
  }

  const int bounded_candidates = std::min(max_candidates, static_cast<int>(keyframes_.size()));
  nearest_indices.reserve(static_cast<std::size_t>(bounded_candidates));

  const auto find_linear_nearest = [this, &query_ring_key, bounded_candidates]() {
    std::vector<std::pair<double, std::size_t>> ring_distances;
    ring_distances.reserve(keyframes_.size());
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      ring_distances.emplace_back(
        ringKeyDistance(keyframes_[i].descriptor.ring_key, query_ring_key), i);
    }
    std::sort(ring_distances.begin(), ring_distances.end());

    std::vector<std::size_t> linear_indices;
    linear_indices.reserve(static_cast<std::size_t>(bounded_candidates));
    for (int i = 0; i < bounded_candidates; ++i) {
      linear_indices.push_back(ring_distances[static_cast<std::size_t>(i)].second);
    }
    return linear_indices;
  };

  if (ring_key_index_ && ring_key_index_->index) {
    try {
      std::vector<float> query_data(static_cast<std::size_t>(params_.num_rings));
      for (int dim = 0; dim < params_.num_rings; ++dim) {
        query_data[static_cast<std::size_t>(dim)] = query_ring_key(dim);
      }

      std::vector<std::size_t> indices(static_cast<std::size_t>(bounded_candidates));
      std::vector<float> distances(static_cast<std::size_t>(bounded_candidates));
      flann::Matrix<float> query_matrix(
        query_data.data(), 1U, static_cast<std::size_t>(params_.num_rings));
      flann::Matrix<std::size_t> indices_matrix(
        indices.data(), 1U, static_cast<std::size_t>(bounded_candidates));
      flann::Matrix<float> distances_matrix(
        distances.data(), 1U, static_cast<std::size_t>(bounded_candidates));

      const int found_count = ring_key_index_->index->knnSearch(
        query_matrix, indices_matrix, distances_matrix,
        static_cast<std::size_t>(bounded_candidates),
        flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
      for (int i = 0; i < found_count; ++i) {
        if (indices[static_cast<std::size_t>(i)] < keyframes_.size()) {
          nearest_indices.push_back(indices[static_cast<std::size_t>(i)]);
        }
      }
      if (!nearest_indices.empty()) {
        return nearest_indices;
      }
    } catch (...) {
      return find_linear_nearest();
    }
  }

  return find_linear_nearest();
}

Eigen::VectorXf ScanContextManager::makeRingKey(const Eigen::MatrixXf & descriptor) const
{
  Eigen::VectorXf ring_key(descriptor.rows());
  for (int row = 0; row < descriptor.rows(); ++row) {
    ring_key(row) = descriptor.row(row).mean();
  }
  return ring_key;
}

double ScanContextManager::ringKeyDistance(
  const Eigen::VectorXf & lhs, const Eigen::VectorXf & rhs) const
{
  if (lhs.size() == 0 || lhs.size() != rhs.size()) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>((lhs - rhs).norm());
}

double ScanContextManager::descriptorDistance(
  const Eigen::MatrixXf & reference, const Eigen::MatrixXf & query, int sector_shift) const
{
  if (
    reference.rows() != query.rows() || reference.cols() != query.cols() || reference.size() == 0) {
    return std::numeric_limits<double>::infinity();
  }

  double distance_sum = 0.0;
  int valid_columns = 0;

  for (int col = 0; col < reference.cols(); ++col) {
    const int shifted_col = (col + sector_shift + reference.cols()) % reference.cols();
    const Eigen::VectorXf reference_column = reference.col(shifted_col);
    const Eigen::VectorXf query_column = query.col(col);
    const double reference_norm = static_cast<double>(reference_column.norm());
    const double query_norm = static_cast<double>(query_column.norm());
    if (reference_norm < 1e-6 || query_norm < 1e-6) {
      continue;
    }

    const double similarity =
      static_cast<double>(reference_column.dot(query_column)) / (reference_norm * query_norm);
    distance_sum += 1.0 - similarity;
    ++valid_columns;
  }

  if (valid_columns == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return distance_sum / static_cast<double>(valid_columns);
}

ScanContextCandidate ScanContextManager::makeCandidate(
  const ScanContextKeyframe & keyframe, const ScanContextDescriptor & query_descriptor) const
{
  ScanContextCandidate candidate;
  candidate.keyframe_id = keyframe.id;
  candidate.stamp_nanoseconds = keyframe.stamp_nanoseconds;
  candidate.map_to_base = keyframe.map_to_base;

  double best_score = std::numeric_limits<double>::infinity();
  int best_shift = 0;
  for (int shift = 0; shift < params_.num_sectors; ++shift) {
    const double score =
      descriptorDistance(keyframe.descriptor.scan_context, query_descriptor.scan_context, shift);
    if (score < best_score) {
      best_score = score;
      best_shift = shift;
    }
  }

  candidate.score = best_score;
  candidate.yaw_delta =
    normalizeAngle(-static_cast<double>(best_shift) * 2.0 * kPi / params_.num_sectors);
  return candidate;
}

}  // namespace scan_context
