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

#include "relocalization_manager/teaser_relocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "pcl/common/transforms.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/search/kdtree.h"
#include "teaser/registration.h"

namespace relocalization_manager
{
namespace
{

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelSample(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, double leaf_size)
{
  auto sampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (leaf_size <= 0.0 || !std::isfinite(leaf_size)) {
    *sampled = cloud;
    return sampled;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud.makeShared());
  const float leaf = static_cast<float>(leaf_size);
  voxel_filter.setLeafSize(leaf, leaf, leaf);
  voxel_filter.filter(*sampled);
  return sampled;
}

teaser::PointCloud toTeaserCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  teaser::PointCloud teaser_cloud;
  teaser_cloud.reserve(cloud.size());
  for (const auto & point : cloud.points) {
    teaser_cloud.push_back({point.x, point.y, point.z});
  }
  return teaser_cloud;
}

}  // namespace

TeaserRelocalizer::TeaserRelocalizer(const TeaserParams & params)
: params_(params),
  overlap_map_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
  target_feature_tree_(std::make_shared<pcl::KdTreeFLANN<pcl::FPFHSignature33>>()),
  overlap_tree_(std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>())
{
  target_.points = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  target_.features = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
}

const TeaserParams & TeaserRelocalizer::params() const { return params_; }

std::size_t TeaserRelocalizer::targetFeaturePointCount() const
{
  return target_.points ? target_.points->size() : 0U;
}

bool TeaserRelocalizer::isFiniteFeature(const pcl::FPFHSignature33 & feature)
{
  double magnitude = 0.0;
  for (const float value : feature.histogram) {
    if (!std::isfinite(value)) {
      return false;
    }
    magnitude += std::abs(static_cast<double>(value));
  }
  return magnitude > 1e-6;
}

TeaserRelocalizer::FeatureSet TeaserRelocalizer::extractFeatures(
  const pcl::PointCloud<pcl::PointXYZ> & cloud) const
{
  FeatureSet result;
  result.points = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  result.features = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();

  const auto sampled = voxelSample(cloud, params_.voxel_size);
  if (!sampled || sampled->empty()) {
    return result;
  }

  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  auto search_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setNumberOfThreads(params_.num_threads);
  normal_estimation.setInputCloud(sampled);
  normal_estimation.setSearchMethod(search_tree);
  normal_estimation.setRadiusSearch(params_.normal_radius);
  normal_estimation.compute(*normals);

  auto features = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
  fpfh_estimation.setNumberOfThreads(params_.num_threads);
  fpfh_estimation.setInputCloud(sampled);
  fpfh_estimation.setInputNormals(normals);
  fpfh_estimation.setSearchMethod(search_tree);
  fpfh_estimation.setRadiusSearch(params_.fpfh_radius);
  fpfh_estimation.compute(*features);

  if (features->size() != sampled->size()) {
    return result;
  }

  result.points->reserve(sampled->size());
  result.features->reserve(features->size());
  for (std::size_t i = 0; i < sampled->size(); ++i) {
    const auto & point = sampled->points[i];
    if (
      !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      !isFiniteFeature(features->points[i])) {
      continue;
    }
    result.points->push_back(point);
    result.features->push_back(features->points[i]);
  }

  result.points->width = result.points->size();
  result.points->height = 1;
  result.points->is_dense = true;
  result.features->width = result.features->size();
  result.features->height = 1;
  result.features->is_dense = true;
  return result;
}

bool TeaserRelocalizer::setGlobalMap(
  const pcl::PointCloud<pcl::PointXYZ> & global_map, std::string & reason)
{
  reason.clear();
  if (global_map.empty()) {
    reason = "global_map_empty";
    return false;
  }

  target_ = extractFeatures(global_map);
  if (
    !target_.points || !target_.features ||
    target_.points->size() < static_cast<std::size_t>(params_.min_feature_points)) {
    reason = "insufficient_target_feature_points";
    return false;
  }
  target_feature_tree_->setInputCloud(target_.features);

  overlap_map_ = voxelSample(global_map, params_.overlap_map_leaf_size);
  if (!overlap_map_ || overlap_map_->empty()) {
    reason = "overlap_map_empty";
    return false;
  }
  overlap_tree_->setInputCloud(overlap_map_);
  return true;
}

std::vector<std::pair<int, int>> TeaserRelocalizer::findCorrespondences(
  const FeatureSet & source) const
{
  std::vector<std::pair<int, int>> correspondences;
  if (
    !source.features || source.features->size() < 2U || !target_.features ||
    target_.features->size() < 2U || !target_feature_tree_) {
    return correspondences;
  }

  pcl::KdTreeFLANN<pcl::FPFHSignature33> source_feature_tree;
  if (params_.cross_check) {
    source_feature_tree.setInputCloud(source.features);
  }

  std::vector<FeatureCorrespondence> candidates;
  candidates.reserve(source.features->size());
  std::vector<int> target_indices(2);
  std::vector<float> target_distances(2);
  std::vector<int> source_indices(1);
  std::vector<float> source_distances(1);
  const double ratio_threshold_sq =
    params_.feature_ratio_threshold * params_.feature_ratio_threshold;

  for (std::size_t source_index = 0; source_index < source.features->size(); ++source_index) {
    if (
      target_feature_tree_->nearestKSearch(
        source.features->points[source_index], 2, target_indices, target_distances) < 2) {
      continue;
    }

    if (
      params_.feature_ratio_threshold > 0.0 && params_.feature_ratio_threshold < 1.0 &&
      target_distances[1] > 1e-12F &&
      static_cast<double>(target_distances[0]) >
        ratio_threshold_sq * static_cast<double>(target_distances[1])) {
      continue;
    }

    const int target_index = target_indices[0];
    if (params_.cross_check) {
      if (
        source_feature_tree.nearestKSearch(
          target_.features->points[static_cast<std::size_t>(target_index)], 1, source_indices,
          source_distances) < 1 ||
        source_indices[0] != static_cast<int>(source_index)) {
        continue;
      }
    }

    candidates.push_back({static_cast<int>(source_index), target_index, target_distances[0]});
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const FeatureCorrespondence & lhs, const FeatureCorrespondence & rhs) {
      return lhs.squared_feature_distance < rhs.squared_feature_distance;
    });

  const std::size_t correspondence_limit = params_.max_correspondences > 0
                                             ? static_cast<std::size_t>(params_.max_correspondences)
                                             : candidates.size();
  correspondences.reserve(std::min(candidates.size(), correspondence_limit));
  std::vector<bool> target_used(target_.features->size(), false);
  for (const auto & candidate : candidates) {
    const auto target_index = static_cast<std::size_t>(candidate.target_index);
    if (target_used[target_index]) {
      continue;
    }
    target_used[target_index] = true;
    correspondences.emplace_back(candidate.source_index, candidate.target_index);
    if (correspondences.size() >= correspondence_limit) {
      break;
    }
  }
  return correspondences;
}

NearestNeighborMetrics TeaserRelocalizer::evaluateOverlap(
  const pcl::PointCloud<pcl::PointXYZ> & source, const Eigen::Isometry3d & map_to_source,
  double max_distance) const
{
  NearestNeighborMetrics metrics;
  if (
    source.empty() || !overlap_map_ || overlap_map_->empty() || !overlap_tree_ ||
    !map_to_source.matrix().allFinite() || !std::isfinite(max_distance) || max_distance <= 0.0) {
    return metrics;
  }

  const auto sampled = voxelSample(source, params_.overlap_source_leaf_size);
  if (!sampled || sampled->empty()) {
    return metrics;
  }

  pcl::PointCloud<pcl::PointXYZ> transformed;
  pcl::transformPointCloud(*sampled, transformed, map_to_source.matrix().cast<float>());

  const double max_distance_sq = max_distance * max_distance;
  double squared_error_sum = 0.0;
  std::vector<int> nearest_indices(1);
  std::vector<float> nearest_distances(1);
  for (const auto & point : transformed.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    ++metrics.source_points;
    if (
      overlap_tree_->nearestKSearch(point, 1, nearest_indices, nearest_distances) > 0 &&
      static_cast<double>(nearest_distances[0]) <= max_distance_sq) {
      ++metrics.inliers;
      squared_error_sum += static_cast<double>(nearest_distances[0]);
    }
  }

  if (metrics.source_points == 0U || metrics.inliers == 0U) {
    return metrics;
  }

  metrics.overlap_ratio =
    static_cast<double>(metrics.inliers) / static_cast<double>(metrics.source_points);
  metrics.rmse = std::sqrt(squared_error_sum / static_cast<double>(metrics.inliers));
  metrics.valid = std::isfinite(metrics.overlap_ratio) && std::isfinite(metrics.rmse);
  return metrics;
}

TeaserResult TeaserRelocalizer::align(const pcl::PointCloud<pcl::PointXYZ> & source) const
{
  TeaserResult result;
  result.target_feature_points = targetFeaturePointCount();
  if (source.size() < static_cast<std::size_t>(params_.min_source_points)) {
    result.reason = "insufficient_source_points";
    return result;
  }
  if (
    !target_.points || target_.points->empty() || !target_.features || target_.features->empty()) {
    result.reason = "target_features_not_ready";
    return result;
  }

  const FeatureSet source_features = extractFeatures(source);
  result.source_feature_points = source_features.points ? source_features.points->size() : 0U;
  if (
    !source_features.points || !source_features.features ||
    source_features.points->size() < static_cast<std::size_t>(params_.min_feature_points)) {
    result.reason = "insufficient_source_feature_points";
    return result;
  }

  const auto correspondences = findCorrespondences(source_features);
  result.correspondences = correspondences.size();
  if (correspondences.size() < static_cast<std::size_t>(params_.min_correspondences)) {
    result.reason = "insufficient_correspondences";
    return result;
  }

  teaser::RobustRegistrationSolver::Params solver_params;
  solver_params.noise_bound = params_.noise_bound;
  solver_params.cbar2 = params_.cbar2;
  solver_params.estimate_scaling = false;
  solver_params.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  solver_params.rotation_max_iterations = static_cast<std::size_t>(params_.rotation_max_iterations);
  solver_params.rotation_gnc_factor = params_.rotation_gnc_factor;
  solver_params.rotation_cost_threshold = params_.rotation_cost_threshold;
  solver_params.max_clique_time_limit = params_.max_clique_time_limit;
  solver_params.max_clique_num_threads = params_.num_threads;
  solver_params.inlier_selection_mode =
    params_.use_exact_max_clique
      ? teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT
      : teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;

  teaser::RobustRegistrationSolver solver(solver_params);
  const auto solution = solver.solve(
    toTeaserCloud(*source_features.points), toTeaserCloud(*target_.points), correspondences);
  if (!solution.valid || !solution.rotation.allFinite() || !solution.translation.allFinite()) {
    result.reason = "invalid_teaser_solution";
    return result;
  }

  result.transform.linear() = solution.rotation;
  result.transform.translation() = solution.translation;
  result.overlap = evaluateOverlap(source, result.transform, params_.initial_overlap_max_distance);
  if (!result.overlap.valid) {
    result.reason = "initial_overlap_invalid";
    return result;
  }
  if (
    result.overlap.overlap_ratio < params_.initial_min_overlap_ratio ||
    result.overlap.inliers < static_cast<std::size_t>(params_.initial_min_overlap_points) ||
    result.overlap.rmse > params_.initial_max_overlap_rmse) {
    result.reason = "initial_overlap_rejected";
    return result;
  }

  result.accepted = true;
  result.reason = "accepted";
  return result;
}

}  // namespace relocalization_manager
