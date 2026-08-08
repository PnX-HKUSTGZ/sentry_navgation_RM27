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

#ifndef RELOCALIZATION_MANAGER__TEASER_RELOCALIZER_HPP_
#define RELOCALIZATION_MANAGER__TEASER_RELOCALIZER_HPP_

#include <Eigen/Geometry>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace relocalization_manager
{

struct NearestNeighborMetrics
{
  bool valid{false};
  double overlap_ratio{0.0};
  double rmse{std::numeric_limits<double>::infinity()};
  std::size_t inliers{0};
  std::size_t source_points{0};
};

struct TeaserParams
{
  bool enabled{true};
  int num_threads{4};
  int min_source_points{1000};
  int min_feature_points{50};
  int min_correspondences{30};
  int max_correspondences{500};
  double voxel_size{0.5};
  double normal_radius{1.0};
  double fpfh_radius{2.0};
  bool cross_check{true};
  double feature_ratio_threshold{0.90};
  double noise_bound{0.4};
  double cbar2{2.0};
  int rotation_max_iterations{100};
  double rotation_gnc_factor{1.4};
  double rotation_cost_threshold{0.005};
  double max_clique_time_limit{1.0};
  bool use_exact_max_clique{true};
  double overlap_map_leaf_size{0.10};
  double overlap_source_leaf_size{0.20};
  double initial_overlap_max_distance{0.8};
  double initial_min_overlap_ratio{0.45};
  int initial_min_overlap_points{200};
  double initial_max_overlap_rmse{0.45};
};

struct TeaserResult
{
  bool accepted{false};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  std::size_t source_feature_points{0};
  std::size_t target_feature_points{0};
  std::size_t correspondences{0};
  NearestNeighborMetrics overlap;
  std::string reason;
};

class TeaserRelocalizer
{
public:
  explicit TeaserRelocalizer(const TeaserParams & params);

  bool setGlobalMap(const pcl::PointCloud<pcl::PointXYZ> & global_map, std::string & reason);
  TeaserResult align(const pcl::PointCloud<pcl::PointXYZ> & source) const;
  NearestNeighborMetrics evaluateOverlap(
    const pcl::PointCloud<pcl::PointXYZ> & source, const Eigen::Isometry3d & map_to_source,
    double max_distance) const;

  const TeaserParams & params() const;
  std::size_t targetFeaturePointCount() const;

private:
  struct FeatureSet
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features;
  };

  struct FeatureCorrespondence
  {
    int source_index{-1};
    int target_index{-1};
    float squared_feature_distance{std::numeric_limits<float>::infinity()};
  };

  FeatureSet extractFeatures(const pcl::PointCloud<pcl::PointXYZ> & cloud) const;
  std::vector<std::pair<int, int>> findCorrespondences(const FeatureSet & source) const;
  static bool isFiniteFeature(const pcl::FPFHSignature33 & feature);

  TeaserParams params_;
  FeatureSet target_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_map_;
  pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr target_feature_tree_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr overlap_tree_;
};

}  // namespace relocalization_manager

#endif  // RELOCALIZATION_MANAGER__TEASER_RELOCALIZER_HPP_
