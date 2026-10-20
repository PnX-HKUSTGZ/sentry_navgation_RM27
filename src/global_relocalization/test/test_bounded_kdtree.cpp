// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/bounded_kdtree.hpp"
#include <small_gicp/points/point_cloud.hpp>
#include <gtest/gtest.h>
#include <random>
using Points=small_gicp::PointCloud;
using Tree=small_gicp::KdTree<Points>;
static void compare(const Tree & tree,const Eigen::Vector4d & query,double bound) {
  size_t old_index=0,new_index=0;double old_distance=0,new_distance=0;
  const bool original=tree.nearest_neighbor_search(query,&old_index,&old_distance)&&old_distance<=bound;
  const global_relocalization::BoundedKdTree bounded(tree,bound);
  const bool actual=small_gicp::traits::nearest_neighbor_search(bounded,query,&new_index,&new_distance)!=0;
  ASSERT_EQ(actual,original);
  if(original){EXPECT_EQ(new_index,old_index);EXPECT_DOUBLE_EQ(new_distance,old_distance);}
}
TEST(BoundedKdTree,RandomMapNearAndFarExactCorrespondences) {
  auto points=std::make_shared<Points>();points->resize(5000);
  std::mt19937 random(27);std::uniform_real_distribution<double> coordinate(-20,20);
  for(auto & point:points->points)point<<coordinate(random),coordinate(random),coordinate(random),1.;
  const Tree tree(points);
  for(int i=0;i<5000;++i)for(double bound:{0.,1.,2.25}) {
    Eigen::Vector4d q(coordinate(random)*(i%2?10.:1.),coordinate(random),coordinate(random),i%3?.95:1.);
    compare(tree,q,bound);
  }
}
TEST(BoundedKdTree,PlanarDuplicateAndEquidistantTieKeepsVisitOrder) {
  auto points=std::make_shared<Points>();
  for(int i=0;i<40;++i)for(int j=0;j<20;++j) {
    points->points.emplace_back(i%2?1.:-1.,j*.2,0,1.);
    points->points.push_back(points->points.back());
  }
  const Tree tree(points);
  for(int i=0;i<100;++i)for(double bound:{1.,2.25})compare(tree,Eigen::Vector4d(0,i*.1,0,1),bound);
}
TEST(BoundedKdTree,InclusiveBoundaryNextRepresentableOutsideAndFourDimensions) {
  auto points=std::make_shared<Points>();points->points.emplace_back(0,0,0,1);const Tree tree(points);
  compare(tree,Eigen::Vector4d(1,0,0,1),1.);
  compare(tree,Eigen::Vector4d(std::nextafter(1.,2.),0,0,1),1.);
  compare(tree,Eigen::Vector4d(0,0,0,2),1.);
  compare(tree,Eigen::Vector4d(0,0,0,std::nextafter(2.,3.)),1.);
  compare(tree,Eigen::Vector4d(0,0,0,1),0.);
}
TEST(BoundedKdTree,BoundaryAcrossSplitPlanesAndRadiusMiss) {
  auto points=std::make_shared<Points>();
  for(int i=0;i<200;++i)points->points.emplace_back((i%2?1.:-1.),(i%5)*.5,0,1);
  const Tree tree(points);for(int i=0;i<1000;++i)compare(tree,Eigen::Vector4d(0,(i%20)*.125,0,1),1.);
  compare(tree,Eigen::Vector4d(1000,1000,1000,1),2.25);
}
TEST(BoundedKdTree,EmptyNonfiniteQueriesAndAllNonfinitePointsAreSafe) {
  auto empty=std::make_shared<Points>();const Tree tree(empty);global_relocalization::BoundedKdTree bounded(tree,1.);
  size_t index;double distance;EXPECT_EQ(bounded.nearest_neighbor_search(Eigen::Vector4d::Ones(),&index,&distance),0u);
  auto invalid=std::make_shared<Points>();for(int i=0;i<40;++i)invalid->points.emplace_back(NAN,NAN,NAN,1);const Tree invalid_tree(invalid);global_relocalization::BoundedKdTree invalid_bounded(invalid_tree,1.);
  EXPECT_EQ(invalid_bounded.nearest_neighbor_search(Eigen::Vector4d::Ones(),&index,&distance),0u);
  EXPECT_EQ(invalid_bounded.nearest_neighbor_search(Eigen::Vector4d(NAN,0,0,1),&index,&distance),0u);
  EXPECT_THROW((global_relocalization::BoundedKdTree(tree,-1.)),std::invalid_argument);
}
