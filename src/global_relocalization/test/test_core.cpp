// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/core.hpp"
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cstdio>
#include <random>
#include <chrono>
#include <atomic>
using namespace global_relocalization;
TEST(ScanContext,AnalyticPolarMaxHeightCells) {
  Cloud cloud;cloud.emplace_back(1,0,-1);cloud.emplace_back(1,0,2);
  cloud.emplace_back(0,2,1);cloud.emplace_back(-3,0,0);cloud.emplace_back(0,-4,-.5);
  auto descriptor=scanContext(cloud,5,4,5);
  EXPECT_DOUBLE_EQ(descriptor.cells(1,0),5.);
  EXPECT_DOUBLE_EQ(descriptor.cells(2,1),4.);
  EXPECT_DOUBLE_EQ(descriptor.cells(3,2),3.);
  EXPECT_DOUBLE_EQ(descriptor.cells(4,3),2.5);
  EXPECT_DOUBLE_EQ(descriptor.cells.sum(),14.5);
}
TEST(ScanContext,YawSignBothQuarterTurns) {
  Cloud target;
  for(int i=0;i<70;++i){double a=.07+i*.071;double r=2.5+(i%11)*.37;
    target.emplace_back(r*cos(a),r*sin(a),.3+(i%7)*.4);}
  auto td=scanContext(target,20,60,15);
  for(double yaw:{M_PI/2,-M_PI/2}) {
    Eigen::Isometry3d query_to_map=Eigen::Isometry3d::Identity();
    query_to_map.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Cloud query;pcl::transformPointCloud(target,query,query_to_map.inverse().matrix().cast<float>());
    auto matched=descriptorMatch(scanContext(query,20,60,15),td);
    EXPECT_NEAR(matched.second,yaw,1e-6);EXPECT_NEAR(matched.first,0.,1e-5);
  }
}
TEST(ReferenceFrame,FullExtrinsicsAndCurrentPose) {
  Eigen::Isometry3d map_from_base=Eigen::Isometry3d::Identity();
  map_from_base.translation()<<8,-4,.27;
  map_from_base.linear()=(Eigen::AngleAxisd(1.2,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitY())).toRotationMatrix();
  Eigen::Isometry3d odom_reference=Eigen::Isometry3d::Identity();odom_reference.translation()<<2,3,.1;
  odom_reference.linear()=Eigen::AngleAxisd(-.6,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Isometry3d base_lidar=Eigen::Isometry3d::Identity();base_lidar.translation()<<.1,.126,.21;
  base_lidar.linear()=Eigen::AngleAxisd(-M_PI/6,Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Isometry3d current=odom_reference*base_lidar;
  auto result=composeCurrentBasePose(map_from_base,odom_reference,current);
  EXPECT_TRUE(result.matrix().isApprox((map_from_base*base_lidar).matrix(),1e-12));
  Eigen::Vector3d point(2,.4,1);
  const auto in_reference=odom_reference.inverse()*(odom_reference*base_lidar*point);
  EXPECT_TRUE(in_reference.isApprox(base_lidar*point,1e-12));
}
TEST(ReferenceFrame,SensorCenteredYawSeedCorrectsMountTranslation) {
  const Eigen::Vector3d sensor_in_map(8,-2,.29),sensor_in_base(.1,.126,.21);
  for(double yaw:{M_PI/2,-M_PI/2}) {
    auto base=basePoseFromSensorView(sensor_in_map,sensor_in_base,yaw);
    EXPECT_TRUE((base*sensor_in_base).isApprox(sensor_in_map,1e-12));
    EXPECT_NEAR(base.translation().z(),.08,1e-12);
  }
}
TEST(SearchSide,StrictBaseBoundaryAndMountOffset) {
  Params p;p.search_side="negative_x";
  Eigen::Isometry3d base=Eigen::Isometry3d::Identity();
  EXPECT_FALSE(poseInSearchSide(base,p));
  base.translation().x()=-1e-12;EXPECT_TRUE(poseInSearchSide(base,p));
  base.translation().x()=1e-12;EXPECT_FALSE(poseInSearchSide(base,p));
  p.search_side="positive_x";EXPECT_TRUE(poseInSearchSide(base,p));
  base.translation().x()=0;EXPECT_FALSE(poseInSearchSide(base,p));
  p.search_side="all";EXPECT_TRUE(poseInSearchSide(base,p));
  p.search_side="negative_x";
  const Eigen::Vector3d mount(.1,.126,.21);
  // A negative sensor origin can describe a positive base after yaw rotation.
  auto wrong_side=basePoseFromSensorView(Eigen::Vector3d(-.01,0,.29),mount,M_PI/2);
  EXPECT_GT(wrong_side.translation().x(),0);EXPECT_FALSE(poseInSearchSide(wrong_side,p));
  // A positive sensor origin can describe a negative base; check the base.
  auto valid_side=basePoseFromSensorView(Eigen::Vector3d(.01,0,.29),mount,0);
  EXPECT_LT(valid_side.translation().x(),0);EXPECT_TRUE(poseInSearchSide(valid_side,p));
  auto reference=Eigen::Isometry3d::Identity();reference.translation().x()=-.05;
  auto current=Eigen::Isometry3d::Identity();current.translation().x()=.1;
  auto output=composeCurrentBasePose(reference,Eigen::Isometry3d::Identity(),current);
  EXPECT_FALSE(poseInSearchSide(output,p));
  p.search_side="invalid";EXPECT_THROW(Core core(p),std::invalid_argument);
}
TEST(Database,SmallFixedMapAndHash) {
  const std::string path="/tmp/global_relocalization_core_test.pcd";
  Cloud map;
  for(int x=0;x<20;++x)for(int z=0;z<10;++z){map.emplace_back(x*.2F,0,z*.15F);map.emplace_back(x*.2F,3,z*.15F);}
  pcl::io::savePCDFileBinary(path,map);
  Params params;params.db_grid_resolution=1.;params.db_min_points=20;
  Core core(params);core.loadMap(path);core.buildDatabase(Eigen::Vector3d(0,.1,.2));
  EXPECT_EQ(core.mapHash().size(),64u);EXPECT_GT(core.databaseSize(),0u);
  auto first=core.databaseSize();core.buildDatabase(Eigen::Vector3d(0,.1,.2));EXPECT_EQ(first,core.databaseSize());
  EXPECT_FALSE(core.search(map,map,Eigen::Vector3d(0,.1,.2),[]{return true;}).success);
  std::remove(path.c_str());
}
TEST(Registration,GlobalRetrievalAndIndependentScanWithNonzeroExtrinsics) {
  const std::string path="/tmp/global_relocalization_registration_test.pcd";
  Cloud map;std::mt19937 generator(27);
  std::uniform_real_distribution<float> x(-4,4),y(-3,3),z(-.2,2.);
  for(int i=0;i<1200;++i)map.emplace_back(x(generator),y(generator),z(generator));
  pcl::io::savePCDFileBinary(path,map);
  Eigen::Isometry3d truth=Eigen::Isometry3d::Identity();truth.translation()<<1,1,0;
  truth.linear()=Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Cloud query;pcl::transformPointCloud(map,query,truth.inverse().matrix().cast<float>());
  Cloud independent;for(size_t i=0;i<query.size();i+=3)independent.push_back(query[i]);
  Params params;params.db_grid_resolution=.5;params.db_min_points=20;params.db_max_radius=10;
  params.gicp_leaf_size=.06;params.gicp_max_iterations=60;params.top_k=8;params.max_fitness=.04;
  params.min_inlier_ratio=.9;
  for(bool deterministic:{false,true}) {
  params.deterministic_preprocessing=deterministic;
  Core core(params);core.loadMap(path);
  auto result=core.search(query,independent,Eigen::Vector3d(.1,.126,.21),[]{return false;});
  ASSERT_TRUE(result.success)<<result.reason<<" fitness="<<result.candidate.fitness;
  EXPECT_LT((result.candidate.map_from_reference.translation()-truth.translation()).norm(),.025);
  EXPECT_LT(Eigen::AngleAxisd(result.candidate.map_from_reference.rotation().transpose()*truth.rotation()).angle(),.01);
  EXPECT_GT(result.candidates_tested,1u);
  EXPECT_EQ(result.query_preparations,1u);
  EXPECT_EQ(result.independent_preparations,1u);
  // Confirmation inputs keep their own point sets: a bad third scan must not
  // accidentally reuse the query or an earlier valid confirmation cache.
  std::vector<Cloud> stopped{independent,independent,Cloud{}};
  auto rejected=core.search(query,stopped,Eigen::Vector3d(.1,.126,.21),[]{return false;});
  EXPECT_FALSE(rejected.success);
  EXPECT_EQ(rejected.query_preparations,1u);
  EXPECT_EQ(rejected.independent_preparations,3u);
  EXPECT_EQ(rejected.reason,"independent stopped scan failed quality or search-side gates");
  }
  std::remove(path.c_str());
}
TEST(Registration,SymmetricMapRequiresCoarseSidePrior) {
  const std::string path="/tmp/global_relocalization_symmetric_test.pcd";
  Cloud map;std::mt19937 generator(81);
  std::uniform_real_distribution<float> x(-4,-1),y(-3,3),z(-.2,2.);
  for(int i=0;i<1200;++i) {
    pcl::PointXYZ point(x(generator),y(generator),z(generator));
    map.push_back(point);map.emplace_back(-point.x,-point.y,point.z);
  }
  pcl::io::savePCDFileBinary(path,map);
  Eigen::Isometry3d truth=Eigen::Isometry3d::Identity();truth.translation()<<-2,1,0;
  truth.linear()=Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Cloud query;pcl::transformPointCloud(map,query,truth.inverse().matrix().cast<float>());
  Cloud independent;for(size_t i=0;i<query.size();i+=3)independent.push_back(query[i]);
  Params p;p.db_grid_resolution=.5;p.db_min_points=20;p.db_max_radius=12;
  p.gicp_leaf_size=.06;p.gicp_max_iterations=60;p.top_k=20;p.max_fitness=.04;p.min_inlier_ratio=.9;
  for(bool deterministic:{false,true}) for(int workers:{1,2}) {
  p.candidate_workers=workers;
  p.deterministic_preprocessing=deterministic;p.search_side="all";p.side_boundary_x=0;
  pcl::io::savePCDFileBinary(path,map);
  Core unrestricted(p);unrestricted.loadMap(path);
  auto both=unrestricted.search(query,independent,Eigen::Vector3d::Zero(),[]{return false;});
  EXPECT_FALSE(both.success);EXPECT_EQ(both.reason,"ambiguous distinct global registration hypotheses");
  p.search_side="negative_x";Core negative(p);negative.loadMap(path);
  auto left=negative.search(query,independent,Eigen::Vector3d::Zero(),[]{return false;});
  ASSERT_TRUE(left.success)<<left.reason;
  EXPECT_TRUE(poseInSearchSide(left.candidate.map_from_reference,p));
  EXPECT_LT((left.candidate.map_from_reference.translation()-truth.translation()).norm(),.025);
  EXPECT_LT(Eigen::AngleAxisd(left.candidate.map_from_reference.rotation().transpose()*truth.rotation()).angle(),.01);
  EXPECT_LT(negative.databaseSize(),unrestricted.databaseSize());
  p.search_side="positive_x";Core positive(p);positive.loadMap(path);
  auto right=positive.search(query,independent,Eigen::Vector3d::Zero(),[]{return false;});
  ASSERT_TRUE(right.success)<<right.reason;
  EXPECT_TRUE(poseInSearchSide(right.candidate.map_from_reference,p));
  Eigen::Isometry3d half_turn=Eigen::Isometry3d::Identity();
  half_turn.linear()=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto equivalent=half_turn*truth;
  EXPECT_LT((right.candidate.map_from_reference.translation()-equivalent.translation()).norm(),.025);
  // Both successful calls receive the identical query. Only the declared side
  // changes; neither call receives a correct pose or a ground-truth transform.
  p.search_side="negative_x";p.side_boundary_x=-2.1;
  Core crossed(p);crossed.loadMap(path);
  auto outside=crossed.search(query,independent,Eigen::Vector3d::Zero(),[]{return false;});
  EXPECT_FALSE(outside.success);
  EXPECT_TRUE(std::any_of(outside.hypotheses.begin(),outside.hypotheses.end(),[&](const Candidate & c) {
    return c.converged&&c.fitness<=p.max_fitness&&!c.within_search_side;
  }))<<"Fixture must refine an admissible seed across the declared boundary";
  // Move the whole symmetric scene into one half: a side prior must preserve
  // ambiguity when both exact solutions remain inside that same half.
  Cloud shifted=map;for(auto & point:shifted)point.x-=8;
  pcl::io::savePCDFileBinary(path,shifted);
  p.side_boundary_x=0;Core same_side(p);same_side.loadMap(path);
  auto unresolved=same_side.search(query,independent,Eigen::Vector3d::Zero(),[]{return false;});
  EXPECT_FALSE(unresolved.success);
  EXPECT_EQ(unresolved.reason,"ambiguous distinct global registration hypotheses");
  }
  std::remove(path.c_str());
}
TEST(Retrieval,RetainsContextHypothesesAndIndependentRingKeyCoverage) {
  std::vector<Candidate> candidates;
  for(size_t i=0;i<24;++i) {
    Candidate c;c.descriptor_distance=.01*i;c.ring_key_rank=i+6;
    c.map_from_reference.translation()<<-13.,double(i),.08;candidates.push_back(c);
  }
  candidates[0].ring_key_rank=1;
  for(size_t i=12;i<16;++i)candidates[i].ring_key_rank=i-10;
  candidates[15].map_from_reference.translation()<<-4.9,-2.38,.08;
  auto original=selectRetrievalCandidates(candidates,10,0);
  ASSERT_EQ(original.size(),10u);
  EXPECT_TRUE(std::none_of(original.begin(),original.end(),[](const Candidate&c){return c.ring_key_rank==5;}));
  auto covered=selectRetrievalCandidates(candidates,10,5);
  ASSERT_EQ(covered.size(),14u);EXPECT_LE(covered.size(),15u);
  auto nearby=std::find_if(covered.begin(),covered.end(),[](const Candidate&c){return c.ring_key_rank==5;});
  ASSERT_NE(nearby,covered.end());EXPECT_EQ(nearby->scan_context_rank,16u);
  EXPECT_TRUE(nearby->from_ring_key);EXPECT_FALSE(nearby->from_scan_context);
  EXPECT_EQ(std::count_if(covered.begin(),covered.end(),[](const Candidate&c){return c.from_scan_context;}),10);
  EXPECT_EQ(std::count_if(covered.begin(),covered.end(),[](const Candidate&c){return c.from_ring_key;}),5);
}

TEST(Registration,ParallelCandidatesPreserveAllSeedsQualityOrderAndConfirmations) {
  const std::string path="/tmp/global_relocalization_parallel_test.pcd";
  Cloud map;std::mt19937 generator(27);
  std::uniform_real_distribution<float> x(-4,4),y(-3,3),z(-.2,2.);
  for(int i=0;i<1800;++i)map.emplace_back(x(generator),y(generator),z(generator));
  pcl::io::savePCDFileBinary(path,map);
  Eigen::Isometry3d truth=Eigen::Isometry3d::Identity();truth.translation()<<1,1,0;
  truth.linear()=Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Cloud query;pcl::transformPointCloud(map,query,truth.inverse().matrix().cast<float>());
  Cloud independent;for(size_t i=0;i<query.size();i+=3)independent.push_back(query[i]);
  Params p;p.db_grid_resolution=.5;p.db_min_points=20;p.db_max_radius=10;
  p.gicp_leaf_size=.06;p.gicp_max_iterations=60;p.top_k=8;p.ring_key_candidates=3;
  p.max_fitness=.04;p.min_inlier_ratio=.9;p.deterministic_preprocessing=true;
  Core serial(p);serial.loadMap(path);
  const auto one=serial.search(query,std::vector<Cloud>{independent,independent,independent},Eigen::Vector3d::Zero(),[]{return false;});
  p.candidate_workers=2;Core parallel(p);parallel.loadMap(path);
  const auto two=parallel.search(query,std::vector<Cloud>{independent,independent,independent},Eigen::Vector3d::Zero(),[]{return false;});
  ASSERT_TRUE(one.success)<<one.reason;ASSERT_TRUE(two.success)<<two.reason;
  ASSERT_GT(one.hypotheses.size(),2u);ASSERT_EQ(one.hypotheses.size(),two.hypotheses.size());
  EXPECT_EQ(two.candidates_tested,two.retrieval_candidate_count);
  EXPECT_EQ(one.candidates_tested,two.candidates_tested);EXPECT_EQ(one.reason,two.reason);
  EXPECT_EQ(two.query_preparations,1u);EXPECT_EQ(two.independent_preparations,3u);
  ASSERT_EQ(two.confirmations.size(),3u);
  for(size_t i=0;i<one.hypotheses.size();++i) {
    const auto & a=one.hypotheses[i];const auto & b=two.hypotheses[i];
    EXPECT_EQ(a.ring_key_rank,b.ring_key_rank);EXPECT_EQ(a.scan_context_rank,b.scan_context_rank);
    EXPECT_EQ(a.from_scan_context,b.from_scan_context);EXPECT_EQ(a.from_ring_key,b.from_ring_key);
    EXPECT_TRUE(a.seed_translation.isApprox(b.seed_translation,1e-12));EXPECT_DOUBLE_EQ(a.seed_yaw,b.seed_yaw);
    EXPECT_EQ(a.converged,b.converged);EXPECT_EQ(a.within_search_side,b.within_search_side);
    EXPECT_NEAR(a.fitness,b.fitness,1e-6);EXPECT_NEAR(a.inlier_ratio,b.inlier_ratio,1e-6);
    EXPECT_TRUE(a.map_from_reference.matrix().isApprox(b.map_from_reference.matrix(),1e-6));
    EXPECT_EQ(a.source_points,b.source_points);EXPECT_GT(b.coarse_iterations,0u);EXPECT_GT(b.fine_iterations,0u);
    EXPECT_GT(b.coarse_sec,0.);EXPECT_GT(b.fine_sec,0.);EXPECT_GT(b.quality_sec,0.);EXPECT_FALSE(b.canceled);
  }
  EXPECT_TRUE(one.candidate.map_from_reference.matrix().isApprox(two.candidate.map_from_reference.matrix(),1e-6));
  auto rejected=parallel.search(query,std::vector<Cloud>{independent,independent,Cloud{}},Eigen::Vector3d::Zero(),[]{return false;});
  EXPECT_FALSE(rejected.success);EXPECT_EQ(rejected.independent_preparations,3u);
  EXPECT_EQ(rejected.reason,"independent stopped scan failed quality or search-side gates");
  // Cancel from inside optimizer-stage checks after retrieval has selected >2
  // candidates. Joining must return diagnostics without accepting any result.
  int checks=0;std::chrono::steady_clock::time_point requested;
  const auto interrupted=parallel.search(query,independent,Eigen::Vector3d::Zero(),[&]{
    if(++checks<30)return false;if(checks==30)requested=std::chrono::steady_clock::now();return true;
  });
  const double join_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-requested).count();
  EXPECT_FALSE(interrupted.success);EXPECT_GT(interrupted.retrieval_candidate_count,2u);
  EXPECT_EQ(interrupted.reason,"canceled during GICP");EXPECT_LT(join_sec,1.0);
  EXPECT_TRUE(std::any_of(interrupted.hypotheses.begin(),interrupted.hypotheses.end(),[](const Candidate & c){return c.canceled&&c.total_sec>0.;}));
  RecordProperty("cancel_join_sec",join_sec);
  // A subsequent complete search proves all canceled worker tasks were joined.
  const auto retry=parallel.search(query,independent,Eigen::Vector3d::Zero(),[]{return false;});
  EXPECT_TRUE(retry.success)<<retry.reason;
  p.candidate_workers=3;EXPECT_THROW(Core invalid(p),std::invalid_argument);
  p.candidate_workers=0;EXPECT_THROW(Core invalid(p),std::invalid_argument);
  std::remove(path.c_str());
}
