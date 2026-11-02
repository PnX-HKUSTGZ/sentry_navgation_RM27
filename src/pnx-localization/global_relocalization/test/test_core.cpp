// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/core.hpp"
#include "global_relocalization/gravity_registration.hpp"
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cstdio>
#include <random>
#include <chrono>
#include <atomic>
#include <limits>
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
TEST(ScanContext,BatchedMatchingPreservesExhaustiveScoresAndSymmetricTieOrder) {
  // Independent exhaustive reference includes singly empty columns: treating
  // those as missing rather than mismatched would change retrieval semantics.
  const auto exhaustive=[](const Descriptor & q,const Descriptor & t) {
    double best=1.;int shift_best=0;const int sectors=q.cells.cols();
    for(int shift=0;shift<sectors;++shift) {
      double score=0;int used=0;
      for(int s=0;s<sectors;++s) {
        const auto a=q.cells.col(s),b=t.cells.col((s+shift)%sectors);
        const double an=a.norm(),bn=b.norm();if(an==0&&bn==0)continue;
        ++used;score+=(an==0||bn==0)?1.:1.-std::clamp(a.dot(b)/(an*bn),-1.,1.);
      }
      score=used?score/used:1.;if(score<best){best=score;shift_best=shift;}
    }
    double yaw=shift_best*2*M_PI/sectors;if(yaw>M_PI)yaw-=2*M_PI;
    return std::make_pair(best,yaw);
  };
  std::mt19937 random(719);std::uniform_real_distribution<double> value(0.,8.);
  for(int sectors:{4,16,60,120,513})for(int fixture=0;fixture<12;++fixture) {
    Descriptor q,t;q.cells.resize(20,sectors);t.cells.resize(20,sectors);
    for(int s=0;s<sectors;++s)for(int r=0;r<20;++r) {
      q.cells(r,s)=(random()%5==0)?0.:value(random);
      t.cells(r,s)=(random()%5==0)?0.:value(random);
    }
    if(fixture==0){q.cells.setZero();t.cells.setZero();}
    if(fixture==1){q.cells.setOnes();t.cells.setOnes();}
    if(fixture==2)for(int s=0;s<sectors;s+=3)q.cells.col(s).setZero();
    if(fixture==3)for(int s=0;s<sectors;s+=3)t.cells.col(s).setZero();
    if(fixture==4){q.cells.setOnes();t.cells.setOnes();t.cells(0,0)+=1e-9;}
    const auto expected=exhaustive(q,t),actual=descriptorMatch(q,t);
    EXPECT_NEAR(actual.first,expected.first,1e-15);
    EXPECT_DOUBLE_EQ(actual.second,expected.second)<<"sectors="<<sectors<<" fixture="<<fixture;
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
  for(bool bounded:{false,true}) for(bool deterministic:{false,true}) {
  params.bounded_global_correspondences=bounded;
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
  for(bool bounded:{false,true}) for(bool deterministic:{false,true}) for(int workers:{1,2}) {
  p.bounded_global_correspondences=bounded;
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

TEST(Retrieval,HeightLayersDoNotConsumeIndependentLocationBudget) {
  std::vector<Candidate> candidates;
  for(size_t xy=0;xy<6;++xy)for(size_t layer=0;layer<3;++layer) {
    Candidate c;c.database_location=xy;c.ring_key_rank=xy*3+layer+1;
    c.descriptor_distance=.1*xy+.001*layer;c.seed_translation<<double(xy),0.,.25*layer;
    candidates.push_back(c);
  }
  // SC chooses XY 0 and 1, including their three layers each. Ring-key's
  // independent best location is XY 5, again retaining all three layers.
  for(auto & c:candidates)c.ring_key_rank=c.database_location==5?c.ring_key_rank-15:c.ring_key_rank+3;
  const auto ranked_pool=selectRetrievalCandidates(candidates,candidates.size(),0);
  const auto selected=selectRetrievalCandidates(ranked_pool,2,1);
  ASSERT_EQ(selected.size(),9u);
  for(size_t xy:{0u,1u,5u}) {
    EXPECT_EQ(std::count_if(selected.begin(),selected.end(),[&](const Candidate & c){return c.database_location==xy;}),3);
  }
  for(const auto & c:selected) {
    EXPECT_EQ(c.scan_context_rank,c.database_location+1);
    EXPECT_EQ(c.from_scan_context,c.database_location<2);
    EXPECT_EQ(c.from_ring_key,c.database_location==5);
  }
}

TEST(Database,HeightLatticeRetainsAllViewsDeduplicatesLayersAndValidatesInputs) {
  const std::string path="/tmp/global_relocalization_height_lattice_test.pcd";
  Cloud map;std::mt19937 generator(919);
  std::uniform_real_distribution<float> x(-2,2),y(-2,2),z(-.1,2.);
  for(int i=0;i<1000;++i)map.emplace_back(x(generator),y(generator),z(generator));
  ASSERT_EQ(pcl::io::savePCDFileBinary(path,map),0);
  Params p;p.db_min_points=20;p.db_grid_resolution=1.;p.db_max_radius=10;
  Core single(p);single.loadMap(path);single.buildDatabase(Eigen::Vector3d::Zero());
  p.db_base_z_offsets={.5,0.,.25,.25};
  Core layered(p);layered.loadMap(path);layered.buildDatabase(Eigen::Vector3d::Zero());
  EXPECT_EQ(layered.databaseSize(),3*single.databaseSize());
  p.db_base_z_offsets={};EXPECT_THROW(Core invalid(p),std::invalid_argument);
  p.db_base_z_offsets={std::numeric_limits<double>::quiet_NaN()};EXPECT_THROW(Core invalid(p),std::invalid_argument);
  p.db_base_z_offsets=std::vector<double>(17,0.);EXPECT_THROW(Core invalid(p),std::invalid_argument);
  std::remove(path.c_str());
}

TEST(Registration,MinimumIndependentConfirmationsRejectsIncompleteInputsBeforeSearch) {
  Params p;p.minimum_confirmation_scans=3;Core core(p);
  for(size_t count:{1u,2u}) {
    const auto result=core.search(Cloud{},std::vector<Cloud>(count),Eigen::Vector3d::Zero(),[]{return false;});
    EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"insufficient independent stopped scans");
    EXPECT_EQ(result.candidates_tested,0u);EXPECT_EQ(result.independent_preparations,0u);
  }
  p.minimum_confirmation_scans=0;EXPECT_THROW(Core invalid(p),std::invalid_argument);
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
  p.max_fitness=.04;p.min_inlier_ratio=.9;p.deterministic_preprocessing=true;p.minimum_confirmation_scans=3;p.bounded_global_correspondences=true;
  Core serial(p);serial.loadMap(path);
  const auto one=serial.search(query,std::vector<Cloud>{independent,independent,independent},Eigen::Vector3d::Zero(),[]{return false;});
  p.candidate_workers=2;Core parallel(p);parallel.loadMap(path);
  std::vector<SearchEvent> observed;
  const auto two=parallel.search(query,std::vector<Cloud>{independent,independent,independent},Eigen::Vector3d::Zero(),[]{return false;},[&](const SearchEvent & event) {
    observed.push_back(event);
    // A broken diagnostic consumer cannot abort a candidate or confirmation.
    if(event.stage=="GICP_CANDIDATE_START")throw std::runtime_error("observer failure");
  });
  ASSERT_FALSE(observed.empty());EXPECT_EQ(observed.front().stage,"RETRIEVAL_COMPLETE");
  EXPECT_EQ(observed.front().seeds.size(),two.retrieval_candidate_count);
  EXPECT_EQ(std::count_if(observed.begin(),observed.end(),[](const SearchEvent & e){return e.stage=="GICP_FINE_QUALITY_COMPLETE";}),two.candidates_tested);
  EXPECT_EQ(std::count_if(observed.begin(),observed.end(),[](const SearchEvent & e){return e.stage=="CONFIRMATION_COMPLETE";}),3);
  EXPECT_EQ(std::count_if(observed.begin(),observed.end(),[](const SearchEvent & e){return e.stage=="GICP_COARSE_SEED_COMPLETE";}),two.candidates_tested);
  for(size_t i=0;i<two.retrieval_candidate_count;++i) {
    auto begin=std::find_if(observed.begin(),observed.end(),[&](const SearchEvent&e){return e.stage=="GICP_CANDIDATE_START"&&e.index==i;});
    auto end=std::find_if(observed.begin(),observed.end(),[&](const SearchEvent&e){return e.stage=="GICP_FINE_QUALITY_COMPLETE"&&e.index==i;});
    ASSERT_NE(begin,observed.end());ASSERT_NE(end,observed.end());EXPECT_LT(begin,end);
    EXPECT_EQ(end->candidate.scan_context_rank,two.hypotheses[i].scan_context_rank);
  }
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
  const auto interrupted=parallel.search(query,std::vector<Cloud>{independent,independent,independent},Eigen::Vector3d::Zero(),[&]{
    if(++checks<30)return false;if(checks==30)requested=std::chrono::steady_clock::now();return true;
  });
  const double join_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-requested).count();
  EXPECT_FALSE(interrupted.success);EXPECT_GT(interrupted.retrieval_candidate_count,2u);
  EXPECT_EQ(interrupted.reason,"canceled during GICP");EXPECT_LT(join_sec,1.0);
  EXPECT_TRUE(std::any_of(interrupted.hypotheses.begin(),interrupted.hypotheses.end(),[](const Candidate & c){return c.canceled&&c.total_sec>0.;}));
  RecordProperty("cancel_join_sec",join_sec);
  // A subsequent complete search proves all canceled worker tasks were joined.
  const auto retry=parallel.search(query,std::vector<Cloud>{independent,independent,independent},Eigen::Vector3d::Zero(),[]{return false;});
  EXPECT_TRUE(retry.success)<<retry.reason;
  p.candidate_workers=3;EXPECT_THROW(Core invalid(p),std::invalid_argument);
  p.candidate_workers=0;EXPECT_THROW(Core invalid(p),std::invalid_argument);
  std::remove(path.c_str());
}

namespace {
class StagedRegistration : public ::testing::Test {
protected:
  void SetUp() override {
    path="/tmp/global_relocalization_staged_test.pcd";
    std::mt19937 generator(27);
    std::uniform_real_distribution<float> x(-4,4),y(-3,3),z(-.2,2.);
    for(int i=0;i<1200;++i)map.emplace_back(x(generator),y(generator),z(generator));
    ASSERT_EQ(pcl::io::savePCDFileBinary(path,map),0);
    truth.translation()<<1,1,0;
    truth.linear()=Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pcl::transformPointCloud(map,query,truth.inverse().matrix().cast<float>());
    for(size_t i=0;i<query.size();i+=3)independent.push_back(query[i]);
    p.db_grid_resolution=.5;p.db_min_points=20;p.db_max_radius=10;
    p.gicp_leaf_size=.06;p.gicp_max_iterations=60;p.top_k=8;
    p.max_fitness=.04;p.min_inlier_ratio=.9;p.minimum_confirmation_scans=3;
    p.deterministic_preprocessing=true;p.bounded_global_correspondences=true;
  }
  void TearDown() override {std::remove(path.c_str());}
  std::vector<Cloud> stopped() const {return {independent,independent,independent};}
  PreparedSearch prepare(Core & core) {return core.prepare(query,offset,[]{return false;});}
  std::string path;
  Params p;
  Cloud map,query,independent;
  Eigen::Isometry3d truth{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d offset{.1,.126,.21};
};
}
TEST_F(StagedRegistration,MatchesLegacyNumericsAndDefersAllConfirmationEvents) {
  Core core(p);core.loadMap(path);
  const auto legacy=core.search(query,stopped(),offset,[]{return false;});
  ASSERT_TRUE(legacy.success)<<legacy.reason;
  std::vector<std::string> stages;
  const auto observer=[&](const SearchEvent & event){stages.push_back(event.stage);};
  const auto prepared=core.prepare(query,offset,[]{return false;},observer);
  ASSERT_TRUE(prepared.ready())<<prepared.diagnostics().reason;
  EXPECT_FALSE(prepared.diagnostics().success);
  EXPECT_EQ(prepared.diagnostics().independent_preparations,0u);
  EXPECT_TRUE(prepared.diagnostics().confirmations.empty());
  EXPECT_EQ(std::count(stages.begin(),stages.end(),"CONFIRMATION_START"),0);
  EXPECT_EQ(prepared.diagnostics().candidates_tested,legacy.candidates_tested);
  ASSERT_EQ(prepared.diagnostics().hypotheses.size(),legacy.hypotheses.size());
  for(size_t i=0;i<legacy.hypotheses.size();++i) {
    const auto & a=legacy.hypotheses[i];const auto & b=prepared.diagnostics().hypotheses[i];
    EXPECT_EQ(a.scan_context_rank,b.scan_context_rank);EXPECT_EQ(a.ring_key_rank,b.ring_key_rank);
    EXPECT_EQ(a.from_scan_context,b.from_scan_context);EXPECT_EQ(a.from_ring_key,b.from_ring_key);
    EXPECT_EQ(a.converged,b.converged);EXPECT_EQ(a.within_search_side,b.within_search_side);
    EXPECT_NEAR(a.fitness,b.fitness,1e-6);EXPECT_NEAR(a.inlier_ratio,b.inlier_ratio,1e-6);
    EXPECT_TRUE(a.map_from_reference.matrix().isApprox(b.map_from_reference.matrix(),1e-6));
  }
  const auto result=core.confirm(prepared,stopped(),[]{return false;},observer);
  ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_FALSE(prepared.ready());EXPECT_FALSE(prepared.diagnostics().success);
  EXPECT_EQ(result.reason,legacy.reason);EXPECT_EQ(result.query_preparations,1u);
  EXPECT_EQ(result.independent_preparations,3u);ASSERT_EQ(result.confirmations.size(),3u);
  EXPECT_EQ(std::count(stages.begin(),stages.end(),"CONFIRMATION_COMPLETE"),3);
  EXPECT_TRUE(result.candidate.map_from_reference.matrix().isApprox(legacy.candidate.map_from_reference.matrix(),1e-6));
  for(size_t i=0;i<3;++i) {
    EXPECT_NEAR(result.confirmations[i].fitness,legacy.confirmations[i].fitness,1e-6);
    EXPECT_TRUE(result.confirmations[i].map_from_reference.matrix().isApprox(legacy.confirmations[i].map_from_reference.matrix(),1e-6));
  }
  EXPECT_FALSE(core.confirm(prepared,stopped(),[]{return false;}).success);
}
TEST_F(StagedRegistration,RejectsForeignStaleCanceledAndFabricatedContinuations) {
  Core core(p),other(p);core.loadMap(path);
  PreparedSearch fabricated;
  EXPECT_FALSE(fabricated.ready());EXPECT_FALSE(fabricated.diagnostics().success);
  auto fake_diagnostics=fabricated.diagnostics();fake_diagnostics.success=true;
  fake_diagnostics.candidate.fitness=0.;fake_diagnostics.candidate.converged=true;
  EXPECT_FALSE(core.confirm(fabricated,stopped(),[]{return false;}).success);
  auto prepared=prepare(core);ASSERT_TRUE(prepared.ready())<<prepared.diagnostics().reason;
  EXPECT_FALSE(other.confirm(prepared,stopped(),[]{return false;}).success);
  EXPECT_TRUE(prepared.ready()); // A foreign Core cannot consume the original owner's token.
  core.loadMap(path); // Same hash still invalidates old target preprocessing/provenance.
  EXPECT_FALSE(prepared.ready());
  EXPECT_FALSE(core.confirm(prepared,stopped(),[]{return false;}).success);
  auto canceled=prepare(core);ASSERT_TRUE(canceled.ready())<<canceled.diagnostics().reason;
  auto copied=canceled;
  const auto stopped_early=core.confirm(canceled,stopped(),[]{return true;});
  EXPECT_FALSE(stopped_early.success);EXPECT_EQ(stopped_early.reason,"canceled before independent confirmation");
  EXPECT_FALSE(copied.ready());EXPECT_FALSE(core.confirm(copied,stopped(),[]{return false;}).success);
  auto superseded=prepare(core);ASSERT_TRUE(superseded.ready());
  const auto aborted=core.prepare(query,offset,[]{return true;});
  EXPECT_FALSE(aborted.ready());EXPECT_FALSE(aborted.diagnostics().success);
  EXPECT_FALSE(superseded.ready());
  EXPECT_FALSE(core.confirm(aborted,stopped(),[]{return false;}).success);
  EXPECT_FALSE(core.confirm(superseded,stopped(),[]{return false;}).success);
}
TEST_F(StagedRegistration,RequiresThreeGoodMutuallyConsistentStoppedScans) {
  Core core(p);core.loadMap(path);
  auto incomplete=prepare(core);ASSERT_TRUE(incomplete.ready());
  const auto too_few=core.confirm(incomplete,{independent,independent},[]{return false;});
  EXPECT_FALSE(too_few.success);EXPECT_EQ(too_few.reason,"insufficient independent stopped scans");
  EXPECT_EQ(too_few.independent_preparations,0u);EXPECT_FALSE(incomplete.ready());
  EXPECT_FALSE(core.confirm(incomplete,{},[]{return false;}).success);
  auto prepared=prepare(core);ASSERT_TRUE(prepared.ready());
  const auto bad_third=core.confirm(prepared,{independent,independent,Cloud{}},[]{return false;});
  EXPECT_FALSE(bad_third.success);EXPECT_EQ(bad_third.independent_preparations,3u);
  EXPECT_EQ(bad_third.reason,"independent stopped scan failed quality or search-side gates");
  // Each registration can agree with the accumulated cloud within 15 cm, yet
  // disagree with the other stopped scan by 20 cm. Preserve the pairwise gate.
  Cloud plus=independent,minus=independent;
  for(auto & point:plus)point.x+=.1F;
  for(auto & point:minus)point.x-=.1F;
  prepared=prepare(core);ASSERT_TRUE(prepared.ready());
  const auto inconsistent=core.confirm(prepared,{plus,minus,independent},[]{return false;});
  EXPECT_FALSE(inconsistent.success);
  EXPECT_EQ(inconsistent.reason,"independent stopped scans disagree with each other");
  ASSERT_EQ(inconsistent.confirmations.size(),2u);
  for(const auto & c:inconsistent.confirmations) {
    EXPECT_TRUE(c.converged);EXPECT_LE(c.fitness,p.max_fitness);EXPECT_GE(c.inlier_ratio,p.min_inlier_ratio);
    EXPECT_LT((c.map_from_reference.translation()-prepared.diagnostics().candidate.map_from_reference.translation()).norm(),p.confirmation_max_translation);
  }
}

TEST_F(StagedRegistration,ConfirmationsValidateGlobalAnchorWithoutLastFrameOrderBias) {
  Cloud plus=independent,minus=independent;
  for(auto & point:plus)point.x+=.04F;
  for(auto & point:minus)point.x-=.04F;
  Core core(p);core.loadMap(path);
  const auto first=prepare(core);ASSERT_TRUE(first.ready());
  const auto anchor=first.diagnostics().candidate;
  const auto forward=core.confirm(first,{plus,independent,minus},[]{return false;});
  ASSERT_TRUE(forward.success)<<forward.reason;ASSERT_EQ(forward.confirmations.size(),3u);
  // The last scan has a distinct but admissible view-specific refinement.
  EXPECT_GT((forward.confirmations.back().map_from_reference.translation()-anchor.map_from_reference.translation()).norm(),.025);
  EXPECT_TRUE(forward.candidate.map_from_reference.matrix().isApprox(anchor.map_from_reference.matrix(),1e-12));
  EXPECT_DOUBLE_EQ(forward.candidate.fitness,anchor.fitness);
  const auto second=prepare(core);ASSERT_TRUE(second.ready());
  const auto reverse=core.confirm(second,{minus,independent,plus},[]{return false;});
  ASSERT_TRUE(reverse.success)<<reverse.reason;
  EXPECT_TRUE(reverse.candidate.map_from_reference.matrix().isApprox(second.diagnostics().candidate.map_from_reference.matrix(),1e-12));
  EXPECT_TRUE(reverse.candidate.map_from_reference.matrix().isApprox(forward.candidate.map_from_reference.matrix(),1e-6));
  // Keeping the reference anchor does not freeze the robot at the old query
  // timestamp: translations and all three current rotation axes still compose.
  Eigen::Isometry3d current=Eigen::Isometry3d::Identity();current.translation()<<.07,-.03,.02;
  current.linear()=(Eigen::AngleAxisd(.12,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.04,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.03,Eigen::Vector3d::UnitX())).toRotationMatrix();
  const auto output=composeCurrentBasePose(forward.candidate.map_from_reference,Eigen::Isometry3d::Identity(),current);
  EXPECT_TRUE(output.matrix().isApprox((anchor.map_from_reference*current).matrix(),1e-12));
  EXPECT_GT((output.translation()-anchor.map_from_reference.translation()).norm(),.05);
}

TEST_F(StagedRegistration,SecondPageIsEvaluatedOnceAndKeepsDistinctObserverIndices) {
  p.top_k=1;p.ring_key_candidates=0;p.retrieval_all_views=true;
  p.max_fitness=0.;p.candidate_workers=2;
  Core core(p);core.loadMap(path);
  std::vector<size_t> started_indices;
  auto prepared=core.prepare(query,offset,[]{return false;},[&](const SearchEvent & event){
    if(event.stage=="GICP_CANDIDATE_START")started_indices.push_back(event.index);
  });
  EXPECT_FALSE(prepared.ready());
  const auto & result=prepared.diagnostics();
  ASSERT_EQ(result.hypotheses.size(),2u);
  EXPECT_EQ(result.candidates_tested,2u);
  EXPECT_EQ(result.retrieval_candidate_count,2u);
  EXPECT_EQ(result.hypotheses[0].scan_context_rank,1u);
  EXPECT_EQ(result.hypotheses[1].scan_context_rank,2u);
  EXPECT_EQ(started_indices,(std::vector<size_t>{0,1}));
  EXPECT_TRUE(result.confirmations.empty());
}

TEST_F(StagedRegistration,ConfiguredSupplementIsBoundedAndDeduplicatesRingSeeds) {
  p.top_k=1;p.ring_key_candidates=5;p.retrieval_all_views=true;
  p.fallback_candidate_count=7;p.max_fitness=0.;p.candidate_workers=2;
  Core core(p);core.loadMap(path);
  std::vector<size_t> indices;
  auto prepared=core.prepare(query,offset,[]{return false;},[&](const SearchEvent & event){
    if(event.stage=="GICP_CANDIDATE_START")indices.push_back(event.index);
  });
  EXPECT_FALSE(prepared.ready());const auto & result=prepared.diagnostics();
  std::vector<size_t> ranks;
  for(const auto & c:result.hypotheses)ranks.push_back(c.scan_context_rank);
  std::sort(ranks.begin(),ranks.end());
  EXPECT_EQ(std::unique(ranks.begin(),ranks.end()),ranks.end());
  for(size_t rank=1;rank<=8;++rank)EXPECT_TRUE(std::binary_search(ranks.begin(),ranks.end(),rank));
  for(const auto & c:result.hypotheses)EXPECT_TRUE(c.scan_context_rank<=8||c.from_ring_key);
  EXPECT_EQ(result.candidates_tested,result.retrieval_candidate_count);
  std::sort(indices.begin(),indices.end());
  ASSERT_EQ(indices.size(),ranks.size());
  for(size_t i=0;i<indices.size();++i)EXPECT_EQ(indices[i],i);
  EXPECT_TRUE(result.confirmations.empty());
  p.fallback_candidate_count=-1;EXPECT_THROW(Core invalid(p),std::invalid_argument);
  p.fallback_candidate_count=257;EXPECT_THROW(Core invalid(p),std::invalid_argument);
}

TEST_F(StagedRegistration,HeightLayersPreserveSupplementLocationBudgetAndObserverIndices) {
  p.top_k=1;p.ring_key_candidates=0;p.fallback_candidate_count=2;
  p.retrieval_all_views=true;p.db_base_z_offsets={0.,.25,.5};p.max_fitness=0.;
  Core core(p);core.loadMap(path);
  std::vector<size_t> indices;
  const auto prepared=core.prepare(query,offset,[]{return false;},[&](const SearchEvent & event){
    if(event.stage=="GICP_CANDIDATE_START")indices.push_back(event.index);
  });
  EXPECT_FALSE(prepared.ready());const auto & result=prepared.diagnostics();
  ASSERT_EQ(result.hypotheses.size(),9u);
  for(size_t rank=1;rank<=3;++rank) {
    EXPECT_EQ(std::count_if(result.hypotheses.begin(),result.hypotheses.end(),[&](const Candidate & c){return c.scan_context_rank==rank;}),3);
  }
  std::sort(indices.begin(),indices.end());ASSERT_EQ(indices.size(),9u);
  for(size_t i=0;i<indices.size();++i)EXPECT_EQ(indices[i],i);
}

TEST_F(StagedRegistration,GenericHeightLatticeRegistersElevatedQueryWithFullSE3) {
  p.db_base_z=.08;p.db_base_z_offsets={0.,.25,.5};
  truth.translation().z()=.38;
  pcl::transformPointCloud(map,query,truth.inverse().matrix().cast<float>());
  independent.clear();for(size_t i=0;i<query.size();i+=3)independent.push_back(query[i]);
  Core core(p);core.loadMap(path);const auto result=core.search(query,stopped(),offset,[]{return false;});
  ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_LT((result.candidate.map_from_reference.translation()-truth.translation()).norm(),.025);
  EXPECT_LT(Eigen::AngleAxisd(result.candidate.map_from_reference.rotation().transpose()*truth.rotation()).angle(),.01);
  EXPECT_TRUE(std::any_of(result.hypotheses.begin(),result.hypotheses.end(),[](const Candidate & c){return std::abs(c.seed_translation.z()-.33)<1e-6;}));
}

TEST(GravityDirectionPrior,RightJacobianAndYawTranslationNullspace) {
  GravityDirectionPrior prior;prior.enabled=true;prior.inverse_variance=40000.;
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.linear()=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.13,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.09,Eigen::Vector3d::UnitX())).toRotationMatrix();
  prior.map_up=Eigen::Vector3d(.1,-.2,1.).normalized();const auto j=prior.jacobian(t);
  for(int i=0;i<6;++i) {
    Eigen::Matrix<double,6,1> d=Eigen::Matrix<double,6,1>::Zero();d[i]=1e-7;
    const Eigen::Vector3d numeric=(prior.residual(t*small_gicp::se3_exp(d))-prior.residual(t*small_gicp::se3_exp(-d)))/(2e-7);
    EXPECT_TRUE(numeric.isApprox(j.col(i),1e-7))<<i;
  }
  EXPECT_LT(j.col(2).norm(),1e-15);EXPECT_LT(j.rightCols<3>().norm(),1e-15);
  Eigen::Isometry3d delta=Eigen::Isometry3d::Identity();delta.linear()=Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitZ()).toRotationMatrix();delta.translation()<<2.,-3.,1.;
  EXPECT_NEAR(prior.cost(t),prior.cost(t*delta),1e-10);
}
TEST(GravityDirectionPrior,RejectsInvalidConfiguration) {
  Params p;p.confirmation_gravity_sigma=0.;EXPECT_THROW(Core invalid(p),std::invalid_argument);
  p.confirmation_gravity_sigma=.005;p.confirmation_map_up.setZero();EXPECT_THROW(Core invalid(p),std::invalid_argument);
}
TEST_F(StagedRegistration,GravityIsConfirmationOnlyAndPreservesCurrentBaseAttitude) {
  Core baseline(p);baseline.loadMap(path);const auto old=prepare(baseline);ASSERT_TRUE(old.ready());
  p.confirmation_gravity_prior=true;Core core(p);core.loadMap(path);const auto prepared=prepare(core);ASSERT_TRUE(prepared.ready());
  EXPECT_TRUE(prepared.diagnostics().candidate.map_from_reference.matrix().isApprox(old.diagnostics().candidate.map_from_reference.matrix(),1e-12));
  EXPECT_DOUBLE_EQ(prepared.diagnostics().candidate.fitness,old.diagnostics().candidate.fitness);
  const auto result=core.confirm(prepared,stopped(),[]{return false;});ASSERT_TRUE(result.success)<<result.reason;
  Eigen::Isometry3d current=Eigen::Isometry3d::Identity();current.translation()<<.07,-.03,.02;
  current.linear()=(Eigen::AngleAxisd(.12,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.04,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.03,Eigen::Vector3d::UnitX())).toRotationMatrix();
  const auto output=composeCurrentBasePose(result.candidate.map_from_reference,Eigen::Isometry3d::Identity(),current);
  EXPECT_TRUE(output.matrix().isApprox((result.candidate.map_from_reference*current).matrix(),1e-12));
  EXPECT_GT((output.rotation()*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm(),.04);
  const auto bad=prepare(core);ASSERT_TRUE(bad.ready());
  auto wrong=independent;for(auto & point:wrong)point.x+=10.F;
  const auto rejected=core.confirm(bad,{wrong,wrong,wrong},[]{return false;});EXPECT_FALSE(rejected.success);
  EXPECT_EQ(rejected.reason,"independent stopped scan failed quality or search-side gates");
}
