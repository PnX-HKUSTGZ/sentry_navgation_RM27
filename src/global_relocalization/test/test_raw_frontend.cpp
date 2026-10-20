// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/stationary_raw_frontend.hpp"
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <random>
#include <limits>
#include <algorithm>
using namespace global_relocalization;
namespace {
Cloud scene(){Cloud cloud;std::mt19937 rng(27);std::uniform_real_distribution<float>d(-4,4);for(int j=0;j<4000;++j)cloud.emplace_back(d(rng),d(rng),d(rng)*.4);return cloud;}
Eigen::Isometry3d pose(double yaw,double pitch=0,double roll=0){Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();return t;}
void stationary(StationaryRawFrontend & f,double end=2.1){for(int j=0;j<=int(end*100+.1);++j)ASSERT_TRUE(f.addImu({j*.01,Eigen::Vector3d(.002,-.001,.003),Eigen::Vector3d(0,0,9.81)}));}
}
TEST(RawFrontend,FullSixDofMotionGravityAndNonzeroImuSensorBaseExtrinsics){
  for(bool robust:{false,true}) {
  RawFrontendParams p;p.stationary_duration=2.;p.max_iterations=30;p.max_fitness=.04;
  p.range_filter=robust;p.deterministic=robust;p.submap_scans=robust?5:1;p.imu_rotation_prior=robust;p.numerical_recovery=robust;
  auto bs=pose(.1,-.12,-.3);bs.translation()<<.1,.126,.21;
  auto si=pose(-.2,.15,.05);si.translation()<<.03,-.02,.01;
  auto b0=pose(.7,.10,-.08);b0.translation()<<2,-1,.08;
  auto b1=pose(.75,.10,-.08);b1.translation()=b0.translation()+Eigen::Vector3d(.015,.005,.002);
  auto s0=b0*bs,s1=b1*bs;auto wimu=b0*bs*si;
  const Eigen::Vector3d bias(.002,-.001,.003),gravity(0,0,9.81);
  StationaryRawFrontend f(p,si,bs);
  for(int j=0;j<=215;++j){double t=j*.01;auto b=t<=2.02?b0:pose(.7+.5*(t-2.02),.1,-.08);auto wi=b*bs*si;const Eigen::Vector3d w=t<=2.02?Eigen::Vector3d::Zero():Eigen::Vector3d(0,0,.5);ASSERT_TRUE(f.addImu({t,wi.rotation().transpose()*w+bias,wi.rotation().transpose()*gravity}));}
  auto map=scene();Cloud c0,c1;pcl::transformPointCloud(map,c0,s0.inverse().matrix().cast<float>());pcl::transformPointCloud(map,c1,s1.inverse().matrix().cast<float>());
  auto first=f.processScan(c0,2.02);ASSERT_TRUE(first.accepted)<<first.reason;EXPECT_TRUE(first.gyro_bias.isApprox(bias,1e-10));
  Eigen::Isometry3d refworld=first.reference_from_sensor*s0.inverse();
  EXPECT_TRUE((first.reference_from_sensor.rotation()*si.rotation()*wimu.rotation().transpose()*gravity).isApprox(gravity,1e-8));
  EXPECT_TRUE(first.reference_from_base.matrix().isApprox((refworld*b0).matrix(),1e-8));
  auto next=f.processScan(c1,2.12);ASSERT_TRUE(next.accepted)<<next.reason<<" rotation="<<next.rotation_prior_error<<" info="<<next.information_eigen_ratio;
  auto expected=refworld*b1;EXPECT_LT((next.reference_from_base.translation()-expected.translation()).norm(),.008);
  EXPECT_LT(Eigen::AngleAxisd(next.reference_from_base.rotation().transpose()*expected.rotation()).angle(),.005);
  EXPECT_GT((next.reference_from_base.translation()-first.reference_from_base.translation()).norm(),.009);
  EXPECT_NEAR(next.cumulative_yaw,.0475,.003); // trapezoid across the measured acceleration interval
  }
}
TEST(RawFrontend,StrictStampsGapAndCompleteReset){RawFrontendParams p;StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f);auto cloud=scene();ASSERT_TRUE(f.processScan(cloud,2.).accepted);EXPECT_FALSE(f.processScan(cloud,2.).accepted);std::string reason;EXPECT_FALSE(f.addImu({2.1,Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,9.81)},&reason));EXPECT_NE(reason.find("strictly"),std::string::npos);EXPECT_FALSE(f.processScan(cloud,2.05).accepted);f.reset();stationary(f);auto after=f.processScan(cloud,2.);ASSERT_TRUE(after.accepted)<<after.reason;EXPECT_DOUBLE_EQ(after.cumulative_yaw,0);f.reset();ASSERT_TRUE(f.addImu({1.,Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,9.81)}));EXPECT_FALSE(f.addImu({1.1,Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,9.81)}));}
TEST(RawFrontend,RejectsTimedMissingCoverageMotionCalibrationAndDegenerateGeometry){RawFrontendParams p;StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());auto cloud=scene();EXPECT_FALSE(f.processScan(cloud,2.,false).accepted);EXPECT_FALSE(f.processScan(cloud,2.).imu_covered);for(int j=0;j<100;++j)ASSERT_TRUE(f.addImu({j*.01,Eigen::Vector3d(0,0,.5),Eigen::Vector3d(0,0,9.81)}));EXPECT_FALSE(f.processScan(cloud,.8).gravity_initialized);f.reset();stationary(f);Cloud plane;for(int x=0;x<40;++x)for(int y=0;y<40;++y)plane.emplace_back(x*.2,y*.2,0);auto degenerate=f.processScan(plane,2.);EXPECT_FALSE(degenerate.accepted);EXPECT_NE(degenerate.reason.find("degenerate"),std::string::npos);f.reset();stationary(f);ASSERT_TRUE(f.processScan(cloud,2.).accepted);EXPECT_FALSE(f.processScan(cloud,2.3).accepted);}
TEST(RawFrontend,RejectsRealExcessTranslationInsteadOfProjectingToZero){RawFrontendParams p;p.max_iterations=30;StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,2.2);auto cloud=scene();ASSERT_TRUE(f.processScan(cloud,2.).accepted);Cloud moved;auto t=pose(0);t.translation().x()=.1;pcl::transformPointCloud(cloud,moved,t.inverse().matrix().cast<float>());auto result=f.processScan(moved,2.1);EXPECT_FALSE(result.accepted);EXPECT_GT(result.translation_step,.08);EXPECT_NE(result.reason.find("translation"),std::string::npos);}
TEST(RawFrontend,ConvergenceOnlyWarmRetryIsBoundedAndStillRequiresAllGates) {
  RawFrontendParams p;p.max_iterations=1;StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,2.2);
  auto cloud=scene();ASSERT_TRUE(f.processScan(cloud,2.).accepted);Cloud moved;auto t=pose(.005);t.translation()<<.005,.002,.001;
  pcl::transformPointCloud(cloud,moved,t.inverse().matrix().cast<float>());auto result=f.processScan(moved,2.1);
  ASSERT_TRUE(result.accepted)<<result.reason;ASSERT_EQ(result.registration_attempts.size(),2u);
  EXPECT_FALSE(result.registration_attempts[0].converged);EXPECT_TRUE(result.converged);EXPECT_EQ(result.retry_count,1);
  EXPECT_LE(result.iterations,16);EXPECT_TRUE(result.rejected_gates.empty());EXPECT_GT(result.translation_step,.002);
  f.reset();stationary(f,2.2);ASSERT_TRUE(f.processScan(cloud,2.).accepted);t.translation()<<.1,0,0;
  pcl::transformPointCloud(cloud,moved,t.inverse().matrix().cast<float>());result=f.processScan(moved,2.1);
  EXPECT_FALSE(result.accepted);EXPECT_LE(result.retry_count,1);EXPECT_LE(result.iterations,16);
  EXPECT_NE(std::find(result.rejected_gates.begin(),result.rejected_gates.end(),"linear_speed"),result.rejected_gates.end());
}
TEST(RawFrontend,RetryCannotRescueFailedQualityOrNonfiniteTransform) {
  RawFrontendParams p;RawRegistrationAttempt a;a.transform_finite=true;a.fitness=.05;a.inlier_ratio=.95;a.information_eigen_ratio=.01;
  EXPECT_TRUE(rawRegistrationRetryEligible(a,p));a.converged=true;EXPECT_FALSE(rawRegistrationRetryEligible(a,p));a.converged=false;
  a.transform_finite=false;EXPECT_FALSE(rawRegistrationRetryEligible(a,p));a.transform_finite=true;
  a.fitness=.121;EXPECT_FALSE(rawRegistrationRetryEligible(a,p));a.fitness=.05;
  a.inlier_ratio=.649;EXPECT_FALSE(rawRegistrationRetryEligible(a,p));a.inlier_ratio=.95;
  a.information_eigen_ratio=1e-7;EXPECT_FALSE(rawRegistrationRetryEligible(a,p));a.information_eigen_ratio=.01;
  a.fitness=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(rawRegistrationRetryEligible(a,p));
}

#include "global_relocalization/raw_preprocessing.hpp"
#include "global_relocalization/raw_registration.hpp"
#include <filesystem>
TEST(RawFrontend,RangeFilteredSensorCloudIsSharedWithAccumulationAndConfirmation) {
  RawFrontendParams p;p.range_filter=true;p.deterministic=true;
  StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f);
  auto cloud=scene();cloud.emplace_back(.1,0,0);cloud.emplace_back(31,0,0);cloud.emplace_back(std::numeric_limits<float>::quiet_NaN(),0,0);
  auto r=f.processScan(cloud,2.);ASSERT_TRUE(r.accepted)<<r.reason;EXPECT_EQ(r.input_points,cloud.size());EXPECT_EQ(r.finite_points,cloud.size()-1);
  EXPECT_EQ(r.range_points,r.filtered_sensor_cloud.size());EXPECT_LT(r.range_points,r.finite_points);
  for(auto&q:r.filtered_sensor_cloud){double d=Eigen::Vector3d(q.x,q.y,q.z).norm();EXPECT_GE(d,.5);EXPECT_LE(d,30.);}
}
TEST(RawFrontend,DeterministicVoxelIgnoresInputOrder) {
  auto cloud=scene();auto a=deterministicVoxel(cloud,.25);std::reverse(cloud.begin(),cloud.end());auto b=deterministicVoxel(cloud,.25);
  ASSERT_EQ(a->size(),b->size());for(size_t k=0;k<a->size();++k){EXPECT_EQ((*a)[k].x,(*b)[k].x);EXPECT_EQ((*a)[k].y,(*b)[k].y);EXPECT_EQ((*a)[k].z,(*b)[k].z);}
}
TEST(RawFrontend,RotationPriorHasRightIncrementJacobianAndCandidateCost) {
  RawRotationPrior prior;prior.enabled=true;prior.inverse_variance=100.;prior.rotation=pose(.2,-.1,.05).rotation();auto t=pose(.23,-.09,.04);
  auto j=prior.jacobian(t);Eigen::Vector3d d(.003,-.002,.001);auto moved=t;moved.linear()=t.rotation()*Eigen::AngleAxisd(d.norm(),d.normalized()).toRotationMatrix();
  EXPECT_LT((prior.residual(moved)-prior.residual(t)-j*d).norm(),1e-6);
  Eigen::Matrix<double,6,6> h=Eigen::Matrix<double,6,6>::Zero();Eigen::Matrix<double,6,1>b=Eigen::Matrix<double,6,1>::Zero();double e=0;prior.linearize(t,h,b,e);EXPECT_DOUBLE_EQ(e,prior.cost(t));
  for(int k=0;k<3;++k){auto plus=t,minus=t;plus.linear()=t.rotation()*Eigen::AngleAxisd(1e-7,Eigen::Vector3d::Unit(k)).toRotationMatrix();minus.linear()=t.rotation()*Eigen::AngleAxisd(-1e-7,Eigen::Vector3d::Unit(k)).toRotationMatrix();EXPECT_NEAR((prior.cost(plus)-prior.cost(minus))/2e-7,b(k),1e-6);}
  Eigen::Vector3d step=-h.topLeftCorner<3,3>().ldlt().solve(b.head<3>());auto improved=t;improved.linear()=t.rotation()*Eigen::AngleAxisd(step.norm(),step.normalized()).toRotationMatrix();EXPECT_LT(prior.cost(improved),e);EXPECT_TRUE((h.bottomRightCorner<3,3>().isZero()));
}
TEST(RawFrontend,SubmapContainsOnlyAcceptedScansAndHardFaultCannotRecover) {
  RawFrontendParams p;p.range_filter=true;p.deterministic=true;p.submap_scans=5;p.imu_rotation_prior=true;p.numerical_recovery=true;
  StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,3.);
  auto cloud=scene();for(int j=0;j<7;++j){auto r=f.processScan(cloud,2.+j*.1);ASSERT_TRUE(r.accepted)<<r.reason;EXPECT_EQ(r.submap_size,std::min(j+1,5));}
  Cloud moved;pcl::transformPointCloud(cloud,moved,pose(0).matrix().cast<float>());for(auto&q:moved)q.x-=.1;
  auto bad=f.processScan(moved,2.7);EXPECT_FALSE(bad.accepted);EXPECT_EQ(bad.failure_class,"hard_fault");EXPECT_EQ(bad.submap_size,5u);EXPECT_EQ(bad.recovery_attempts,0u);
  f.authorizeRecoveryAfterStop(2.7);auto after=f.processScan(cloud,2.8);EXPECT_FALSE(after.accepted);EXPECT_NE(after.reason.find("latched hard fault"),std::string::npos);
}
TEST(RawFrontend,SubmapWindowAllowsOneFastRevolutionAndRejectsLonger) {
  RawFrontendParams p;p.deterministic=true;p.submap_scans=20;
  StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,4.3);
  auto cloud=scene();
  for(int j=0;j<22;++j){auto r=f.processScan(cloud,2.+j*.1);ASSERT_TRUE(r.accepted)<<r.reason;EXPECT_EQ(r.submap_size,std::min(j+1,20));}
  p.submap_scans=31;EXPECT_THROW(StationaryRawFrontend(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity()),std::invalid_argument);
}
TEST(RawFrontend,RecoveryRequiresPublishedStopThenOneNewScanAndPreservesTrustedPose) {
  RawFrontendParams p;p.deterministic=true;p.numerical_recovery=true;p.max_iterations=1;p.retry_iterations=0;
  StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,2.5);auto cloud=scene();auto first=f.processScan(cloud,2.);ASSERT_TRUE(first.accepted);
  auto t=pose(.005);t.translation()<<.005,.002,.001;Cloud moved;pcl::transformPointCloud(cloud,moved,t.inverse().matrix().cast<float>());
  auto failed=f.processScan(moved,2.1);ASSERT_EQ(failed.failure_class,"numerical_recoverable")<<failed.reason;EXPECT_EQ(failed.recovery_attempts,1u);EXPECT_TRUE(failed.reference_from_sensor.matrix().isApprox(first.reference_from_sensor.matrix()));EXPECT_EQ(failed.submap_size,1u);
  auto before_stop=f.processScan(cloud,2.2);EXPECT_FALSE(before_stop.accepted);EXPECT_EQ(before_stop.failure_class,"waiting");f.authorizeRecoveryAfterStop(2.2);
  auto old=f.processScan(cloud,2.2);EXPECT_FALSE(old.accepted);auto recovered=f.processScan(cloud,2.3);ASSERT_TRUE(recovered.accepted)<<recovered.reason;EXPECT_EQ(recovered.recovered_scan_count,1u);EXPECT_EQ(recovered.recovery_attempts,1u);
  failed=f.processScan(moved,2.4);EXPECT_FALSE(failed.accepted);EXPECT_EQ(failed.failure_class,"hard_fault");EXPECT_EQ(failed.recovery_attempts,1u);
}
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
TEST(RawFrontend,OptionalDiagnosticRecordsExactInputsAndPriorObjectiveTrials) {
  auto dir=std::filesystem::temp_directory_path()/("raw_frontend_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  RawFrontendParams p;p.deterministic=true;p.imu_rotation_prior=true;p.diagnostic_directory=dir.string();
  StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,2.2);auto cloud=scene();ASSERT_TRUE(f.processScan(cloud,2.).accepted);
  auto t=pose(.005);t.translation()<<.005,.002,.001;Cloud moved;pcl::transformPointCloud(cloud,moved,t.inverse().matrix().cast<float>());auto r=f.processScan(moved,2.1);ASSERT_TRUE(r.accepted)<<r.reason;
  ASSERT_FALSE(r.iteration_trace.empty());for(auto & row:r.iteration_trace)if(row.accepted)EXPECT_LE(row.candidate_objective,row.objective);
  nlohmann::json j;std::ifstream(dir/"000001.json")>>j;EXPECT_EQ(j["imu"].size(),221u);EXPECT_TRUE(j.contains("initial_guess"));EXPECT_EQ(j["params"]["imu_rotation_prior"],true);EXPECT_EQ(j["iterations"].size(),r.iteration_trace.size());EXPECT_TRUE(std::filesystem::exists(dir/"000001_target.pcd"));
  std::filesystem::remove_all(dir);
}
TEST(RawFrontend,RecoveryNeverBridgesContinuityGapOrAcceptsInvalidExtrinsics) {
  RawFrontendParams p;p.deterministic=true;p.numerical_recovery=true;p.max_iterations=1;p.retry_iterations=0;
  auto reflection=Eigen::Isometry3d::Identity();reflection.linear()(0,0)=-1;EXPECT_THROW(StationaryRawFrontend(p,reflection,Eigen::Isometry3d::Identity()),std::invalid_argument);
  StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,2.5);auto cloud=scene();ASSERT_TRUE(f.processScan(cloud,2.).accepted);
  auto t=pose(.005);t.translation()<<.005,.002,.001;Cloud moved;pcl::transformPointCloud(cloud,moved,t.inverse().matrix().cast<float>());ASSERT_EQ(f.processScan(moved,2.1).failure_class,"numerical_recoverable");
  f.authorizeRecoveryAfterStop(2.1);auto r=f.processScan(cloud,2.4);EXPECT_FALSE(r.accepted);EXPECT_EQ(r.failure_class,"hard_fault");EXPECT_EQ(r.reason,"raw scan coverage gap");EXPECT_FALSE(r.recovery_pending);EXPECT_EQ(r.recovered_scan_count,0u);EXPECT_EQ(r.submap_size,1u);
}
