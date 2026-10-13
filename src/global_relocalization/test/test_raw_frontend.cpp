// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/stationary_raw_frontend.hpp"
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <random>
using namespace global_relocalization;
namespace {
Cloud scene(){Cloud cloud;std::mt19937 rng(27);std::uniform_real_distribution<float>d(-4,4);for(int j=0;j<4000;++j)cloud.emplace_back(d(rng),d(rng),d(rng)*.4);return cloud;}
Eigen::Isometry3d pose(double yaw,double pitch=0,double roll=0){Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();return t;}
void stationary(StationaryRawFrontend & f,double end=2.1){for(int j=0;j<=int(end*100+.1);++j)ASSERT_TRUE(f.addImu({j*.01,Eigen::Vector3d(.002,-.001,.003),Eigen::Vector3d(0,0,9.81)}));}
}
TEST(RawFrontend,FullSixDofMotionGravityAndNonzeroImuSensorBaseExtrinsics){
  RawFrontendParams p;p.stationary_duration=2.;p.max_iterations=30;p.max_fitness=.04;
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
TEST(RawFrontend,StrictStampsGapAndCompleteReset){RawFrontendParams p;StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f);auto cloud=scene();ASSERT_TRUE(f.processScan(cloud,2.).accepted);EXPECT_FALSE(f.processScan(cloud,2.).accepted);std::string reason;EXPECT_FALSE(f.addImu({2.1,Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,9.81)},&reason));EXPECT_NE(reason.find("strictly"),std::string::npos);EXPECT_FALSE(f.processScan(cloud,2.05).accepted);f.reset();stationary(f);auto after=f.processScan(cloud,2.);ASSERT_TRUE(after.accepted)<<after.reason;EXPECT_DOUBLE_EQ(after.cumulative_yaw,0);f.reset();ASSERT_TRUE(f.addImu({1.,Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,9.81)}));EXPECT_FALSE(f.addImu({1.1,Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,9.81)}));}
TEST(RawFrontend,RejectsTimedMissingCoverageMotionCalibrationAndDegenerateGeometry){RawFrontendParams p;StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());auto cloud=scene();EXPECT_FALSE(f.processScan(cloud,2.,false).accepted);EXPECT_FALSE(f.processScan(cloud,2.).imu_covered);for(int j=0;j<100;++j)ASSERT_TRUE(f.addImu({j*.01,Eigen::Vector3d(0,0,.5),Eigen::Vector3d(0,0,9.81)}));EXPECT_FALSE(f.processScan(cloud,.8).gravity_initialized);f.reset();stationary(f);Cloud plane;for(int x=0;x<40;++x)for(int y=0;y<40;++y)plane.emplace_back(x*.2,y*.2,0);auto degenerate=f.processScan(plane,2.);EXPECT_FALSE(degenerate.accepted);EXPECT_NE(degenerate.reason.find("degenerate"),std::string::npos);f.reset();stationary(f);ASSERT_TRUE(f.processScan(cloud,2.).accepted);EXPECT_FALSE(f.processScan(cloud,2.3).accepted);}
TEST(RawFrontend,RejectsRealExcessTranslationInsteadOfProjectingToZero){RawFrontendParams p;p.max_iterations=30;StationaryRawFrontend f(p,Eigen::Isometry3d::Identity(),Eigen::Isometry3d::Identity());stationary(f,2.2);auto cloud=scene();ASSERT_TRUE(f.processScan(cloud,2.).accepted);Cloud moved;auto t=pose(0);t.translation().x()=.1;pcl::transformPointCloud(cloud,moved,t.inverse().matrix().cast<float>());auto result=f.processScan(moved,2.1);EXPECT_FALSE(result.accepted);EXPECT_GT(result.translation_step,.08);EXPECT_NE(result.reason.find("translation"),std::string::npos);}
