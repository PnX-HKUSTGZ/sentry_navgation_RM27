#include <gtest/gtest.h>
#include "startup_handover.hpp"
#include "startup_scenarios.hpp"

TEST(JointStartup, CovarianceJacobianMatchesIndependentRetraction)
{
  const auto scenario = startup_test::scenarios()[3];
  const auto state = startup_test::truth(scenario,1.3);
  zlio::Mat15 numerical;
  for (int axis = 0; axis < 15; ++axis) {
    zlio::Vec15 delta = zlio::Vec15::Zero(); delta[axis] = 1e-6;
    const auto plus = zlio::retract(state, delta), minus = zlio::retract(state, -delta);
    zlio::Vec15 d;
    d.segment<3>(0) = plus.pose.position-minus.pose.position;
    d.segment<3>(3) = zlio::logRotation(state.pose.rotation.conjugate()*plus.pose.rotation) -
                      zlio::logRotation(state.pose.rotation.conjugate()*minus.pose.rotation);
    d.segment<3>(6) = plus.velocity-minus.velocity;
    d.segment<3>(9) = plus.gyro_bias-minus.gyro_bias;
    d.segment<3>(12) = plus.accel_bias-minus.accel_bias;
    numerical.col(axis) = d / 2e-6;
  }
  EXPECT_TRUE(numerical.isApprox(point_lio::startupErrorJacobian(state),1e-7));
}

TEST(JointStartup, StaticAndMovingScansHandoverFiniteStateMapAndCovariance)
{
  for (int index : {0, 1, 3, 4, 5}) {
    const auto scenario = startup_test::scenarios()[index];
    SCOPED_TRACE(scenario.name);
    point_lio::JointStartup startup(startup_test::config(scenario));
    std::optional<point_lio::StartupSeed> seed;
    for (int frame = 0; frame < 48 && !seed; ++frame) {
      auto p = startup_test::packet(scenario,frame,1);
      seed = startup.add(p.scan,p.begin,p.end,p.imus);
    }
    ASSERT_TRUE(seed.has_value()) << startup.reason();
    const auto actual = startup_test::truth(scenario,zlio::seconds(seed->boundary_imu.time)-startup_test::epoch);
    EXPECT_LT(startup_test::tiltError(seed->solution.estimate.state,actual),2.0);
    EXPECT_LT(startup_test::speedError(seed->solution.estimate.state,actual),.15);
    EXPECT_GT(seed->solution.map.size(),100U);
    esekfom::esekf<state_output,30,input_ikfom> out;
    esekfom::esekf<state_input,24,input_ikfom> in;
    point_lio::installStartupSeed(*seed,scenario.extrinsic.rotation.toRotationMatrix(),
                                scenario.extrinsic.position,out,in);
    EXPECT_TRUE(out.x_.vel.isApprox(seed->solution.estimate.state.velocity,1e-12));
    EXPECT_TRUE(in.x_.pos.isApprox(out.x_.pos,1e-12));
    EXPECT_EQ(Eigen::LLT<decltype(out.P_)>(out.P_).info(),Eigen::Success);
    EXPECT_EQ(Eigen::LLT<decltype(in.P_)>(in.P_).info(),Eigen::Success);
    EXPECT_LT((out.x_.omg + out.x_.bg - seed->boundary_imu.angular_velocity).norm(),1e-12);
    EXPECT_LT((out.x_.acc + out.x_.ba - seed->boundary_imu.acceleration).norm(),1e-12);
  }
}

TEST(JointStartup, DegenerateGeometryDoesNotInitialize)
{
  const auto scenario = startup_test::scenarios().back();
  point_lio::JointStartup startup(startup_test::config(scenario));
  for (int frame = 0; frame < 45; ++frame) {
    auto p = startup_test::packet(scenario,frame,1);
    EXPECT_FALSE(startup.add(p.scan,p.begin,p.end,p.imus));
  }
  EXPECT_NE(startup.phase(),"READY");
}

TEST(JointStartup, InvalidImuAndTimeRollbackClearTheStartupWindow)
{
  const auto scenario = startup_test::scenarios()[0];
  point_lio::JointStartup startup(startup_test::config(scenario));
  auto p = startup_test::packet(scenario,0,1);
  p.imus[3].acceleration.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(startup.add(p.scan,p.begin,p.end,p.imus));
  EXPECT_EQ(startup.phase(),"RETRYING");
  EXPECT_EQ(startup.acceptedPackets(),0U);
  p = startup_test::packet(scenario,1,1);
  EXPECT_FALSE(startup.add(p.scan,p.begin,p.end,p.imus));
  p = startup_test::packet(scenario,0,1);
  EXPECT_FALSE(startup.add(p.scan,p.begin,p.end,p.imus));
  EXPECT_EQ(startup.phase(),"RETRYING");
  EXPECT_EQ(startup.acceptedPackets(),0U);
  startup.reset();
  EXPECT_EQ(startup.phase(),"COLLECTING");
}

TEST(JointStartup, RejectsSaturationGapAndBadConfiguration)
{
  const auto scenario = startup_test::scenarios()[0];
  for (int failure=0; failure<2; ++failure) {
    point_lio::JointStartup startup(startup_test::config(scenario));
    auto p=startup_test::packet(scenario,0,1);
    if (failure==0) {p.imus[3].angular_velocity.z()=40;}
    else {p.imus.erase(p.imus.begin()+2,p.imus.begin()+12);}
    EXPECT_FALSE(startup.add(p.scan,p.begin,p.end,p.imus));
    EXPECT_EQ(startup.phase(),"RETRYING");
  }
  auto c=startup_test::config(scenario); c.solver.initialization.window_frames=1000;
  EXPECT_THROW(point_lio::JointStartup invalid(c),std::invalid_argument);
}

TEST(JointStartup, RawGUnitsMatchSiInitializationAndRepeatedEndpointIsNotConsumedTwice)
{
  const auto scenario=startup_test::scenarios()[1];
  auto c=startup_test::config(scenario);
  c.acceleration_scale=9.81;
  c.acceleration_limit=4;
  point_lio::JointStartup startup(c);
  std::optional<point_lio::StartupSeed> seed;
  for (int frame=0;frame<30 && !seed;++frame) {
    auto p=startup_test::packet(scenario,frame,1);
    for (auto & imu:p.imus) {imu.acceleration/=9.81;}
    seed=startup.add(p.scan,p.begin,p.end,p.imus);
  }
  ASSERT_TRUE(seed) << startup.reason();
  EXPECT_NEAR(seed->specific_force.norm(),9.81,1e-6);
  EXPECT_LT(startup_test::tiltError(seed->solution.estimate.state,
                                  startup_test::truth(scenario,2.2)),1e-4);
}

TEST(JointStartup, RejectsUnorderedImuAndInvalidPointTimeBeforeOptimization)
{
  const auto scenario=startup_test::scenarios()[0];
  for (int failure=0;failure<2;++failure) {
    point_lio::JointStartup startup(startup_test::config(scenario));
    auto p=startup_test::packet(scenario,0,1);
    if (failure==0) {std::swap(p.imus[3],p.imus[4]);}
    else {p.scan[0].time=p.end+1;}
    EXPECT_FALSE(startup.add(p.scan,p.begin,p.end,p.imus));
    EXPECT_EQ(startup.phase(),"RETRYING");
    EXPECT_EQ(startup.acceptedPackets(),0U);
  }
}
