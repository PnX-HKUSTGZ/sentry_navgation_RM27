#include <gtest/gtest.h>
#include <limits>
#include "rm_27_stimulation/pose_imu.hpp"

using namespace rm_27_stimulation;
using namespace std::chrono_literals;
const gz::math::Vector3d gravity(0, 0, -9.81);

TEST(PoseImu, RestAtTiltedMount) {
  const gz::math::Pose3d pose(1, 2, 3, -M_PI/6, 0, 0);
  auto m = MakePoseImu({0ms, pose}, {5ms, pose}, {10ms, pose}, gravity, "imu_link");
  ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->angular_velocity().x(), 0);
  EXPECT_DOUBLE_EQ(m->angular_velocity().y(), 0);
  EXPECT_DOUBLE_EQ(m->angular_velocity().z(), 0);
  EXPECT_NEAR(m->linear_acceleration().y(), -4.905, 1e-10);
  EXPECT_NEAR(m->linear_acceleration().z(), 9.81*std::cos(M_PI/6), 1e-10);
  EXPECT_EQ(m->header().stamp().nsec(), 5000000);
  EXPECT_EQ(m->header().data(0).value(0), "imu_link");
}

TEST(PoseImu, RotatingOffsetSensorIncludesCentripetalAcceleration) {
  const double omega = 1.5;
  const gz::math::Pose3d mount(0, .126, .130, -M_PI/6, 0, 0);
  auto pose = [&](double t) {
    return gz::math::Pose3d(0, 0, 0, 0, 0, omega*t) * mount;
  };
  auto m = MakePoseImu({0ms, pose(0)}, {5ms, pose(.005)}, {10ms, pose(.01)}, gravity, "imu");
  ASSERT_TRUE(m);
  auto expectedGyro = mount.Rot().RotateVectorReverse({0, 0, omega});
  auto expectedAcc = mount.Rot().RotateVectorReverse({0, -.126*omega*omega, 9.81});
  EXPECT_NEAR(m->angular_velocity().y(), expectedGyro.Y(), 1e-10);
  EXPECT_NEAR(m->angular_velocity().z(), expectedGyro.Z(), 1e-10);
  EXPECT_NEAR(m->linear_acceleration().y(), expectedAcc.Y(), 2e-6);
  EXPECT_NEAR(m->linear_acceleration().z(), expectedAcc.Z(), 2e-6);
}

TEST(PoseImu, LinearAccelerationWithUnequalSampleIntervals) {
  auto pose = [](double t) { return gz::math::Pose3d(t*t, -2*t*t, 0, 0, 0, 0); };
  auto m = MakePoseImu({0ms, pose(0)}, {5ms, pose(.005)}, {11ms, pose(.011)}, gravity, "imu");
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->linear_acceleration().x(), 2, 1e-10);
  EXPECT_NEAR(m->linear_acceleration().y(), -4, 1e-10);
  EXPECT_NEAR(m->linear_acceleration().z(), 9.81, 1e-10);
}

TEST(PoseImu, QuaternionSignAndYawWrapDoNotProduceSpikes) {
  const gz::math::Pose3d before(0, 0, 0, 0, 0, M_PI-.005);
  const gz::math::Pose3d middle(0, 0, 0, 0, 0, M_PI);
  auto after = gz::math::Pose3d(0, 0, 0, 0, 0, -M_PI+.005);
  auto q = after.Rot();
  after.Rot() = gz::math::Quaterniond(-q.W(), -q.X(), -q.Y(), -q.Z());
  auto m = MakePoseImu({0ms, before}, {5ms, middle}, {10ms, after}, gravity, "imu");
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->angular_velocity().z(), 1, 1e-10);
}

TEST(PoseImu, RejectsBadTimeAndInvalidPose) {
  ImuPoseSample a{0ms, {}}, b{5ms, {}}, c{10ms, {}};
  EXPECT_FALSE(MakePoseImu(a, a, c, gravity, "imu"));
  EXPECT_FALSE(MakePoseImu(c, b, a, gravity, "imu"));
  c.pose.Pos().X(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(MakePoseImu(a, b, c, gravity, "imu"));
}
