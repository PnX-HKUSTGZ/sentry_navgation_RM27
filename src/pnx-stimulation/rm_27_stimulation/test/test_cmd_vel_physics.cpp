#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>
#include <gz/math/Vector3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

TEST(CmdVelPhysics, ForceAndTorqueReachPhysicsAndExpireWithoutNewCommands) {
  // Use the actual plugin and Bullet engine. A free body isolates actuation
  // scheduling from CAD contact geometry, friction and the navigation stack.
  const auto partition = "rm27_cmd_vel_physics_" + std::to_string(getpid());
  setenv("GZ_PARTITION", partition.c_str(), 1);
  gz::sim::ServerConfig config;
  config.SetPhysicsEngine("gz-physics-bullet-featherstone-plugin");
  ASSERT_TRUE(config.SetSdfString(
      "<sdf version='1.9'><world name='actuation_test'>"
      "<gravity>0 0 0</gravity>"
      "<physics name='physics' type='ignored'><max_step_size>0.001</max_step_size>"
      "<real_time_factor>0</real_time_factor></physics>"
      "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>"
      "<model name='body'><link name='body_link'>"
      "<inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy>"
      "<izz>0.1</izz></inertia></inertial>"
      "<collision name='body_collision'><geometry><box><size>0.1 0.1 0.1</size>"
      "</box></geometry></collision></link>"
      "<plugin filename='" + std::string(RM27_ACTUATOR_PLUGIN) +
      "' name='rm_27_stimulation::CmdVelPoseControlSystem'>"
      "<topic>/test/cmd_vel</topic><body_frame>false</body_frame>"
      "<command_timeout>0.5</command_timeout></plugin>"
      "</model></world></sdf>"));
  gz::sim::TestFixture fixture(config);
  gz::math::Vector3d linear, angular;
  bool observed = false;
  fixture.OnPostUpdate([&](const gz::sim::UpdateInfo &,
                           const gz::sim::EntityComponentManager &ecm) {
    const auto entity = ecm.EntityByComponents(gz::sim::components::Link(),
                                              gz::sim::components::Name("body_link"));
    const gz::sim::Link link(entity);
    const auto v = link.WorldLinearVelocity(ecm);
    const auto w = link.WorldAngularVelocity(ecm);
    if (v && w) {
      linear = *v;
      angular = *w;
      observed = true;
    }
  }).Finalize();
  ASSERT_TRUE(fixture.Server()->Run(true, 1, false));
  // Global initialization intentionally rests before sending the first turn.
  ASSERT_TRUE(fixture.Server()->Run(true, 2000, false));
  gz::transport::Node node;
  auto publisher = node.Advertise<gz::msgs::Twist>("/test/cmd_vel");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!publisher.HasConnections() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(publisher.HasConnections());
  gz::msgs::Twist command;
  command.mutable_linear()->set_x(0.25);
  command.mutable_angular()->set_z(1.0);
  ASSERT_TRUE(publisher.Publish(command));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_TRUE(fixture.Server()->Run(true, 200, false));
  ASSERT_TRUE(observed);
  EXPECT_GT(linear.X(), 0.20);
  EXPECT_GT(angular.Z(), 0.8);
  EXPECT_LT(linear.Length(), 0.4);
  EXPECT_LT(angular.Length(), 1.2);
  // Stop through the existing command watchdog and physical braking torque.
  ASSERT_TRUE(fixture.Server()->Run(true, 1000, false));
  EXPECT_NEAR(linear.Length(), 0.0, 0.01);
  EXPECT_NEAR(angular.Length(), 0.0, 0.01);
}

TEST(CmdVelPhysics, RestingCadRobotTurnsBothWaysAndStopsOnFlatGround) {
  const auto partition = "rm27_cad_rotation_" + std::to_string(getpid());
  setenv("GZ_PARTITION", partition.c_str(), 1);
  const auto resources = std::string(RM27_RESOURCE_PARENT) + ":" +
      (std::getenv("GZ_SIM_RESOURCE_PATH") ? std::getenv("GZ_SIM_RESOURCE_PATH") : "");
  setenv("GZ_SIM_RESOURCE_PATH", resources.c_str(), 1);
  gz::sim::ServerConfig config;
  config.SetPhysicsEngine("gz-physics-bullet-featherstone-plugin");
  ASSERT_TRUE(config.SetSdfFile(RM27_ACTUATION_WORLD));
  gz::sim::TestFixture fixture(config);
  gz::math::Vector3d angular;
  gz::math::Pose3d pose;
  bool observed = false;
  fixture.OnPostUpdate([&](const gz::sim::UpdateInfo &,
                           const gz::sim::EntityComponentManager &ecm) {
    const auto entity = ecm.EntityByComponents(gz::sim::components::Link(),
        gz::sim::components::Name("base_footprint"));
    const gz::sim::Link link(entity);
    if (const auto velocity = link.WorldAngularVelocity(ecm)) {
      angular = *velocity;
      pose = gz::sim::worldPose(entity, ecm);
      observed = true;
    }
  }).Finalize();
  ASSERT_TRUE(fixture.Server()->Run(true, 2000, false));
  ASSERT_TRUE(observed);
  EXPECT_LT(angular.Length(), 0.01);
  gz::transport::Node node;
  auto publisher = node.Advertise<gz::msgs::Twist>("/cmd_vel");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!publisher.HasConnections() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(publisher.HasConnections());
  // Heading restoration caps the requested rate at 0.6 rad/s. The original
  // P-only actuator stayed stationary at that command despite a valid bridge.
  const auto headingOrigin = pose.Rot().Yaw();
  gz::msgs::Twist slow;
  slow.mutable_angular()->set_z(0.6);
  for (int step = 0; step < 30; ++step) {
    ASSERT_TRUE(publisher.Publish(slow));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_TRUE(fixture.Server()->Run(true, 100, false));
  }
  EXPECT_GT(angular.Z(), 0.3);
  EXPECT_GT(std::atan2(std::sin(pose.Rot().Yaw() - headingOrigin),
                       std::cos(pose.Rot().Yaw() - headingOrigin)), 0.2);
  // Explicit zero must clear stored torque, independently of the watchdog.
  ASSERT_TRUE(publisher.Publish(gz::msgs::Twist{}));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(fixture.Server()->Run(true, 500, false));
  EXPECT_LT(angular.Length(), 0.03);
  for (const double direction : {1.0, -1.0}) {
    const auto origin = pose.Pos();
    auto previousYaw = pose.Rot().Yaw();
    double measuredTurn = 0.0;
    gz::msgs::Twist command;
    command.mutable_angular()->set_z(direction * 2.25);
    // The global protocol needs an actual full revolution, not an arbitrarily
    // small angular response or exact velocity tracking against contact drag.
    const double fullTurn = 2.0 * std::acos(-1.0);
    for (int step = 0; step < 120 && measuredTurn < fullTurn; ++step) {
      ASSERT_TRUE(publisher.Publish(command));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ASSERT_TRUE(fixture.Server()->Run(true, 100, false));
      const auto yaw = pose.Rot().Yaw();
      measuredTurn += direction * std::atan2(std::sin(yaw - previousYaw),
                                            std::cos(yaw - previousYaw));
      previousYaw = yaw;
    }
    EXPECT_GE(measuredTurn, fullTurn);
    EXPECT_GT(direction * angular.Z(), 0.0);
    EXPECT_LT(angular.Length(), 3.0);
    EXPECT_LT((pose.Pos() - origin).Length(), 0.1);
    EXPECT_GT(pose.Pos().Z(), 0.075);
    EXPECT_LT(pose.Pos().Z(), 0.090);
    ASSERT_TRUE(fixture.Server()->Run(true, 1500, false));
    EXPECT_LT(angular.Length(), 0.03);
  }
}
