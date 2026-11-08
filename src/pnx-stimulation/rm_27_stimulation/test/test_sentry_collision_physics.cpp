// Exercise the spawn representation and actual Bullet Featherstone contact
// path.
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

TEST(SentryCollision, EmptyClearancesAndSolidSurfacesUseRealBulletContacts) {
  const auto partition = "sentry_collision_test_" + std::to_string(getpid());
  setenv("GZ_PARTITION", partition.c_str(), 1);
  gz::sim::ServerConfig config;
  config.SetPhysicsEngine("gz-physics-bullet-featherstone-plugin");
  ASSERT_TRUE(config.SetSdfFile(RM27_COLLISION_TEST_WORLD));
  gz::sim::TestFixture fixture(config);
  std::map<std::string, int> contacts;
  std::set<std::string> sensors;
  bool finite = true;
  fixture
      .OnPostUpdate([&](const gz::sim::UpdateInfo &,
                        const gz::sim::EntityComponentManager &ecm) {
        ecm.Each<gz::sim::components::ContactSensor, gz::sim::components::Name>(
            [&](const auto &, const auto *, const auto *name) {
              sensors.insert(name->Data());
              return true;
            });
        ecm.Each<gz::sim::components::ContactSensorData,
                 gz::sim::components::Name>(
            [&](const auto &, const auto *data, const auto *name) {
              contacts[name->Data()] += data->Data().contact_size();
              return true;
            });
        ecm.Each<gz::sim::components::Pose>(
            [&](const auto &, const auto *pose) {
              finite = finite && pose->Data().IsFinite();
              return true;
            });
      })
      .Finalize();
  ASSERT_TRUE(fixture.Server()->Run(true, 3, false));
  ASSERT_TRUE(finite);
  // ContactSensorData lives on collision entities after contact; also verify
  // sensor entities in empty cases. Positive controls reject disabled
  // collision.
  for (const auto &name : {"air_gap_0", "solid_0", "roof_clear_0",
                           "roof_clear_1", "roof_blocked_0", "roof_blocked_1"})
    ASSERT_TRUE(sensors.count(name)) << name;
  EXPECT_EQ(contacts["air_gap_collision0"], 0);
  EXPECT_GT(contacts["solid_collision0"], 0);
  EXPECT_EQ(
      contacts["roof_clear_collision0"] + contacts["roof_clear_collision1"], 0);
  EXPECT_GT(contacts["roof_blocked_collision0"] +
                contacts["roof_blocked_collision1"],
            0);
}
