#include <chrono>
#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "rm_27_stimulation/ground_truth_odometry.hpp"

namespace rm_27_stimulation {

class GroundTruthOdometrySystem final : public gz::sim::System,
                                        public gz::sim::ISystemConfigure,
                                        public gz::sim::ISystemPostUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "GroundTruthOdometrySystem requires a model.\n";
      return;
    }
    this->link_ = gz::sim::Link(model.CanonicalLink(ecm));
    if (!this->link_.Valid(ecm)) {
      gzerr << "GroundTruthOdometrySystem requires a canonical link.\n";
      return;
    }
    const auto frequency = sdf->Get<double>("odom_publish_frequency", 50.0).first;
    if (!std::isfinite(frequency) || frequency <= 0.0) {
      gzerr << "GroundTruthOdometrySystem requires a positive frequency.\n";
      return;
    }
    this->period_ = std::chrono::duration<double>(1.0 / frequency);
    this->odomFrame_ = sdf->Get<std::string>("odom_frame", "world").first;
    this->baseFrame_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
    const auto topic = sdf->Get<std::string>("odom_topic", "/ground_truth/odometry").first;
    this->publisher_ = this->node_.Advertise<gz::msgs::Odometry>(topic);
    this->link_.EnableVelocityChecks(ecm, true);
    this->configured_ = static_cast<bool>(this->publisher_);
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (!this->configured_ || info.paused) {
      return;
    }
    if (info.simTime < this->lastPublish_) {
      this->lastPublish_ = info.simTime;
    }
    if (info.simTime - this->lastPublish_ < this->period_) {
      return;
    }
    this->lastPublish_ = info.simTime;
    const auto linear = this->link_.WorldLinearVelocity(ecm);
    const auto angular = this->link_.WorldAngularVelocity(ecm);
    if (!linear || !angular) {
      return;
    }
    // Physics already supplies these velocities. Differencing quaternion poses
    // can generate NaN angular rates while resting at a tilted contact.
    const auto message = MakeGroundTruthOdometry(
        gz::sim::worldPose(this->link_.Entity(), ecm), *linear, *angular,
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime),
        this->odomFrame_, this->baseFrame_);
    if (message) {
      this->publisher_.Publish(*message);
    }
  }

private:
  gz::sim::Link link_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::string odomFrame_;
  std::string baseFrame_;
  std::chrono::duration<double> period_{0.02};
  std::chrono::steady_clock::duration lastPublish_{};
  bool configured_{false};
};

} // namespace rm_27_stimulation

GZ_ADD_PLUGIN(rm_27_stimulation::GroundTruthOdometrySystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(rm_27_stimulation::GroundTruthOdometrySystem,
                    "rm_27_stimulation::GroundTruthOdometrySystem")
