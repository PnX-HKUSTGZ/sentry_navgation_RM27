#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include "rm_27_stimulation/pose_imu.hpp"

namespace rm_27_stimulation {

// Bullet Featherstone's non-base link velocity cache can remain zero. Sample
// the actual sensor pose instead; no localization output or command is used.
class PoseImuSystem final : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPostUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    const gz::sim::Model model(entity);
    const auto link = sdf->Get<std::string>("parent_link", "gimbal_yaw").first;
    link_ = model.LinkByName(ecm, link);
    const auto frequency = sdf->Get<double>("update_rate", 200.0).first;
    if (link_ == gz::sim::kNullEntity || !std::isfinite(frequency) || frequency <= 0.0) {
      gzerr << "PoseImuSystem requires a valid parent link and positive update_rate.\n";
      return;
    }
    period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / frequency));
    if (period_.count() <= 0) {
      gzerr << "PoseImuSystem update_rate is too high.\n";
      return;
    }
    mount_ = sdf->Get<gz::math::Pose3d>("sensor_pose", gz::math::Pose3d::Zero).first;
    frame_ = sdf->Get<std::string>("frame_id", "imu_link").first;
    const auto world = ecm.EntityByComponents(gz::sim::components::World());
    const auto gravity = ecm.Component<gz::sim::components::Gravity>(world);
    if (!gravity || !mount_.IsFinite()) {
      gzerr << "PoseImuSystem requires world gravity and a finite sensor_pose.\n";
      return;
    }
    gravity_ = gravity->Data();
    publisher_ = node_.Advertise<gz::msgs::IMU>(
        sdf->Get<std::string>("topic", "/livox/imu").first);
    configured_ = static_cast<bool>(publisher_);
    gzmsg << "PoseImuSystem: " << link << " -> " << frame_ << ", "
          << frequency << " Hz; timestamped at central sample.\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (!configured_ || info.paused) {
      return;
    }
    const auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime);
    if (count_ && (time < samples_[count_ - 1].time ||
                  time - samples_[count_ - 1].time > 4 * period_)) {
      count_ = 0;
    }
    if (count_ && time - samples_[count_ - 1].time < period_) {
      return;
    }
    samples_[count_++] = {time, gz::sim::worldPose(link_, ecm) * mount_};
    if (count_ < samples_.size()) {
      return;
    }
    const auto message = MakePoseImu(samples_[0], samples_[1], samples_[2], gravity_, frame_);
    if (message) {
      publisher_.Publish(*message);
    }
    samples_[0] = samples_[1];
    samples_[1] = samples_[2];
    count_ = 2;
  }

private:
  gz::sim::Entity link_{gz::sim::kNullEntity};
  gz::math::Pose3d mount_;
  gz::math::Vector3d gravity_;
  std::string frame_;
  std::chrono::nanoseconds period_{};
  std::array<ImuPoseSample, 3> samples_;
  size_t count_{0};
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  bool configured_{false};
};

} // namespace rm_27_stimulation

GZ_ADD_PLUGIN(rm_27_stimulation::PoseImuSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(rm_27_stimulation::PoseImuSystem,
                    "rm_27_stimulation::PoseImuSystem")
