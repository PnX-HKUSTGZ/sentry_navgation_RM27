// Initialization-only actuation. The baseline chassis controller/contact model
// is unchanged; this separate system permanently retires before Nav2 starts.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace rm_27_stimulation {
class StartupYawAssistSystem final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    link_ = gz::sim::Link(gz::sim::Model(entity).LinkByName(ecm, "base_footprint"));
    if (!link_.Valid(ecm)) return;
    link_.EnableVelocityChecks(ecm, true);
    node_.Subscribe("/cmd_vel", &StartupYawAssistSystem::command, this);
    node_.Advertise("/rm27/startup_assist/retire", &StartupYawAssistSystem::retire, this);
    configured_ = true;
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (!configured_ || info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    const auto velocity = link_.WorldAngularVelocity(ecm);
    if (!velocity || !std::isfinite(velocity->Z()) || dt <= 0.0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    yaw_rate_ = velocity->Z();
    if (retired_) return;
    if (sequence_ != consumed_) { age_ = 0.0; consumed_ = sequence_; }
    else age_ += dt;
    const double target = age_ <= 0.5 ? requested_ : 0.0;
    integral_torque_ = target == 0.0 ? 0.0 : std::clamp(
      integral_torque_ + 600.0 * (target - yaw_rate_) * dt, -120.0, 120.0);
    // Do not introduce translation or alter contacts. Static torsional drag in
    // the baseline CAD model requires more torque than its normal yaw servo.
    const double torque = std::clamp(
      60.0 * (target - yaw_rate_) + integral_torque_, -120.0, 120.0);
    link_.AddWorldWrench(ecm, gz::math::Vector3d::Zero, gz::math::Vector3d(0, 0, torque));
  }
private:
  void command(const gz::msgs::Twist &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_) return;
    const bool finite = std::isfinite(msg.linear().x()) && std::isfinite(msg.linear().y()) &&
      std::isfinite(msg.angular().z());
    requested_ = finite && msg.linear().x() == 0.0 && msg.linear().y() == 0.0 ?
      std::clamp(msg.angular().z(), -3.5, 3.5) : 0.0;
    ++sequence_;
  }
  bool retire(const gz::msgs::Empty &, gz::msgs::Boolean &response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!retired_ && (std::abs(yaw_rate_) >= 0.03 || (age_ <= 0.5 && requested_ != 0.0))) {
        response.set_data(false);
        return true;
      }
      retired_ = true;
    }
    node_.Unsubscribe("/cmd_vel");
    response.set_data(true);
    return true;
  }
  gz::transport::Node node_;
  gz::sim::Link link_;
  std::mutex mutex_;
  bool configured_{false}, retired_{false};
  double age_{1.0}, requested_{0.0}, yaw_rate_{0.0}, integral_torque_{0.0};
  uint64_t sequence_{0}, consumed_{0};
};
}
GZ_ADD_PLUGIN(rm_27_stimulation::StartupYawAssistSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(rm_27_stimulation::StartupYawAssistSystem,
                    "rm_27_stimulation::StartupYawAssistSystem")
