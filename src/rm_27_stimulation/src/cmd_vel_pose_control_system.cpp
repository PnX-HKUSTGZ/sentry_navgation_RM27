#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "rm_27_stimulation/planar_velocity_pi.hpp"
#include "rm_27_stimulation/stiction_assist.hpp"

namespace rm_27_stimulation {
class CmdVelPoseControlSystem final : public gz::sim::System,
                                      public gz::sim::ISystemConfigure,
                                      public gz::sim::ISystemUpdate {
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager & /*_eventMgr*/) override {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "CmdVelPoseControlSystem must be attached to a model entity.\n";
      return;
    }

    if (_sdf->HasElement("topic")) {
      this->topic_ = _sdf->Get<std::string>("topic");
    }
    if (_sdf->HasElement("body_frame")) {
      this->bodyFrame_ = _sdf->Get<bool>("body_frame");
    }
    this->ReadNonnegativeFinite(_sdf, "command_timeout", this->commandTimeout_);
    this->ReadNonnegativeFinite(_sdf, "max_linear_velocity",
                                this->maxLinearVelocity_);
    this->ReadNonnegativeFinite(_sdf, "max_angular_velocity",
                                this->maxAngularVelocity_);
    this->ReadNonnegativeFinite(_sdf, "linear_velocity_gain",
                                this->linearVelocityGain_);
    this->ReadNonnegativeFinite(_sdf, "linear_velocity_integral_gain",
                                this->linearVelocityIntegralGain_);
    this->ReadNonnegativeFinite(_sdf, "angular_velocity_gain",
                                this->angularVelocityGain_);
    this->ReadNonnegativeFinite(_sdf, "max_planar_force",
                                this->maxPlanarForce_);
    this->ReadNonnegativeFinite(_sdf, "max_planar_integral_force",
                                this->maxPlanarIntegralForce_);
    this->ReadNonnegativeFinite(_sdf, "stall_assist_command_threshold",
                                this->stictionAssistConfig_.command_threshold);
    this->ReadNonnegativeFinite(_sdf, "stall_assist_engage_velocity",
                                this->stictionAssistConfig_.engage_velocity);
    this->ReadNonnegativeFinite(_sdf, "stall_assist_release_velocity",
                                this->stictionAssistConfig_.release_velocity);
    this->ReadNonnegativeFinite(_sdf, "stall_assist_delay",
                                this->stictionAssistConfig_.delay);
    this->ReadNonnegativeFinite(_sdf, "stall_assist_ramp_rate",
                                this->stictionAssistConfig_.ramp_rate);
    this->ReadNonnegativeFinite(_sdf, "max_stall_assist_force",
                                this->stictionAssistConfig_.max_force);
    this->ReadNonnegativeFinite(_sdf, "max_yaw_torque", this->maxYawTorque_);

    const auto modelName = this->model_.Name(_ecm);
    this->driveLink_ = gz::sim::Link(this->model_.CanonicalLink(_ecm));
    if (!this->driveLink_.Valid(_ecm)) {
      gzerr
          << "CmdVelPoseControlSystem failed to find canonical link for model ["
          << modelName << "].\n";
      return;
    }

    if (!this->node_.Subscribe(this->topic_, &CmdVelPoseControlSystem::OnCmdVel,
                               this)) {
      gzerr << "CmdVelPoseControlSystem failed to subscribe to ["
            << this->topic_ << "] for model [" << modelName << "].\n";
      return;
    }

    const auto linkName = this->driveLink_.Name(_ecm).value_or("<unnamed>");
    this->driveLink_.EnableVelocityChecks(_ecm, true);
    this->driveLink_.EnableAccelerationChecks(_ecm, true);

    this->configured_ = true;
    gzmsg << "CmdVelPoseControlSystem controlling model [" << modelName
          << "] link [" << linkName << "] from topic [" << this->topic_
          << "] with planar Kp=" << this->linearVelocityGain_
          << ", Ki=" << this->linearVelocityIntegralGain_
          << ", integral-force limit=" << this->maxPlanarIntegralForce_
          << ", stall-assist limit=" << this->stictionAssistConfig_.max_force
          << " N.\n";
  }

  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override {
    if (!this->configured_ || _info.paused) {
      return;
    }

    const auto dt = std::chrono::duration<double>(_info.dt).count();
    if (!std::isfinite(dt) || dt <= 0.0) {
      this->planarVelocityPi_.Reset();
      return;
    }

    gz::msgs::Twist cmd;
    uint64_t commandSeq = 0;
    bool hasCommand = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      cmd = this->latestCmd_;
      commandSeq = this->commandSeq_;
      hasCommand = this->hasCommand_;
    }

    if (hasCommand && commandSeq != this->lastCommandSeq_) {
      this->lastCommandSeq_ = commandSeq;
      this->commandAge_ = 0.0;
    } else {
      this->commandAge_ += dt;
    }

    const auto commandFinite = std::isfinite(cmd.linear().x()) &&
                               std::isfinite(cmd.linear().y()) &&
                               std::isfinite(cmd.angular().z());
    const auto commandActive = hasCommand && commandFinite &&
                               this->commandAge_ <= this->commandTimeout_;
    if (!commandActive) {
      this->planarVelocityPi_.Reset();
      this->stictionAssist_.Reset();
    }
    auto vx = commandActive ? this->ClampFinite(cmd.linear().x(),
                                                this->maxLinearVelocity_)
                            : 0.0;
    auto vy = commandActive ? this->ClampFinite(cmd.linear().y(),
                                                this->maxLinearVelocity_)
                            : 0.0;
    auto wz = commandActive ? this->ClampFinite(cmd.angular().z(),
                                                this->maxAngularVelocity_)
                            : 0.0;

    auto pose = gz::sim::worldPose(this->model_.Entity(), _ecm);
    const auto yaw = pose.Rot().Euler().Z();
    if (!std::isfinite(yaw)) {
      this->planarVelocityPi_.Reset();
      this->stictionAssist_.Reset();
      return;
    }

    double dx = vx;
    double dy = vy;
    if (this->bodyFrame_) {
      const auto cosYaw = std::cos(yaw);
      const auto sinYaw = std::sin(yaw);
      dx = cosYaw * vx - sinYaw * vy;
      dy = sinYaw * vx + cosYaw * vy;
    } else {
      dx = vx;
      dy = vy;
    }

    const auto currentLinear =
        this->driveLink_.WorldLinearVelocity(_ecm).value_or(
            gz::math::Vector3d::Zero);
    const auto currentAngular =
        this->driveLink_.WorldAngularVelocity(_ecm).value_or(
            gz::math::Vector3d::Zero);
    if (!std::isfinite(currentLinear.X()) ||
        !std::isfinite(currentLinear.Y()) ||
        !std::isfinite(currentAngular.Z())) {
      this->planarVelocityPi_.Reset();
      this->stictionAssist_.Reset();
      return;
    }

    // A force servo keeps IMU acceleration, contact response and model motion
    // in the same physics solution. Only planar force and yaw torque are
    // commanded, so gravity and terrain still determine z, roll and pitch.
    const auto planarForceCommand = this->planarVelocityPi_.Update(
        dx - currentLinear.X(), dy - currentLinear.Y(), dt,
        this->linearVelocityGain_,
        commandActive ? this->linearVelocityIntegralGain_ : 0.0,
        commandActive ? this->maxPlanarIntegralForce_ : 0.0,
        this->maxPlanarForce_);
    const auto stictionAssist =
        commandActive
            ? this->stictionAssist_.Update(dx, dy, currentLinear.X(),
                                           currentLinear.Y(), dt,
                                           this->stictionAssistConfig_)
            : PlanarForce{};
    const auto assistedForce =
        this->ClampMagnitude({planarForceCommand.x + stictionAssist.x,
                              planarForceCommand.y + stictionAssist.y},
                             this->maxPlanarForce_);
    const auto planarForce =
        gz::math::Vector3d(assistedForce.x, assistedForce.y, 0.0);

    const auto yawTorque =
        this->Clamp((wz - currentAngular.Z()) * this->angularVelocityGain_,
                    this->maxYawTorque_);
    this->driveLink_.AddWorldForce(_ecm, planarForce);
    this->driveLink_.AddWorldWrench(_ecm, gz::math::Vector3d::Zero,
                                    gz::math::Vector3d(0.0, 0.0, yawTorque));
  }

private:
  void OnCmdVel(const gz::msgs::Twist &_msg) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->latestCmd_ = _msg;
    ++this->commandSeq_;
    this->hasCommand_ = true;
    if (!this->loggedFirstCommand_) {
      gzmsg << "CmdVelPoseControlSystem received first cmd_vel: vx="
            << _msg.linear().x() << " vy=" << _msg.linear().y()
            << " wz=" << _msg.angular().z() << ".\n";
      this->loggedFirstCommand_ = true;
    }
  }

  static double Clamp(double _value, double _limit) {
    if (_limit <= 0.0) {
      return _value;
    }
    return std::clamp(_value, -_limit, _limit);
  }

  static double ClampFinite(double _value, double _limit) {
    return std::isfinite(_value) ? Clamp(_value, _limit) : 0.0;
  }

  static PlanarForce ClampMagnitude(PlanarForce _value, double _limit) {
    if (_limit <= 0.0) {
      return _value;
    }
    const auto magnitude = std::hypot(_value.x, _value.y);
    if (magnitude > _limit) {
      const auto scale = _limit / magnitude;
      _value.x *= scale;
      _value.y *= scale;
    }
    return _value;
  }

  void ReadNonnegativeFinite(const std::shared_ptr<const sdf::Element> &_sdf,
                             const std::string &_name, double &_value) {
    if (!_sdf->HasElement(_name)) {
      return;
    }

    const auto configured = _sdf->Get<double>(_name);
    if (!std::isfinite(configured) || configured < 0.0) {
      gzwarn << "CmdVelPoseControlSystem ignoring invalid <" << _name
             << "> value [" << configured << "]; using " << _value << ".\n";
      return;
    }
    _value = configured;
  }

  gz::sim::Model model_;
  gz::sim::Link driveLink_;
  gz::transport::Node node_;
  std::string topic_{"/cmd_vel"};
  bool bodyFrame_{true};
  bool configured_{false};

  double commandTimeout_{0.5};
  double commandAge_{0.0};
  double maxLinearVelocity_{3.0};
  double maxAngularVelocity_{6.0};
  double linearVelocityGain_{80.0};
  double linearVelocityIntegralGain_{0.0};
  double angularVelocityGain_{4.0};
  double maxPlanarForce_{120.0};
  double maxPlanarIntegralForce_{0.0};
  double maxYawTorque_{8.0};
  PlanarVelocityPi planarVelocityPi_;
  StictionAssistConfig stictionAssistConfig_;
  StictionAssist stictionAssist_;
  std::mutex mutex_;
  gz::msgs::Twist latestCmd_;
  uint64_t commandSeq_{0};
  uint64_t lastCommandSeq_{0};
  bool hasCommand_{false};
  bool loggedFirstCommand_{false};
};
} // namespace rm_27_stimulation

GZ_ADD_PLUGIN(rm_27_stimulation::CmdVelPoseControlSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(rm_27_stimulation::CmdVelPoseControlSystem,
                    "rm_27_stimulation::CmdVelPoseControlSystem")
