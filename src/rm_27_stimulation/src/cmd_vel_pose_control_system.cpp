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

namespace rm_27_stimulation
{
class CmdVelPoseControlSystem final
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm))
    {
      gzerr << "CmdVelPoseControlSystem must be attached to a model entity.\n";
      return;
    }

    if (_sdf->HasElement("topic"))
    {
      this->topic_ = _sdf->Get<std::string>("topic");
    }
    if (_sdf->HasElement("body_frame"))
    {
      this->bodyFrame_ = _sdf->Get<bool>("body_frame");
    }
    if (_sdf->HasElement("command_timeout"))
    {
      this->commandTimeout_ = std::max(0.0, _sdf->Get<double>("command_timeout"));
    }
    if (_sdf->HasElement("max_linear_velocity"))
    {
      this->maxLinearVelocity_ = std::max(0.0, _sdf->Get<double>("max_linear_velocity"));
    }
    if (_sdf->HasElement("max_angular_velocity"))
    {
      this->maxAngularVelocity_ = std::max(0.0, _sdf->Get<double>("max_angular_velocity"));
    }
    if (_sdf->HasElement("linear_velocity_gain"))
    {
      this->linearVelocityGain_ =
          std::max(0.0, _sdf->Get<double>("linear_velocity_gain"));
    }
    if (_sdf->HasElement("angular_velocity_gain"))
    {
      this->angularVelocityGain_ =
          std::max(0.0, _sdf->Get<double>("angular_velocity_gain"));
    }
    if (_sdf->HasElement("max_planar_force"))
    {
      this->maxPlanarForce_ =
          std::max(0.0, _sdf->Get<double>("max_planar_force"));
    }
    if (_sdf->HasElement("max_yaw_torque"))
    {
      this->maxYawTorque_ =
          std::max(0.0, _sdf->Get<double>("max_yaw_torque"));
    }

    const auto modelName = this->model_.Name(_ecm);
    this->driveLink_ = gz::sim::Link(this->model_.CanonicalLink(_ecm));
    if (!this->driveLink_.Valid(_ecm))
    {
      gzerr << "CmdVelPoseControlSystem failed to find canonical link for model ["
            << modelName << "].\n";
      return;
    }

    if (!this->node_.Subscribe(this->topic_, &CmdVelPoseControlSystem::OnCmdVel, this))
    {
      gzerr << "CmdVelPoseControlSystem failed to subscribe to ["
            << this->topic_ << "] for model [" << modelName << "].\n";
      return;
    }

    const auto linkName = this->driveLink_.Name(_ecm).value_or("<unnamed>");
    this->driveLink_.EnableVelocityChecks(_ecm, true);
    this->driveLink_.EnableAccelerationChecks(_ecm, true);

    this->configured_ = true;
    gzmsg << "CmdVelPoseControlSystem controlling model [" << modelName
          << "] link [" << linkName
          << "] from topic [" << this->topic_ << "].\n";
  }

  void Update(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured_ || _info.paused)
    {
      return;
    }

    const auto dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
    {
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

    if (hasCommand && commandSeq != this->lastCommandSeq_)
    {
      this->lastCommandSeq_ = commandSeq;
      this->commandAge_ = 0.0;
    }
    else
    {
      this->commandAge_ += dt;
    }

    const auto commandActive = hasCommand &&
        this->commandAge_ <= this->commandTimeout_;
    auto vx = commandActive
        ? this->Clamp(cmd.linear().x(), this->maxLinearVelocity_) : 0.0;
    auto vy = commandActive
        ? this->Clamp(cmd.linear().y(), this->maxLinearVelocity_) : 0.0;
    auto wz = commandActive
        ? this->Clamp(cmd.angular().z(), this->maxAngularVelocity_) : 0.0;

    auto pose = gz::sim::worldPose(this->model_.Entity(), _ecm);
    const auto yaw = pose.Rot().Euler().Z();

    double dx = vx;
    double dy = vy;
    if (this->bodyFrame_)
    {
      const auto cosYaw = std::cos(yaw);
      const auto sinYaw = std::sin(yaw);
      dx = cosYaw * vx - sinYaw * vy;
      dy = sinYaw * vx + cosYaw * vy;
    }
    else
    {
      dx = vx;
      dy = vy;
    }

    const auto currentLinear =
        this->driveLink_.WorldLinearVelocity(_ecm).value_or(
            gz::math::Vector3d::Zero);
    const auto currentAngular =
        this->driveLink_.WorldAngularVelocity(_ecm).value_or(
            gz::math::Vector3d::Zero);

    // A force servo keeps IMU acceleration, contact response and model motion
    // in the same physics solution. Only planar force and yaw torque are
    // commanded, so gravity and terrain still determine z, roll and pitch.
    auto planarForce = gz::math::Vector3d(
        (dx - currentLinear.X()) * this->linearVelocityGain_,
        (dy - currentLinear.Y()) * this->linearVelocityGain_,
        0.0);
    const auto forceLength = planarForce.Length();
    if (this->maxPlanarForce_ > 0.0 && forceLength > this->maxPlanarForce_)
    {
      planarForce *= this->maxPlanarForce_ / forceLength;
    }

    const auto yawTorque = this->Clamp(
        (wz - currentAngular.Z()) * this->angularVelocityGain_,
        this->maxYawTorque_);
    this->driveLink_.AddWorldForce(_ecm, planarForce);
    this->driveLink_.AddWorldWrench(
        _ecm, gz::math::Vector3d::Zero,
        gz::math::Vector3d(0.0, 0.0, yawTorque));
  }

private:
  void OnCmdVel(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->latestCmd_ = _msg;
    ++this->commandSeq_;
    this->hasCommand_ = true;
    if (!this->loggedFirstCommand_)
    {
      gzmsg << "CmdVelPoseControlSystem received first cmd_vel: vx="
            << _msg.linear().x() << " vy=" << _msg.linear().y()
            << " wz=" << _msg.angular().z() << ".\n";
      this->loggedFirstCommand_ = true;
    }
  }

  static double Clamp(double _value, double _limit)
  {
    if (_limit <= 0.0)
    {
      return _value;
    }
    return std::clamp(_value, -_limit, _limit);
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
  double angularVelocityGain_{4.0};
  double maxPlanarForce_{120.0};
  double maxYawTorque_{8.0};
  std::mutex mutex_;
  gz::msgs::Twist latestCmd_;
  uint64_t commandSeq_{0};
  uint64_t lastCommandSeq_{0};
  bool hasCommand_{false};
  bool loggedFirstCommand_{false};
};
}  // namespace rm_27_stimulation

GZ_ADD_PLUGIN(
    rm_27_stimulation::CmdVelPoseControlSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(
    rm_27_stimulation::CmdVelPoseControlSystem,
    "rm_27_stimulation::CmdVelPoseControlSystem")
