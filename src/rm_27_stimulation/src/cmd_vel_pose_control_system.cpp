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
#include <gz/sim/components/Pose.hh>
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

    if (!hasCommand)
    {
      return;
    }

    if (commandSeq != this->lastCommandSeq_)
    {
      this->lastCommandSeq_ = commandSeq;
      this->commandAge_ = 0.0;
    }
    else
    {
      this->commandAge_ += dt;
    }

    if (this->commandAge_ > this->commandTimeout_)
    {
      return;
    }

    auto vx = this->Clamp(cmd.linear().x(), this->maxLinearVelocity_);
    auto vy = this->Clamp(cmd.linear().y(), this->maxLinearVelocity_);
    auto wz = this->Clamp(cmd.angular().z(), this->maxAngularVelocity_);

    auto pose = gz::sim::worldPose(this->model_.Entity(), _ecm);
    auto rpy = pose.Rot().Euler();
    const auto yaw = rpy.Z();

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

    pose.Pos().X() += dx * dt;
    pose.Pos().Y() += dy * dt;
    rpy.Z() = this->NormalizeAngle(yaw + wz * dt);
    pose.Rot().SetFromEuler(rpy);

    // Bullet in Harmonic does not consume link velocity or world pose commands
    // for this URDF model, so the simulation base is driven kinematically.
    _ecm.SetComponentData<gz::sim::components::Pose>(this->model_.Entity(), pose);
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

  static double NormalizeAngle(double _angle)
  {
    return std::atan2(std::sin(_angle), std::cos(_angle));
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
