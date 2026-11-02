// Generic dynamic obstacle plugin for Gazebo classic.
// Moves a model along a configurable relative segment using PoseAnimation.

#include <ignition/math.hh>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class DynamicObstacle: public ModelPlugin
{
public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override
  {
    if (!parent) {
      return;
    }

    const double delta_x = sdf->Get<double>("delta_x", 0.0).first;
    const double delta_y = sdf->Get<double>("delta_y", 0.0).first;
    const double period_sec = std::max(0.4, sdf->Get<double>("period_sec", 6.0).first);
    const double hold_sec = std::max(0.0, sdf->Get<double>("hold_sec", 0.0).first);

    const double half_period = std::max(0.2, period_sec / 2.0);
    const double total_period = std::max(0.4, period_sec + 2.0 * hold_sec);

    auto anim = gazebo::common::PoseAnimationPtr(
      new gazebo::common::PoseAnimation("dynamic_obstacle_motion", total_period, true));

    gazebo::common::PoseKeyFrame * key = nullptr;

    key = anim->CreateKeyFrame(0.0);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0.0, 0.0, 0.0));

    key = anim->CreateKeyFrame(half_period);
    key->Translation(ignition::math::Vector3d(delta_x, delta_y, 0.0));
    key->Rotation(ignition::math::Quaterniond(0.0, 0.0, 0.0));

    key = anim->CreateKeyFrame(half_period + hold_sec);
    key->Translation(ignition::math::Vector3d(delta_x, delta_y, 0.0));
    key->Rotation(ignition::math::Quaterniond(0.0, 0.0, 0.0));

    key = anim->CreateKeyFrame(period_sec + hold_sec);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0.0, 0.0, 0.0));

    key = anim->CreateKeyFrame(total_period);
    key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
    key->Rotation(ignition::math::Quaterniond(0.0, 0.0, 0.0));

    parent->SetAnimation(anim);
  }
};

GZ_REGISTER_MODEL_PLUGIN(DynamicObstacle)
}  // namespace gazebo
