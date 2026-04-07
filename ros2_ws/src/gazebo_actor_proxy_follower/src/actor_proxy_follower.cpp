#include <gazebo/common/Events.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/gazebo.hh>

#include <ignition/math/Pose3.hh>

#include <memory>
#include <string>

namespace gazebo_actor_proxy_follower
{
class ActorProxyFollower final : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    world_ = _world;

    if (_sdf && _sdf->HasElement("actor_name")) {
      actor_name_ = _sdf->Get<std::string>("actor_name");
    }
    if (_sdf && _sdf->HasElement("proxy_name")) {
      proxy_name_ = _sdf->Get<std::string>("proxy_name");
    }
    if (_sdf && _sdf->HasElement("z_offset")) {
      z_offset_ = _sdf->Get<double>("z_offset");
    }

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ActorProxyFollower::OnUpdate, this));
  }

private:
  void ResolveOnce()
  {
    if (!world_) {
      return;
    }

    if (!actor_ && !actor_name_.empty()) {
      auto m = world_->ModelByName(actor_name_);
      if (m) {
        actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(m);
      }
    }

    if (!proxy_ && !proxy_name_.empty()) {
      proxy_ = world_->ModelByName(proxy_name_);
    }
  }

  void OnUpdate()
  {
    if (!actor_ || !proxy_) {
      ResolveOnce();
      if (!actor_ || !proxy_) {
        return;
      }
    }

    ignition::math::Pose3d p = actor_->WorldPose();

    p.Pos().Z(p.Pos().Z() + z_offset_);

    // Actors may introduce roll/pitch during animation. For a "collision proxy"
    // used by 2D LiDAR, keep it upright: preserve yaw only.
    const auto yaw = p.Rot().Yaw();
    p.Rot() = ignition::math::Quaterniond(0.0, 0.0, yaw);

    proxy_->SetWorldPose(p);
    proxy_->SetLinearVel({0, 0, 0});
    proxy_->SetAngularVel({0, 0, 0});
  }

private:
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ActorPtr actor_;
  gazebo::physics::ModelPtr proxy_;
  gazebo::event::ConnectionPtr update_connection_;

  std::string actor_name_{"walking_actor_01"};
  std::string proxy_name_{"walking_person_proxy_01"};
  double z_offset_{0.0};
};

GZ_REGISTER_WORLD_PLUGIN(ActorProxyFollower)
}  // namespace gazebo_actor_proxy_follower
