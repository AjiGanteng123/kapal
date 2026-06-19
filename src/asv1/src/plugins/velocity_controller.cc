#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <string>
#include <vector>

namespace asv1
{

class VelocityController :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> childEntities;
  gz::transport::Node node;
  double targetVx{0.0};
  double targetWz{0.0};

  template<typename CompT>
  static void SetPose(gz::sim::EntityComponentManager &_ecm,
                      gz::sim::Entity _entity,
                      const typename CompT::Type &_data)
  {
    auto comp = _ecm.Component<CompT>(_entity);
    if (comp) _ecm.SetComponentData<CompT>(_entity, _data);
    else _ecm.CreateComponent(_entity, CompT(_data));
  }

  void OnCmdVel(const gz::msgs::Twist &_msg)
  {
    targetVx = _msg.linear().x();
    targetWz = _msg.angular().z();
  }

  void UpdateChildVisuals(gz::sim::EntityComponentManager &_ecm,
                          const gz::math::Pose3d &_worldPose)
  {
    for (auto child : childEntities)
    {
      auto childPoseComp = _ecm.Component<gz::sim::components::Pose>(child);
      if (!childPoseComp) continue;
      auto childWorldPose = _worldPose * childPoseComp->Data();
      SetPose<gz::sim::components::WorldPose>(_ecm, child, childWorldPose);
    }
  }

  public: void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &
  ) override
  {
    modelEntity = _entity;
    gz::sim::Model model(_entity);
    auto links = model.Links(_ecm);
    if (!links.empty())
      linkEntity = links[0];

    // Cache all child entities of the link that have a Pose component
    _ecm.Each<gz::sim::components::ParentEntity, gz::sim::components::Pose>(
        [&](const gz::sim::Entity &child,
            const gz::sim::components::ParentEntity *parent,
            const gz::sim::components::Pose *) -> bool
        {
          if (parent->Data() == linkEntity)
            childEntities.push_back(child);
          return true;
        });

    auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
    std::string modelName = nameComp ? nameComp->Data() : "model";
    std::string topic = "/model/" + modelName + "/cmd_vel";
    node.Subscribe(topic, &VelocityController::OnCmdVel, this);
  }

  public: void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
  ) override
  {
    if (_info.paused) return;

    double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0 || dt > 0.1) dt = 0.001;

    auto poseComp = _ecm.Component<gz::sim::components::Pose>(modelEntity);
    if (!poseComp) return;

    gz::math::Pose3d pose = poseComp->Data();
    double yaw = pose.Rot().Yaw();
    pose.Pos().X(pose.Pos().X() + targetVx * dt * std::cos(yaw));
    pose.Pos().Y(pose.Pos().Y() + targetVx * dt * std::sin(yaw));
    pose.Rot() = gz::math::Quaterniond(0, 0, yaw + targetWz * dt);

    SetPose<gz::sim::components::Pose>(_ecm, modelEntity, pose);
    SetPose<gz::sim::components::WorldPose>(_ecm, modelEntity, pose);
    if (linkEntity != gz::sim::kNullEntity)
    {
      SetPose<gz::sim::components::WorldPose>(_ecm, linkEntity, pose);
      UpdateChildVisuals(_ecm, pose);
    }
  }
};

}

#include <gz/plugin/Register.hh>
GZ_ADD_PLUGIN(
    asv1::VelocityController,
    gz::sim::System,
    asv1::VelocityController::ISystemConfigure,
    asv1::VelocityController::ISystemPreUpdate)
