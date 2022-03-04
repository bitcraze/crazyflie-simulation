#ifndef SYSTEM_PLUGIN_CRAZYFLIECONTROLLER_HH_
#define SYSTEM_PLUGIN_CRAZYFLIECONTROLLER_HH_

#include <ignition/gazebo/System.hh>
#include <memory>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

namespace crazyflie_controller
{
  class CrazyflieController:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    public: CrazyflieController();

    public: ~CrazyflieController() override;

    public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &) override;

    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    private: ignition::msgs::Actuators motorCommands;
    private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

    private: double pastTime;

    private: void velCmd_cb(const ignition::msgs::Twist &_msg);
    private: ignition::msgs::Twist velCmd;
    private: ignition::transport::Node node;

  };
}
//! [header]

#endif
