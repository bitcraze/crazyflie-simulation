#ifndef SYSTEM_PLUGIN_CRAZYFLIECONTROLLER_HH_
#define SYSTEM_PLUGIN_CRAZYFLIECONTROLLER_HH_

//! [header]
#include <ignition/gazebo/System.hh>

namespace crazyflie_controller
{
  class CrazyflieController:
    // This class is a system.
    public ignition::gazebo::System,
    // This class also implements the ISystemPreUpdate, ISystemUpdate,
    // and ISystemPostUpdate interfaces.
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemUpdate
  {
    public: CrazyflieController();

    public: ~CrazyflieController() override;

    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    public: void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

  };
}
//! [header]

#endif
