/** 
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *  
 * MIT License
 * 
 * Copyright (c) 2022 Bitcraze
 * 
 * @file CrazyflieController.hh
 * Header file for CrazyflieController.cc
 * 
 */

#ifndef SYSTEM_PLUGIN_CRAZYFLIECONTROLLER_HH_
#define SYSTEM_PLUGIN_CRAZYFLIECONTROLLER_HH_

#include <ignition/gazebo/System.hh>
#include <memory>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>
#include "ignition/gazebo/Model.hh"

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

  };
}
//! [header]

#endif
