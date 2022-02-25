#include "CrazyflieController.hh"

#include <ignition/plugin/Register.hh>


IGNITION_ADD_PLUGIN(
    crazyflie_controller::CrazyflieController,
    ignition::gazebo::System,
    crazyflie_controller::CrazyflieController::ISystemPreUpdate,
    crazyflie_controller::CrazyflieController::ISystemUpdate)
//! [registerSampleSystem2]

using namespace crazyflie_controller;

CrazyflieController::CrazyflieController()
{
}

CrazyflieController::~CrazyflieController()
{
}

void CrazyflieController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem2::PreUpdate" << std::endl;
}

void CrazyflieController::Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "SampleSystem2::Update" << std::endl;
}

