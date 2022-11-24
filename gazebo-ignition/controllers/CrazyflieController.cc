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
 * 
 * @file CrazyflieController.cc
 * A simple control plugin for Ignition Gazebo for controlling the motors
 * 
 */

#include "CrazyflieController.hh"

#include <ignition/plugin/Register.hh>

#include <ignition/msgs/actuators.pb.h>
#include "ignition/gazebo/components/Actuators.hh"
#include "ignition/gazebo/components/Actuators.hh"

#include <ignition/gazebo/Util.hh>


extern "C" {
#include "../../controllers/pid_controller.h"
}

using namespace crazyflie_controller;

IGNITION_ADD_PLUGIN(
    crazyflie_controller::CrazyflieController,
    ignition::gazebo::System,
    crazyflie_controller::CrazyflieController::ISystemConfigure,
    crazyflie_controller::CrazyflieController::ISystemPreUpdate)

using namespace crazyflie_controller;

CrazyflieController::CrazyflieController()
{
}

CrazyflieController::~CrazyflieController()
{
}

void CrazyflieController::Configure(const ignition::gazebo::Entity &_entity,
                                    const std::shared_ptr<const sdf::Element> &_sdf,
                                    ignition::gazebo::EntityComponentManager &_ecm,
                                    ignition::gazebo::EventManager & /*_eventMgr*/)
{
    this->model = ignition::gazebo::Model(_entity);
    this->motorCommands.mutable_velocity()->Resize(
        4, 0);

    _ecm.CreateComponent(this->model.Entity(),
                         ignition::gazebo::components::Actuators(this->motorCommands));
    init_pid_attitude_fixed_height_controller();

    std::string topic{"/cmd_vel"};

    this->node.Subscribe(topic, &CrazyflieController::velCmd_cb, this);
}

void CrazyflieController::velCmd_cb(const ignition::msgs::Twist &_msg)
{
      this->velCmd = _msg;
}

double past_time;

void CrazyflieController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                    ignition::gazebo::EntityComponentManager &_ecm)
{

    auto current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      _info.simTime).count();

    double current_time = (double)current_time_ms/1000.0;
    double dt = 0.01;

    ignition::math::Pose3d poseCF = ignition::gazebo::worldPose(this->model.Entity(), _ecm);

    double rollActual = poseCF.Roll();
    double pitchActual = poseCF.Pitch();
    double yawActual = poseCF.Yaw();
    double altitudeActual = poseCF.Z();

    double xGlobal = poseCF.X();
    double yGlobal = poseCF.Y();

    double vxGlobal = (xGlobal - this->pastXGlobal) / dt;
    double vyGlobal = (yGlobal - this->pastYGlobal) / dt;

    double cosYaw = cos(yawActual);
    double sinYaw = sin(yawActual);

    ActualState_t actualState;
    actualState.roll = rollActual;
    actualState.pitch = pitchActual;
    actualState.yaw_rate = (yawActual - this->pastYawActual) / dt;
    actualState.vx = vxGlobal * cosYaw + vyGlobal * sinYaw;
    actualState.vy = -vxGlobal * sinYaw + vyGlobal * cosYaw;
    actualState.altitude = altitudeActual;

    DesiredState_t desiredState;
    desiredState.roll = 0;
    desiredState.pitch = 0;
    desiredState.vx = 0;
    desiredState.vy = 0;
    desiredState.yaw_rate = 0;
    desiredState.altitude = 1;

    desiredState.vx = this->velCmd.linear().x();
    desiredState.vy = this->velCmd.linear().y();
    desiredState.yaw_rate = this->velCmd.angular().z();

//    ignmsg << desiredState.pitch << " " << desiredState.roll << " " << desiredState.yaw_rate << std::endl;

    double factor = 1;

    GainsPID_t gainsPID;
    gainsPID.kp_att_y = 1 * factor;
    gainsPID.kd_att_y = 0.5 * factor;
    gainsPID.kp_att_rp = 0.5 * factor;
    gainsPID.kd_att_rp = 0.1 * factor;
    gainsPID.kp_vel_xy = 2 * factor;
    gainsPID.kd_vel_xy = 0.5 * factor;
    gainsPID.kp_z = 10 * factor;
    gainsPID.ki_z = 50 * factor;
    gainsPID.kd_z = 5 * factor;
    MotorPower_t motorPower;

    pid_velocity_fixed_height_controller(actualState, &desiredState, gainsPID, dt, &motorPower);

//    ignmsg << rollActual <<" "<<rollDesired << std::endl;

    this->motorCommands.set_velocity(0, motorPower.m1);
    this->motorCommands.set_velocity(1, motorPower.m2);
    this->motorCommands.set_velocity(2, motorPower.m3);
    this->motorCommands.set_velocity(3, motorPower.m4);

    auto motorCommandsComponent =
        _ecm.Component<ignition::gazebo::components::Actuators>(this->model.Entity());

    motorCommandsComponent->SetData(this->motorCommands,
                                    [](const ignition::msgs::Actuators &, const ignition::msgs::Actuators &)
                                    { return false; });

    _ecm.SetChanged(this->model.Entity(), ignition::gazebo::components::Actuators::typeId, ignition::gazebo::ComponentState::PeriodicChange);

    ignmsg << "set motors" << std::endl;

    this->pastTime = current_time;
    this->pastXGlobal = xGlobal;
    this->pastYGlobal = yGlobal;
    this->pastYawActual = yawActual;
}
