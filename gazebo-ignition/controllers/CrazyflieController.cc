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
    

    ignition::math::Pose3d poseCF = ignition::gazebo::worldPose(this->model.Entity(), _ecm);
    double rollActual = poseCF.Roll();
    double pitchActual = poseCF.Pitch();
    double yawActual = poseCF.Yaw();
    double altitudeActual = poseCF.Z();

    double rollDesired = 0;
    double pitchDesired = 0;
    double yawDesired = 0;
    double altitudeDesired = 1;


        pitchDesired = this->velCmd.linear().x();
        rollDesired = this->velCmd.linear().y();

            ignmsg << pitchDesired <<" "<<rollDesired << std::endl;



    double factor = 1;

    double kp_att_y = 1 * factor;
    double kd_att_y = 0.5 * factor;
    double kp_att_rp = 0.5 * factor;
    double kd_att_rp = 0.1 * factor;
    double kp_z = 10 * factor;
    double ki_z = 50 * factor;
    double kd_y = 5 * factor;
    MotorPower_t motorPower;

    double dt = 0.01;
    
    pid_attitude_fixed_height_controller(rollActual, pitchActual, yawActual, altitudeActual, 
    rollDesired, pitchDesired, yawDesired, altitudeDesired,
     kp_att_rp,  kd_att_rp,  kp_att_y,  kd_att_y,  kp_z,  kd_y,  ki_z, dt, &motorPower);

    ignmsg << rollActual <<" "<<rollDesired << std::endl;



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

    past_time = current_time;
}
