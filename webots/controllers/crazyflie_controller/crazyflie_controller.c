#include <math.h>
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>

#include "../../../controllers/pid_controller.h"

int main(int argc, char **argv) {
  wb_robot_init();

  int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize motors
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  // Initialize Sensors
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }

  // Intialize variables
  double rollActual, pitchActual, yawActual, altitudeActual;
  double rollControl, pitchControl, yawControl, altitudeControl;
  double rollDesired, pitchDesired, yawDesired, altitudeDesired;
  double past_time = wb_robot_get_time();
  double altitudeIntergralError=0;
  double pastAltitudeError=0;
  double pastYawError=0;
  double pastPitchError = 0;
  double pastRollError = 0;

  // Initialize PID gains.
  double kp = 1;
  double kd = 0.5;
  double kp_rp =0.5;
  double kd_rp = 0.1;
  double kpHeight = 10;
  double kiHeight = 50;
  double kdHeight = 5;

  MotorPower_t motorPower;

  printf("Take off!\n");

  while (wb_robot_step(timestep) != -1) {

    const double dt = wb_robot_get_time() - past_time;

    // Get measurements
    rollActual = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    pitchActual = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    yawActual = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    altitudeActual = wb_gps_get_values(gps)[2];

    // Initialize values
    rollDesired = 0;
    pitchDesired = 0;
    yawDesired = 0;
    altitudeDesired = 1;

    // Control altitude
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          pitchDesired = + 0.05;
          break;
        case WB_KEYBOARD_DOWN:
          pitchDesired = - 0.05;
          break;
        case WB_KEYBOARD_RIGHT:
          rollDesired = + 0.05;
          break;
        case WB_KEYBOARD_LEFT:
          rollDesired = - 0.05;
          break;
          }
      key = wb_keyboard_get_key();
    }

    pid_attitude_fixed_height_controller(rollActual, pitchActual, yawActual, altitudeActual, 
    rollDesired, pitchDesired, yawDesired, altitudeDesired,
     kp_rp,  kd_rp,  kp,  kd,  kpHeight,  kdHeight,  kiHeight, dt, &motorPower);

    /*
    // Calculate errors
    double altitudeError = altitudeDesired - altitudeActual;
    altitudeIntergralError += altitudeError*dt;
    double altitudeDerivativeError = (altitudeError - pastAltitudeError)/dt;
    double yawError = yawDesired - yawActual;
    double yawDerivativeError = (yawError - pastYawError)/dt;
    double pitchError = pitchDesired - pitchActual;
    double pitchDerivativeError = (pitchError - pastPitchError)/dt;
    double rollError = rollDesired - rollActual;
    double rollDerivativeError = (rollError - pastRollError)/dt;

    //PID control
    rollControl =kp_rp * constrain(rollError,-1, 1) + kd_rp*rollDerivativeError;
    pitchControl =-kp_rp * constrain(pitchError,-1, 1) - kd_rp*pitchDerivativeError;
    yawControl = kp * constrain(yawError, -1, 1)+ kd*yawDerivativeError;
    altitudeControl = kpHeight* constrain(altitudeError, -1, 1) + kiHeight+ kdHeight*altitudeDerivativeError;
    
    // Motor mixing
    motorPower.m1 =  altitudeControl -rollControl + pitchControl + yawControl;
    motorPower.m2 =  altitudeControl - rollControl - pitchControl - yawControl;
    motorPower.m3 =  altitudeControl + rollControl - pitchControl + yawControl;
    motorPower.m4 =  altitudeControl + rollControl + pitchControl - yawControl;
    */

    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, - motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, - motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);
    
    // Save past time and errors for next time step
    past_time = wb_robot_get_time();
    /*pastAltitudeError = altitudeError;
    pastYawError = yawError;
    pastPitchError= pitchError;
    pastRollError= rollError;*/

  };

  wb_robot_cleanup();

  return 0;
}
