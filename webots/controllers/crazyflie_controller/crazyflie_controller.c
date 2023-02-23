/*
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
 * @file crazyflie_controller.c
 * Controls the crazyflie motors in webots
 */

#include <math.h>
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>

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
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag range_front = wb_robot_get_device("range_front");
  wb_distance_sensor_enable(range_front, timestep);
  WbDeviceTag range_left = wb_robot_get_device("range_left");
  wb_distance_sensor_enable(range_left, timestep);
  WbDeviceTag range_back = wb_robot_get_device("range_back");
  wb_distance_sensor_enable(range_back, timestep);
  WbDeviceTag range_right = wb_robot_get_device("range_right");
  wb_distance_sensor_enable(range_right, timestep);


  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }

  // Initialize variables
  State_t actualState = {0};
  State_t desiredState = {0};
  double pastXGlobal =0;
  double pastYGlobal=0;
  double pastZGlobal=0;
  double past_time = wb_robot_get_time();

  // Initialize struct for motor power
  StatePID_t statePID;
  MotorPower_t motorPower;
  ControlCommands_t control;

  // Initialize PID gains.
  GainsPID_t gainsPID;
  gainsPID.rollpitchKP = 0.5;
  gainsPID.rollpitchKD = 0.1;
  gainsPID.rollpitchKI = 0.0;
  gainsPID.yawKP = 1;
  gainsPID.yawKD = 0.5;
  gainsPID.yawKI = 0.0;
  gainsPID.xyKP = 1;
  gainsPID.xyKD = 0.5;
  gainsPID.xyKI = 0.0;
  gainsPID.zKP = 30;
  gainsPID.zKD = 10;
  gainsPID.zKI = 10;

  while (wb_robot_step(timestep) != -1) {

    const double dt = wb_robot_get_time() - past_time;

    // Get measurements
    actualState.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actualState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actualState.yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    actualState.rollRate = wb_gyro_get_values(gyro)[0];
    actualState.pitchRate = wb_gyro_get_values(gyro)[1];
    actualState.yawRate = wb_gyro_get_values(gyro)[2];
    actualState.x = wb_gps_get_values(gps)[0];
    actualState.xVelocity = (actualState.x - pastXGlobal)/dt;
    actualState.y = wb_gps_get_values(gps)[1];
    actualState.yVelocity = (actualState.y - pastYGlobal)/dt;
    actualState.z = wb_gps_get_values(gps)[2];
    actualState.zVelocity = (actualState.z - pastZGlobal)/dt;

    // Initialize values for height Zs
    desiredState.z = 1.0;

    double forwardDesired = 0;
    double sidewaysDesired = 0;
    double yawDesired = 0;

    // Control altitude
    int key = wb_keyboard_get_key();

    // check keys
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          forwardDesired = + 0.2;
          break;
        case WB_KEYBOARD_DOWN:
          forwardDesired = - 0.2;
          break;
        case WB_KEYBOARD_RIGHT:
          sidewaysDesired = - 0.2;
          break;
        case WB_KEYBOARD_LEFT:
          sidewaysDesired = + 0.2;
          break;
        case 'Q':
          yawDesired = 0.5;
          break;
        case 'E':
          yawDesired = - 0.5;
          break;
        }
      key = wb_keyboard_get_key();
    }
    printf("forwardDesired: %f  sidewaysDesired: %f \n", forwardDesired, sidewaysDesired);

    // Example how to get sensor data
    // range_front_value = wb_distance_sensor_get_value(range_front));
    // const unsigned char *image = wb_camera_get_image(camera);

    // yaw based on yaw desired
    desiredState.yaw = actualState.yaw + yawDesired * dt;

    // calculate global desired velocity from body fixed desired velocity
    double vxGlobal = forwardDesired * cos(actualState.yaw) - sidewaysDesired * sin(actualState.yaw);
    double vyGlobal = forwardDesired * sin(actualState.yaw) + sidewaysDesired * cos(actualState.yaw);

    // Calculate global desired position based on global desired velocity
    desiredState.x = actualState.x + vxGlobal * dt;
    desiredState.y = actualState.y + vyGlobal * dt;

    // full PID controller
    pid_pos_att_controller(&statePID, gainsPID, actualState, &desiredState, dt, &control);

    //motor mixing
    motor_mixing(control, &motorPower);

    printf("cmd roll: %f, cmd pitch: %f, cmd yaw: %f, cmd z: %f   \n", control.roll, control.pitch, control.yaw, control.z);
    // print out the motor powers
    printf("m1: %f, m2: %f, m3: %f, m4: %f   \n", motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4);
    // print desired z and actual z
    printf("desired z: %f, actual z: %f   \n", desiredState.z, actualState.z);

    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, - motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, - motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);

    // Save past time for next time step
    past_time = wb_robot_get_time();
    pastXGlobal = actualState.x;
    pastYGlobal = actualState.y;
    pastZGlobal = actualState.z;

  };

  wb_robot_cleanup();

  return 0;
}
