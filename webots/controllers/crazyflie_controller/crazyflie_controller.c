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
  double pastRoll =0;
  double pastPitch=0;
  double pastYaw=0;
  double past_time = wb_robot_get_time();

  // Initialize struct for motor power
  StatePID_t statePID;
  MotorPower_t motorPower;
  ControlCommands_t control;

 double gain_factor = 0.5;
  // Initialize PID gains.
  GainsPID_t gainsPID;
  gainsPID.rollpitchKP = 0.05;
  gainsPID.rollpitchKD = 0.00;
  gainsPID.rollpitchKI = 0.0;
  gainsPID.yawKP = 0.5;
  gainsPID.yawKD = 0.1;
  gainsPID.yawKI = 0.0;
  gainsPID.yawRateKP = gainsPID.yawKP * gain_factor;
  gainsPID.yawRateKD = gainsPID.yawKD * gain_factor;
  gainsPID.yawRateKI = gainsPID.yawKI * gain_factor;
  gainsPID.xyKP = 0.005;
  gainsPID.xyKD = 0.1;
  gainsPID.xyKI = 0.0;
  gainsPID.xyVelocityKP =   0.8;
  gainsPID.xyVelocityKD =  0.0;
  gainsPID.xyVelocityKI =   0.0;
  gainsPID.zKP = 40;
  gainsPID.zKD = 5;
  gainsPID.zKI = 12;
  gainsPID.zVelocityKP = 40*gain_factor/2;
  gainsPID.zVelocityKD = 5*gain_factor/2;
  gainsPID.zVelocityKI = 12*gain_factor/2;

  while (wb_robot_step(timestep) != -1) {

    const double dt = wb_robot_get_time() - past_time;

    // Get measurements
    actualState.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actualState.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actualState.yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    actualState.rollRate = wb_gyro_get_values(gyro)[0];
    actualState.pitchRate = wb_gyro_get_values(gyro)[1];
    actualState.yawRate = wb_gyro_get_values(gyro)[2];
    //actualState.rollRate = (actualState.roll - pastRoll)/dt;
    //actualState.pitchRate = (actualState.pitch - pastPitch)/dt;
    //actualState.yawRate = (actualState.yaw - pastYaw)/dt;
    double xGlobal = wb_gps_get_values(gps)[0];
    double xGlobalVel = (xGlobal - pastXGlobal)/dt;
    double yGlobal = wb_gps_get_values(gps)[1];
    double yGlobalVel = (yGlobal - pastYGlobal)/dt;
    double zGlobal = wb_gps_get_values(gps)[2];
    double zGlobalVel = (zGlobal - pastZGlobal)/dt;

    // rotate to body frame
    actualState.x = xGlobal*cos(actualState.yaw) + yGlobal*sin(actualState.yaw);
    actualState.y = -xGlobal*sin(actualState.yaw) + yGlobal*cos(actualState.yaw);
    actualState.z = zGlobal;
    actualState.xVelocity = xGlobalVel*cos(actualState.yaw) + yGlobalVel*sin(actualState.yaw);
    actualState.yVelocity = -xGlobalVel*sin(actualState.yaw) + yGlobalVel*cos(actualState.yaw);
    actualState.zVelocity = zGlobalVel;


    // Initialize values for height Zs
    desiredState.z = 1.0;

    double forwardDesired = 0;
    double sidewaysDesired = 0;
    double yawDesired = 0;
    double heightVelocityDesired = 0;

    // Control altitude
    int key = wb_keyboard_get_key();

    // check keys
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          forwardDesired = + 0.5;
          //desiredState.x = 1;
          break;
        case WB_KEYBOARD_DOWN:
          forwardDesired = - 0.5;
          break;
        case WB_KEYBOARD_RIGHT:
          sidewaysDesired = - 0.5;
          break;
        case WB_KEYBOARD_LEFT:
          sidewaysDesired = + 0.5;
          break;
        case 'Q':
          yawDesired = 0.5;
          break;
        case 'E':
          yawDesired = - 0.5;
          break;
        case 'T':
          heightVelocityDesired = 0.5;
          break;
        case 'B':
          heightVelocityDesired = - 0.5;
          break;
        }
      key = wb_keyboard_get_key();
    }

    // Example how to get sensor data
    // range_front_value = wb_distance_sensor_get_value(range_front));
    // const unsigned char *image = wb_camera_get_image(camera);

    // yaw based on yaw desired
    desiredState.yawRate = yawDesired;

    statePID.modeXY = 0;
    statePID.modeZ = 0;
    statePID.modeYaw = 1;

    desiredState.roll = sidewaysDesired/5;
    desiredState.pitch = forwardDesired/5;
    desiredState.zVelocity = heightVelocityDesired;

    printf("Desired %f actual %f\n",forwardDesired, actualState.pitch);

    // full PID controller
    pid_pos_att_controller(&statePID, gainsPID, actualState, &desiredState, dt, &control);

    //motor mixing
    motor_mixing(control, &motorPower);

    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, - motorPower.m1);
    wb_motor_set_velocity(m2_motor, motorPower.m2);
    wb_motor_set_velocity(m3_motor, - motorPower.m3);
    wb_motor_set_velocity(m4_motor, motorPower.m4);


    // Save past time for next time step
    past_time = wb_robot_get_time();
    pastXGlobal = xGlobal;
    pastYGlobal = yGlobal;
    pastZGlobal = zGlobal;
    pastRoll = actualState.roll;
    pastPitch = actualState.pitch;
    pastYaw = actualState.yaw;

  };

  wb_robot_cleanup();

  return 0;
}
