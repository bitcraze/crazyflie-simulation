#pragma once

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
 * @file pid_controller.h
 * PID controller header file
 * 
 */

typedef struct MotorPower_s{
  double m1;
  double m2;
  double m3;
  double m4;
} MotorPower_t;

float constrain(float value, const float minVal, const float maxVal);
void init_pid_attitude_fixed_height_controller();

MotorPower_t pid_attitude_fixed_height_controller(double rollActual, double pitchActual, double yawActual, double altitudeActual, 
    double rollDesired, double pitchDesired, double yawDesired, double altitudeDesired,
    double kp_rp, double kd_rp, double kp_y, double kd_yaw, double kp_alt, double kd_alt, double ki_alt,
    double dt, MotorPower_t* motorCommands);

MotorPower_t pid_velocity_controller(double vxActual, double vyActual, double yawActual, double altitudeActual, 
    double vxDesired, double vyDesired, double yawDesired, double vzDesired,
    double kp_vel_xy, double kd_vel_xy, double kp_att_y, double kd_att_y, double kp_z, double kd_z, double ki_z,
    double dt, MotorPower_t* motorCommands);
