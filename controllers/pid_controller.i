/* File: pid_controller.i */
%module pid_controller

%{
#define SWIG_FILE_WITH_INIT
#include "pid_controller.h"
%}

typedef struct MotorPower_s{
  double m1;
  double m2;
  double m3;
  double m4;
} MotorPower_t;

typedef struct DesiredState_s{
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} DesiredState_t;

typedef struct ActualState_s{
  double roll;
  double pitch;
  double yaw_rate;
  double altitude;
  double vx;
  double vy;
} ActualState_t;

typedef struct GainsPID_s{
  double kp_att_rp;
  double kd_att_rp;
  double kp_att_y;
  double kd_att_y;
  double kp_vel_xy;
  double kd_vel_xy;
  double kp_z;
  double kd_z;
  double ki_z;
} GainsPID_t;

void init_pid_attitude_fixed_height_controller();

void pid_attitude_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands);

void pid_velocity_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands);