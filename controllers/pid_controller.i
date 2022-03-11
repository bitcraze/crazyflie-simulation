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

void init_pid_attitude_fixed_height_controller();

void pid_attitude_fixed_height_controller(double rollActual, double pitchActual, double yawActual, double altitudeActual, 
    double rollDesired, double pitchDesired, double yawDesired, double altitudeDesired,
    double kp_att_rp, double kd_att_rp, double kp_att_y, double kd_att_y, double kp_z, double kd_z, double ki_z,
    double dt, MotorPower_t* motorCommands);
