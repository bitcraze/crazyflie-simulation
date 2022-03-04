#pragma once

typedef struct MotorPower_s{
  double m1;
  double m2;
  double m3;
  double m4;
} MotorPower_t;

float constrain(float value, const float minVal, const float maxVal);
void init_pid_attitude_fixed_height_controller();
void pid_attitude_fixed_height_controller(double rollActual, double pitchActual, double yawActual, double altitudeActual, 
    double rollDesired, double pitchDesired, double yawDesired, double altitudeDesired,
    double kp_rp, double kd_rp, double kp_y, double kd_yaw, double kp_alt, double kd_alt, double ki_alt,
    double dt, MotorPower_t* motorCommands);
