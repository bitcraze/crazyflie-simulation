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
 * @file pid_controller.c
 * A simple PID controller for attitude control of an 
 * quadcopter 
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pid_controller.h"

float constrain(float value, const float minVal, const float maxVal)
{
  return fminf(maxVal, fmaxf(minVal,value));
}


double pastAltitudeError, pastYawError, pastPitchError, pastRollError;
double pastVxError, pastVyError;

void init_pid_attitude_fixed_height_controller()
{
    pastAltitudeError = 0;
    pastYawError = 0;
    pastPitchError = 0;
    pastRollError = 0;
}


MotorPower_t pid_attitude_fixed_height_controller(double rollActual, double pitchActual, double yawActual, double altitudeActual, 
    double rollDesired, double pitchDesired, double yawDesired, double altitudeDesired,
    double kp_att_rp, double kd_att_rp, double kp_att_y, double kd_att_y, double kp_z, double kd_z, double ki_z,
    double dt, MotorPower_t* motorCommands)
{

    // Calculate errors
    double altitudeError = altitudeDesired - altitudeActual;
    double altitudeDerivativeError = (altitudeError - pastAltitudeError)/dt;
    double yawError = yawDesired - yawActual;
    double yawDerivativeError = (yawError - pastYawError)/dt;
    double pitchError = pitchDesired - pitchActual;
    double pitchDerivativeError = (pitchError - pastPitchError)/dt;
    double rollError = rollDesired - rollActual;
    double rollDerivativeError = (rollError - pastRollError)/dt;

    //PID control
    double rollControl =kp_att_rp * constrain(rollError,-1, 1) + kd_att_rp*rollDerivativeError;
    double pitchControl =-kp_att_rp * constrain(pitchError,-1, 1) - kd_att_rp*pitchDerivativeError;
    double yawControl = kp_att_y * constrain(yawError, -1, 1)+ kd_att_y*yawDerivativeError;
    double altitudeControl = kp_z * constrain(altitudeError, -1, 1) + kd_z*altitudeDerivativeError + ki_z;
    
    // Motor mixing
    motorCommands->m1 =  altitudeControl -rollControl + pitchControl + yawControl;
    motorCommands->m2 =  altitudeControl - rollControl - pitchControl - yawControl;
    motorCommands->m3 =  altitudeControl + rollControl - pitchControl + yawControl;
    motorCommands->m4 =  altitudeControl + rollControl + pitchControl - yawControl;


    MotorPower_t motorCommandsReturn;
    memcpy(&motorCommandsReturn, motorCommands, sizeof(MotorPower_t));
    // Save error for the next round
    pastAltitudeError = altitudeError;
    pastYawError = yawError;
    pastPitchError= pitchError;
    pastRollError= rollError;

    return motorCommandsReturn;
}


MotorPower_t pid_velocity_controller(double vxActual, double vyActual, double yawActual, double altitudeActual, 
    double vxDesired, double vyDesired, double yawDesired, double vzDesired,
    double kp_vel_xy, double kd_vel_xy, double kp_att_y, double kd_att_y, double kp_z, double kd_z, double ki_z,
    double dt, MotorPower_t* motorCommands)
{


    double altitudeDesired = vzDesired;

    double altitudeError = altitudeDesired - altitudeActual;
    double altitudeDerivativeError = (altitudeError - pastAltitudeError)/dt;
    double yawError = yawDesired - yawActual;
    double yawDerivativeError = (yawError - pastYawError)/dt;
    double vxError = vxDesired - vxActual;
    double vxDerivative = (vxError - pastVxError)/dt;
    double vyError = vyDesired - vyActual;
    double vyDerivative = (vyError - pastVyError)/dt;


    //PID control
    double vxControl =kp_vel_xy * constrain(vxError,-1, 1) + kd_vel_xy*vxDerivative;
    double vyControl =-kp_vel_xy * constrain(vyError,-1, 1) - kd_vel_xy*vyDerivative;
    double yawControl = kp_att_y * constrain(yawError, -1, 1)+ kd_att_y*yawDerivativeError;
    double altitudeControl = kp_z * constrain(altitudeError, -1, 1) + kd_z*altitudeDerivativeError + ki_z;
    
    // Motor mixing
    motorCommands->m1 =  altitudeControl - vyControl + vxControl + yawControl;
    motorCommands->m2 =  altitudeControl - vyControl - vxControl - yawControl;
    motorCommands->m3 =  altitudeControl + vyControl - vxControl + yawControl;
    motorCommands->m4 =  altitudeControl + vyControl + vxControl - yawControl;


    MotorPower_t motorCommandsReturn;
    memcpy(&motorCommandsReturn, motorCommands, sizeof(MotorPower_t));

    pastAltitudeError = altitudeError;
    pastYawError = yawError;
    pastVxError = vxError;
    pastVyError = vyError;

}