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


void pid_attitude_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands)
{

    ControlCommands_t controlCommands = {0};
    pid_fixed_height_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    pid_attitude_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    motor_mixing(controlCommands, motorCommands);

}


void pid_velocity_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, MotorPower_t* motorCommands)
{
    ControlCommands_t controlCommands = {0};
    pid_horizontal_velocity_controller(actualState, 
    desiredState, gainsPID, dt);
    pid_fixed_height_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    pid_attitude_controller(actualState, 
    desiredState, gainsPID, dt, &controlCommands);
    motor_mixing(controlCommands, motorCommands);

}

void pid_fixed_height_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, ControlCommands_t* controlCommands)
{
    double altitudeError = desiredState->altitudeDesired - actualState.altitudeActual;
    double altitudeDerivativeError = (altitudeError - pastAltitudeError)/dt;
    controlCommands->altitudeControl = gainsPID.kp_z * constrain(altitudeError, -1, 1) + gainsPID.kd_z*altitudeDerivativeError + gainsPID.ki_z;
    pastAltitudeError = altitudeError;

}

void motor_mixing(ControlCommands_t controlCommands, MotorPower_t* motorCommands)
{
    // Motor mixing
    motorCommands->m1 =  controlCommands.altitudeControl - controlCommands.rollControl + controlCommands.pitchControl + controlCommands.yawControl;
    motorCommands->m2 =  controlCommands.altitudeControl - controlCommands.rollControl - controlCommands.pitchControl - controlCommands.yawControl;
    motorCommands->m3 =  controlCommands.altitudeControl + controlCommands.rollControl - controlCommands.pitchControl + controlCommands.yawControl;
    motorCommands->m4 =  controlCommands.altitudeControl + controlCommands.rollControl + controlCommands.pitchControl - controlCommands.yawControl;
}


void pid_attitude_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt, ControlCommands_t* controlCommands)
{

    // Calculate errors

    double yawError = desiredState->yawDesired - actualState.yawActual;
    double yawDerivativeError = (yawError - pastYawError)/dt;
    double pitchError = desiredState->pitchDesired - actualState.pitchActual;
    double pitchDerivativeError = (pitchError - pastPitchError)/dt;
    double rollError = desiredState->rollDesired - actualState.rollActual;
    double rollDerivativeError = (rollError - pastRollError)/dt;

    //PID control
    controlCommands->rollControl = gainsPID.kp_att_rp * constrain(rollError,-1, 1) + gainsPID.kd_att_rp*rollDerivativeError;
    controlCommands->pitchControl =-gainsPID.kp_att_rp * constrain(pitchError,-1, 1) - gainsPID.kd_att_rp*pitchDerivativeError;
    controlCommands->yawControl = gainsPID.kp_att_y * constrain(yawError, -1, 1)+ gainsPID.kd_att_y*yawDerivativeError;
    
    // Save error for the next round
    pastYawError = yawError;
    pastPitchError= pitchError;
    pastRollError= rollError;

}


void pid_horizontal_velocity_controller(ActualState_t actualState, 
    DesiredState_t* desiredState, GainsPID_t gainsPID,
    double dt)
{

    double vxError = desiredState->vxDesired - actualState.vxActual;
    double vxDerivative = (vxError - pastVxError)/dt;
    double vyError = desiredState->vyDesired - actualState.vyActual;
    double vyDerivative = (vyError - pastVyError)/dt;


    //PID control
    double pitchCommand = gainsPID.kp_vel_xy * constrain(vxError,-1, 1) + gainsPID.kd_vel_xy*vxDerivative;
    double rollCommand = - gainsPID.kp_vel_xy * constrain(vyError,-1, 1) - gainsPID.kd_vel_xy*vyDerivative;
    
    desiredState->pitchDesired = pitchCommand;
    desiredState->rollDesired = rollCommand;

    // Save error for the next round
    pastVxError = vxError;
    pastVyError = vyError;

}