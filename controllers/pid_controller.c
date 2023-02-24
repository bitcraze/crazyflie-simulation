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


void motor_mixing(ControlCommands_t controlCommands, MotorPower_t* motorCommands)
{
    // Motor mixing
    motorCommands->m1 =  controlCommands.z - controlCommands.roll - controlCommands.pitch + controlCommands.yaw;
    motorCommands->m2 =  controlCommands.z - controlCommands.roll + controlCommands.pitch - controlCommands.yaw;
    motorCommands->m3 =  controlCommands.z + controlCommands.roll + controlCommands.pitch + controlCommands.yaw;
    motorCommands->m4 =  controlCommands.z + controlCommands.roll - controlCommands.pitch - controlCommands.yaw;
}

void calculate_errors(double desired, double actual, double dt,
    double *error, double *prevError, double*integ,
    double *deriv)
{
    *error = desired - actual;
    *deriv = (*error - *prevError)/dt;
    *integ += *error * dt;
    *prevError = *error;
}

double pid_controller(double desired, double actual, double dt,
    double *error, double *prevError, double*integ,
    double *deriv, double kp, double ki, double kd)
{
    calculate_errors(desired, actual, dt, error, prevError, integ, deriv);
    return kp * constrain(*error, -1, 1) + kd * (*deriv) + ki * (*integ);
}

void pid_attitude_controller(StatePID_t *pidState, GainsPID_t gainsPID,
    State_t actualState, State_t* desiredState,
    double dt, ControlCommands_t* controlCommands)
{
    printf("desiredState->pitch: %f , actualState->pitch: %f \n", desiredState->pitch, actualState.pitch);
        // Calculate errors
    double rollcmd = pid_controller(desiredState->roll, actualState.roll, dt,
        &(pidState->rollError), &(pidState->prevRollError), &(pidState->rollInteg),
        &(pidState->rollDeriv), gainsPID.rollpitchKP, gainsPID.rollpitchKI, gainsPID.rollpitchKD);

    double pitchcmd = pid_controller(desiredState->pitch, actualState.pitch, dt,
        &(pidState->pitchError), &(pidState->prevPitchError), &(pidState->pitchInteg),
        &(pidState->pitchDeriv), gainsPID.rollpitchKP, gainsPID.rollpitchKI, gainsPID.rollpitchKD);

    double yawcmd = pid_controller(desiredState->yaw, actualState.yaw, dt,
        &(pidState->yawError), &(pidState->prevYawError), &(pidState->yawInteg),
        &(pidState->yawDeriv), gainsPID.yawKP, gainsPID.yawKI, gainsPID.yawKD);

    controlCommands->roll = rollcmd;
    controlCommands->pitch = pitchcmd;
    controlCommands->yaw = yawcmd;
}

void pid_position_controller(StatePID_t *pidState, GainsPID_t gainsPID,
    State_t actualState, State_t* desiredState,
    double dt, ControlCommands_t* controlCommands)
{
    // Calculate errors
    double xcmd = pid_controller(desiredState->x, actualState.x, dt,
        &(pidState->xError), &(pidState->prevXError), &(pidState->xInteg),
        &(pidState->xDeriv), gainsPID.xyKP, gainsPID.xyKI, gainsPID.xyKD);

    double ycmd = pid_controller(desiredState->y, actualState.y, dt,
        &(pidState->yError), &(pidState->prevYError), &(pidState->yInteg),
        &(pidState->yDeriv), gainsPID.xyKP, gainsPID.xyKI, gainsPID.xyKD);

    double zcmd = pid_controller(desiredState->z, actualState.z, dt,
        &(pidState->zError), &(pidState->prevZError), &(pidState->zInteg),
        &(pidState->zDeriv), gainsPID.zKP, gainsPID.zKI, gainsPID.zKD);

    controlCommands->x = xcmd;
    controlCommands->y = ycmd;
    controlCommands->z = zcmd;
}

void pid_pos_att_controller(StatePID_t* pidState, GainsPID_t gainsPID,
    State_t actualState, State_t* desiredState,
    double dt, ControlCommands_t* controlCommands)
{

    pid_position_controller(pidState, gainsPID, actualState, desiredState, dt, controlCommands);

    //desiredState->roll = controlCommands->y;
    //desiredState->pitch = controlCommands->x;
    pid_attitude_controller(pidState, gainsPID, actualState, desiredState, dt, controlCommands);

}
