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

typedef struct ControlCommands_s{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
} ControlCommands_t;

typedef struct State_s{
  double rollRate;
  double pitchRate;
  double yawRate;
  double roll;
  double pitch;
  double yaw;
  double xVelocity;
  double yVelocity;
  double zVelocity;
  double x;
  double y;
  double z;
} State_t;

typedef struct GainsPID_s{
  double rollpitchKP;
  double rollpitchKD;
  double rollpitchKI;
  double yawKP;
  double yawKD;
  double yawKI;
  double yawRateKP;
  double yawRateKD;
  double yawRateKI;
  double xyKP;
  double xyKD;
  double xyKI;
  double xyVelocityKP;
  double xyVelocityKD;
  double xyVelocityKI;
  double zKP;
  double zKD;
  double zKI;
  double zVelocityKP;
  double zVelocityKD;
  double zVelocityKI;
} GainsPID_t;

typedef struct StatePID_s
{
  double rollError;
  double prevRollError;
  double rollInteg;
  double rollDeriv;
  double pitchError;
  double prevPitchError;
  double pitchInteg;
  double pitchDeriv;
  double yawError;
  double prevYawError;
  double yawInteg;
  double yawDeriv;
  double xError;
  double prevXError;
  double xInteg;
  double xDeriv;
  double yError;
  double prevYError;
  double yInteg;
  double yDeriv;
  double zError;
  double prevZError;
  double zInteg;
  double zDeriv;
  int modeXY;
  int modeZ;
  int modeRollPitch;
  int modeYaw;

} StatePID_t;

float constrain(float value, const float minVal, const float maxVal);

void motor_mixing(ControlCommands_t controlCommands, MotorPower_t* motorCommands);

void pid_attitude_controller(StatePID_t *pidState, GainsPID_t gainsPID,
    State_t actualState, State_t* desiredState,
    double dt, ControlCommands_t* controlCommands);

void pid_position_controller(StatePID_t *pidState, GainsPID_t gainsPID,
    State_t actualState, State_t* desiredState,
    double dt, ControlCommands_t* controlCommands);

void pid_pos_att_controller(StatePID_t* pidState, GainsPID_t gainsPID,
    State_t actualState, State_t* desiredState,
    double dt, ControlCommands_t* controlCommands);

double pid_controller(double desired, double actual, double dt,
    double *error, double *prevError, double*integ,
    double *deriv, double kp, double ki, double kd);