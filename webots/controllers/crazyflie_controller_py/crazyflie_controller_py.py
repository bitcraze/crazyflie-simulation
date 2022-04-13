#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 
# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""


from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

from math import cos, sin

import sys
sys.path.append('../../../controllers/')
from  pid_controller import init_pid_attitude_fixed_height_controller, pid_velocity_fixed_height_controller
from pid_controller import MotorPower_t, ActualState_t, GainsPID_t, DesiredState_t
from pid_controller import motor_mixing, ControlCommands_t

sys.path.append('../../../../../C/crazyflie-firmware')
import cffirmware

robot = Robot()

timestep = int(robot.getBasicTimeStep())

## Initialize motors
m1_motor = robot.getDevice("m1_motor");
m1_motor.setPosition(float('inf'))
m1_motor.setVelocity(-1)
m2_motor = robot.getDevice("m2_motor");
m2_motor.setPosition(float('inf'))
m2_motor.setVelocity(1)
m3_motor = robot.getDevice("m3_motor");
m3_motor.setPosition(float('inf'))
m3_motor.setVelocity(-1)
m4_motor = robot.getDevice("m4_motor");
m4_motor.setPosition(float('inf'))
m4_motor.setVelocity(1)

## Initialize Sensors
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
Keyboard().enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
camera = robot.getDevice("camera")
camera.enable(timestep)
range_front = robot.getDevice("range_front")
range_front.enable(timestep)
range_left = robot.getDevice("range_left")
range_left.enable(timestep)
range_back = robot.getDevice("range_back")
range_back.enable(timestep)
range_right = robot.getDevice("range_right")
range_right.enable(timestep)

## Wait for two seconds
#while robot.step(timestep) != -1:
#    if robot.getTime()>2.0:
#        break
    
## Initialize variables
actualState = ActualState_t()
desiredState = DesiredState_t()
pastXGlobal = 0
pastYGlobal = 0
pastZGlobal = 0

past_time = robot.getTime()

## Initialize PID gains.
gainsPID = GainsPID_t()
gainsPID.kp_att_y = 1
gainsPID.kd_att_y = 0.5
gainsPID.kp_att_rp =0.5
gainsPID.kd_att_rp = 0.1
gainsPID.kp_vel_xy = 2;
gainsPID.kd_vel_xy = 0.5;
gainsPID.kp_z = 10
gainsPID.ki_z = 50
gainsPID.kd_z = 5
init_pid_attitude_fixed_height_controller();

## Initialize struct for motor power
motorPower = MotorPower_t()
controlCommands = ControlCommands_t()
cffirmware.controllerPidInit()

print('Take off!')

# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time;

    ## Get measurements
    actualState.roll = imu.getRollPitchYaw()[0]
    actualState.pitch = imu.getRollPitchYaw()[1]
    actualState.yaw_rate = gyro.getValues()[2];
    pitch_rate = gyro.getValues()[1];
    roll_rate = gyro.getValues()[0];
    actualState.altitude = gps.getValues()[2];
    xGlobal = gps.getValues()[0]
    vxGlobal = (xGlobal - pastXGlobal)/dt
    yGlobal = gps.getValues()[1]
    vyGlobal = (yGlobal - pastYGlobal)/dt
    zGlobal = gps.getValues()[2]
    vzGlobal = (zGlobal - pastZGlobal)/dt

    ## Get body fixed velocities
    actualYaw = imu.getRollPitchYaw()[2];
    cosyaw = cos(actualYaw)
    sinyaw = sin(actualYaw)
    actualState.vx = vxGlobal * cosyaw + vyGlobal * sinyaw
    actualState.vy = - vxGlobal * sinyaw + vyGlobal * cosyaw


    ## Initialize values
    desiredState.roll = 0
    desiredState.pitch = 0
    desiredState.vx = 0
    desiredState.vy = 0
    desiredState.yaw_rate = 0
    desiredState.altitude = 1.0

    forwardDesired = 0
    sidewaysDesired = 0
    yawDesired = 0

    key = Keyboard().getKey()
    while key>0:
        if key == Keyboard.UP:
            forwardDesired = 0.2
        elif key == Keyboard.DOWN:
            forwardDesired = -0.2
        elif key == Keyboard.RIGHT:
            sidewaysDesired = -0.2
        elif key == Keyboard.LEFT:
            sidewaysDesired = 0.2
        elif key == ord('Q'):
            yawDesired = 0.5
        elif key == ord('E'):
            yawDesired = -0.5

        key = Keyboard().getKey()

    ## Example how to get sensor data
    ## range_front_value = range_front.getValue();
    ## cameraData = camera.getImage()

    ## Firmware PID bindings
    control = cffirmware.control_t()
    setpoint = cffirmware.setpoint_t()
    setpoint.mode.z = 1;
    setpoint.position.z = 1;
    setpoint.mode.yaw = 2;
    setpoint.attitudeRate.yaw = yawDesired*180/3.14;
    setpoint.mode.x = 2;
    setpoint.mode.y = 2;
    setpoint.velocity.x = forwardDesired;
    setpoint.velocity.y = sidewaysDesired;
    setpoint.mode.x 
    setpoint.mode.z = 1
    setpoint.position.z = 1.0

    state = cffirmware.state_t()
    state.attitude.roll = actualState.roll*180/3.14
    state.attitude.pitch = -actualState.pitch*180/3.14
    state.attitude.yaw = actualYaw*180/3.14
    state.position.x = xGlobal
    state.position.y = yGlobal
    state.position.z = zGlobal
    state.velocity.x = vxGlobal
    state.velocity.y = vyGlobal
    state.velocity.z = vzGlobal

    sensors = cffirmware.sensorData_t()
    sensors.gyro.x =  roll_rate*180/3.14
    sensors.gyro.y = -pitch_rate*180/3.14
    sensors.gyro.z = actualState.yaw_rate*180/3.14

    tick = 100



    cffirmware.controllerPid(control, setpoint,sensors,state,tick)
    print(state.attitude.pitch,control.pitch )

    controlCommands.roll = control.roll*3.14/180
    controlCommands.pitch = control.pitch*3.14/180
    controlCommands.yaw = control.yaw*3.14/180
    controlCommands.altitude = control.thrust

    motor_mixing(controlCommands, motorPower);


    #print(controlCommands.altitude/1000)
    ## PID velocity controller with fixed height
    '''
    desiredState.yaw_rate = yawDesired;
    desiredState.vy = sidewaysDesired;
    desiredState.vx = forwardDesired;
    pid_velocity_fixed_height_controller(actualState, desiredState,
    gainsPID, dt, motorPower);
    '''
    

    ## PID attitude controller with fixed height
    '''
    desiredState.yaw_rate = yawDesired;
    desiredState.roll = sidewaysDesired;
    desiredState.pitch = forwardDesired;
     pid_attitude_fixed_height_controller(actualState, desiredState,
    gainsPID, dt, motorPower);
    '''
    scaling = 1000
    m1_motor.setVelocity(-motorPower.m1/scaling)
    m2_motor.setVelocity(motorPower.m2/scaling)
    m3_motor.setVelocity(-motorPower.m3/scaling)
    m4_motor.setVelocity(motorPower.m4/scaling)
    
    past_time = robot.getTime()
    pastXGlobal = xGlobal
    pastYGlobal = yGlobal
    pastZGlobal = zGlobal


    pass
