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
sys.path.append('../../../controllers/python')
from pid_controller import pid_velocity_fixed_height_controller
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

## Get keyboard
keyboard = Keyboard()
keyboard.enable(timestep)
    
## Initialize variables

pastXGlobal = 0
pastYGlobal = 0
past_time = robot.getTime()

# Crazyflie velocity PID controller
PID_CF = pid_velocity_fixed_height_controller()
PID_update_last_time = robot.getTime()
sensor_read_last_time = robot.getTime()
step_count = 0


# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time

    actual_state = {}

    ## Get sensor data
    roll = imu.getRollPitchYaw()[0]
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    yaw_rate = gyro.getValues()[2]
    altitude = gps.getValues()[2]
    xGlobal = gps.getValues()[0]
    vxGlobal = (xGlobal - pastXGlobal)/dt
    yGlobal = gps.getValues()[1]
    vyGlobal = (yGlobal - pastYGlobal)/dt

    ## Get body fixed velocities
    cosyaw = cos(yaw)
    sinyaw = sin(yaw)
    v_x = vxGlobal * cosyaw + vyGlobal * sinyaw
    v_y = - vxGlobal * sinyaw + vyGlobal * cosyaw

    ## Initialize values
    desired_state = [0, 0, 0, 0]
    forwardDesired = 0
    sidewaysDesired = 0
    yawDesired = 0

    key = keyboard.getKey()
    while key>0:
        if key == Keyboard.UP:
            forwardDesired += 0.2
        elif key == Keyboard.DOWN:
            forwardDesired -= 0.2
        elif key == Keyboard.RIGHT:
            sidewaysDesired -= 0.2
        elif key == Keyboard.LEFT:
            sidewaysDesired += 0.2
        elif key == ord('Q'):
            yawDesired =  + 0.5
        elif key == ord('E'):
            yawDesired = - 0.5

        key = keyboard.getKey()

    ## Example how to get sensor data
    ## range_front_value = range_front.getValue();
    ## cameraData = camera.getImage()
    
    desired_state[0] = forwardDesired
    desired_state[1] = sidewaysDesired
    desired_state[2] = yawDesired
    desired_state[3] = 1
    ## PID velocity controller with fixed height
    motor_power = PID_CF.pid(dt, desired_state, roll, pitch, yaw_rate,
                            altitude, v_x, v_y)

    m1_motor.setVelocity(-motor_power[0])
    m2_motor.setVelocity(motor_power[1])
    m3_motor.setVelocity(-motor_power[2])
    m4_motor.setVelocity(motor_power[3])
    
    past_time = robot.getTime()
    pastXGlobal = xGlobal
    pastYGlobal = yGlobal

    pass
