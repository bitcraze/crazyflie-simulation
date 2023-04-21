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

from python.pid_controller_full_state import PIDControllerFullState


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
actual_state = [[0 for x in range(3)] for y in range(4)]
desired_state = [[0 for x in range(3)] for y in range(4)]
pos_mode = [0, 0, 0]
att_mode = [0, 0, 0]

past_time = robot.getTime()
past_x_global = 0
past_y_global = 0

## Initialize PID gains.
gains_pid_pos = [[0 for x in range(3)] for y in range(3)]
gains_pid_pos[0][0] = 1.0
gains_pid_pos[0][1] = 0.0
gains_pid_pos[0][2] = 0.0
gains_pid_pos[1][0] = 1.0
gains_pid_pos[1][1] = 0.0
gains_pid_pos[1][2] = 0.0
gains_pid_pos[2][0] = 10
gains_pid_pos[2][1] = 10
gains_pid_pos[2][2] = 5

gains_pid_vel = [[0 for x in range(3)] for y in range(3)]
gains_pid_vel[0][0] = 1.0
gains_pid_vel[0][1] = 0.0
gains_pid_vel[0][2] = 0.0
gains_pid_vel[1][0] = 1.0
gains_pid_vel[1][1] = 0.0
gains_pid_vel[1][2] = 0.0
gains_pid_vel[2][0] = 1.0
gains_pid_vel[2][1] = 0.0
gains_pid_vel[2][2] = 0.0

gains_pid_att = [[0 for x in range(3)] for y in range(3)]
gains_pid_att[0][0] = 1.0
gains_pid_att[0][1] = 0.0
gains_pid_att[0][2] = 0.5
gains_pid_att[1][0] = 0.5
gains_pid_att[1][1] = 0.0
gains_pid_att[1][2] = 0.1
gains_pid_att[2][0] = 1.0
gains_pid_att[2][1] = 0.0
gains_pid_att[2][2] = 0.0

gains_pid_rate = [[0 for x in range(3)] for y in range(3)]
gains_pid_rate[0][0] = 1.0
gains_pid_rate[0][1] = 0.0
gains_pid_rate[0][2] = 0.0
gains_pid_rate[1][0] = 1.0
gains_pid_rate[1][1] = 0.0
gains_pid_rate[1][2] = 0.0
gains_pid_rate[2][0] = 1.0
gains_pid_rate[2][1] = 0.0
gains_pid_rate[2][2] = 0.0

controller = PIDControllerFullState(gains_pid_pos, gains_pid_vel, gains_pid_att, gains_pid_rate)

# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time;

    ## Get measurements
    roll = imu.getRollPitchYaw()[0]
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    roll_rate = gyro.getValues()[0]
    pitch_rate = gyro.getValues()[1]
    yaw_rate = gyro.getValues()[2]
    altitude = gps.getValues()[2]
    x_global = gps.getValues()[0]
    vx_global = (x_global - past_x_global)/dt
    y_global = gps.getValues()[1]
    vy_global = (y_global - past_y_global)/dt

    cosyaw = cos(yaw)
    sinyaw = sin(yaw)
    x_body = x_global * cosyaw + y_global * sinyaw
    y_body = - x_global * sinyaw + y_global * cosyaw
    z_body = altitude
    vx_body = vx_global * cosyaw + vy_global * sinyaw
    vy_body = - vx_global * sinyaw + vy_global * cosyaw
    vz_body = altitude

    actual_state[0][0] = x_body
    actual_state[0][1] = y_body
    actual_state[0][2] = z_body
    actual_state[1][0] = vx_body
    actual_state[1][1] = vy_body
    actual_state[1][2] = vz_body
    actual_state[2][0] = roll
    actual_state[2][1] = pitch
    actual_state[2][2] = yaw
    actual_state[3][0] = roll_rate
    actual_state[3][1] = pitch_rate
    actual_state[3][2] = yaw_rate

    desired_state[0][0] = 0
    desired_state[0][1] = 0
    desired_state[0][2] = 0
    desired_state[1][0] = 0
    desired_state[1][1] = 0
    desired_state[1][2] = 0
    desired_state[2][0] = 0
    desired_state[2][1] = 0
    desired_state[2][2] = 0
    desired_state[3][0] = 0
    desired_state[3][1] = 0
    desired_state[3][2] = 0

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

    desired_state[0][2] = 1
    desired_state[1][0] = forwardDesired
    desired_state[1][1] = sidewaysDesired
    desired_state[3][0] = yawDesired

    pos_mode[0] = 1
    pos_mode[1] = 1
    pos_mode[2] = 0

    att_mode[0] = 0
    att_mode[1] = 0
    att_mode[2] = 1

    cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust = controller.update(actual_state, desired_state, pos_mode, att_mode, 0, dt)

    ## Motor mixing
    motor_power_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
    motor_power_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
    motor_power_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
    motor_power_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw

    m1_motor.setVelocity(-motor_power_m1)
    m2_motor.setVelocity(motor_power_m2)
    m3_motor.setVelocity(-motor_power_m3)
    m4_motor.setVelocity(motor_power_m4)

    past_time = robot.getTime()
    past_x_global = x_global
    past_y_global = y_global

    pass
