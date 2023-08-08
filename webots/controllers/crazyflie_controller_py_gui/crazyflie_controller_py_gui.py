# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2023 Bitcraze


"""
file: crazyflie_py_wallfollowing.py

Controls the crazyflie and implements a wall following method in webots in
Python

Author:   Kimberly McGuire (Bitcraze AB) and Simon D. Levy
"""


from controller import Robot
from controller import Keyboard

import socket
import threading
from math import cos, sin, degrees
from time import sleep

import sys
sys.path.append('../../../controllers/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1

# Client comms
HOST = '127.0.0.1'
PORT = 5000

# Scaling factors between client sticks demands and PID inputs
THROTTLE_SCALEDOWN = 100
CYCLIC_SCALEDOWN = 60
YAW_SCALEUP = 5


def threadfun(conn, pose, client_data):
    '''Threaded Gommunication with GUI client'''

    while True:

        try:
            conn.send(struct.pack('ffffff',
                                  pose['x'],
                                  pose['y'],
                                  pose['z'],
                                  math.degrees(pose['phi']),
                                  math.degrees(pose['theta']),
                                  math.degrees(pose['psi'])))

            (
                    client_data[0],
                    client_data[1],
                    client_data[2],
                    client_data[3],
                    client_data[4]) = struct.unpack('fffff', conn.recv(20))

        except Exception:  # client disconnected
            break

        time.sleep(0)  # yield to main thread


def deadband(x):
    '''Stick deadband for roll, pitch'''

    y = x / CYCLIC_SCALEDOWN

    return 0 if abs(y) < 0.2 else +0.5 if y > 0 else -0.5


if __name__ == '__main__':

    # Make socket to talk to the client and wait for client ot join
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
    sock.bind((HOST, PORT))
    sock.listen(1)
    conn, _ = sock.accept()

    # Need to get assist mode and raw stick values from client
    client_data = [0, 0, 0, 0, 0]

    # This data will be sent to the GUI
    pose = {'x': 0, 'y': 0, 'z': 0, 'phi': 0, 'theta': 0, 'psi': 0}

    # Start thread for communicating with client
    threading.Thread(target=threadfun,
                     args=(conn, pose, client_data)).start()

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize Sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("position")
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

    # Initialize variables

    past_x_global = 0
    past_y_global = 0
    past_time = 0
    first_time = True

    # Crazyflie velocity PID controller
    PID_crazyflie = pid_velocity_fixed_height_controller()
    PID_update_last_time = robot.getTime()
    sensor_read_last_time = robot.getTime()

    height_desired = FLYING_ATTITUDE

    # Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Main loop:
    while robot.step(timestep) != -1:

        # Get desired assist mode from client: 
        # 1 = none; 2 = altitude-hold; 3 = hover
        # XXX We currently ignore this and stay in mode 3
        mode = int(client_data[0])

        # Get stick demands from client
        gui_height_diff_desired = client_data[1] / THROTTLE_SCALEDOWN
        gui_sideways_demand = -deadband(client_data[2])  # note negation
        gui_forward_demand = deadband(client_data[3])
        gui_yaw_demand = -client_data[4] * YAW_SCALEUP  # note negation

        dt = robot.getTime() - past_time
        actual_state = {}

        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()
            first_time = False

        # Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global)/dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global)/dt
        altitude = gps.getValues()[2]

        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

        # Initialize values
        desired_state = [0, 0, 0, 0]
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                forward_desired += 0.5
            elif key == Keyboard.DOWN:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired = + 1
            elif key == ord('E'):
                yaw_desired = - 1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = - 0.1
            key = keyboard.getKey()



        height_desired += height_diff_desired * dt

        # PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
