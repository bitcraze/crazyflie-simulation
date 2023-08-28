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
file: crazyflie_controller_py_socket.py

Controls the crazyflie via a socket in Python

Author:   Kimberly McGuire (Bitcraze AB) and Simon D. Levy
"""


from controller import Robot

import socket
from struct import pack, unpack
from threading import Thread
from math import cos, sin, degrees
from time import sleep

import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1

# Client comms
HOST = '127.0.0.1'
PORT = 5000

# Scaling factors between client sticks demands and PID inputs
THROTTLE_SCALEDOWN = 10
CYCLIC_SCALEDOWN = 60
YAW_SCALEDOWN = 500


def threadfun(conn, pose, velocity_socket_data):
    '''Threaded Gommunication with GUI client'''

    while True:

        try:
            conn.send(pack('ffffff',
                           pose['x'],
                           pose['y'],
                           pose['z'],
                           degrees(pose['phi']),
                           degrees(pose['theta']),
                           degrees(pose['psi'])))


            (velocity_socket_data[0],
             velocity_socket_data[1],
             velocity_socket_data[2],
             velocity_socket_data[3]) = unpack('ffff', conn.recv(16))



        except Exception as e:  # client disconnected
            print('Error on comms thread: ' + str(e))
            break

        sleep(0)  # yield to main thread

def make_motor(robot, motor_name, spin):
    '''Helper'''

    motor = robot.getDevice(motor_name)
    motor.setPosition(float('inf'))
    motor.setVelocity(spin)

    return motor

def make_sensor(robot, sensor_name, timestep):

    sensor = robot.getDevice(sensor_name)
    sensor.enable(timestep)

    return sensor


if __name__ == '__main__':

    # Make socket to talk to the client and wait for client ot join
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
    sock.bind((HOST, PORT))
    sock.listen(1)
    conn, _ = sock.accept()

    # Need to get assist mode and raw stick values from client
    velocity_socket_data = [0, 0, 0, 0]

    # This data will be sent to the GUI
    pose = {'x': 0, 'y': 0, 'z': 0, 'phi': 0, 'theta': 0, 'psi': 0}

    # Start thread for communicating with client
    Thread(target=threadfun, args=(conn, pose, velocity_socket_data)).start()

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = make_motor(robot, 'm1_motor', +1)
    m2_motor = make_motor(robot, 'm2_motor', -1)
    m3_motor = make_motor(robot, 'm3_motor', +1)
    m4_motor = make_motor(robot, 'm4_motor', -1)

    # Initialize Sensors
    imu = make_sensor(robot, 'inertial_unit', timestep)
    gps = make_sensor(robot, 'gps', timestep)
    gyro = make_sensor(robot, 'gyro', timestep)

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

    # Main loop:
    while robot.step(timestep) != -1:

        # Get stick demands from client
        forward_desired = velocity_socket_data[0]
        sideways_desired = velocity_socket_data[1]
        height_diff_desired = velocity_socket_data[2]
        yaw_desired = -velocity_socket_data[3] # note the minus sign

        dt = robot.getTime() - past_time
        actual_state = {}

        if first_time:
            past_x_global, past_y_global, _ = gps.getValues()
            past_time = robot.getTime()
            first_time = False

        # Get sensor data
        roll, pitch, yaw = imu.getRollPitchYaw()
        _, _, yaw_rate = gyro.getValues()
        x_global, y_global, altitude = gps.getValues()

        # Fill in pose
        pose['x'] = x_global
        pose['y'] = y_global
        pose['z'] = altitude
        pose['phi'] = roll
        pose['theta'] = pitch
        pose['psi'] = yaw

        v_x_global = (x_global - past_x_global)/dt
        v_y_global = (y_global - past_y_global)/dt

        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw

        height_desired += height_diff_desired * dt

        # Run PID velocity controller with fixed height
        motor_power = PID_crazyflie.pid(dt, forward_desired, sideways_desired,
                                        yaw_desired, height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)

        # Run motors
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
