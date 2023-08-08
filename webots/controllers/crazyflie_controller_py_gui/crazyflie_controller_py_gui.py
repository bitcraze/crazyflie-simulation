'''
Controls the crazyflie from the GUI client over a socket connection

Authors: Kimberly McGuire (Bitcraze AB) & Simon D. Levy

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 51
Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
'''

import numpy as np
import socket
import threading
import time
import struct

from controller import Robot

import cffirmware

from pids import AltitudePidController
from pids import PositionPidController
from pids import AnglePidController

HOST = '127.0.0.1'
PORT = 5000


def mix(demands):

    # Rescale throttle from [0,1] to [48,58]
    t = 10 * demands['throttle'] + 48

    r = demands['roll']
    p = demands['pitch']
    y = demands['yaw']

    m1 = t - r + p + y
    m2 = t - r - p - y
    m3 = t + r - p + y
    m4 = t + r + p - y

    return -m1, +m2, -m3, +m4


def _threadfun(conn, pose, client_data):

    while True:

        try:
            conn.send(struct.pack('ffffff',
                                  pose['x'],
                                  pose['y'],
                                  pose['z'],
                                  np.degrees(pose['phi']),
                                  np.degrees(pose['theta']),
                                  np.degrees(pose['psi'])))

            (
                    client_data[0],
                    client_data[1],
                    client_data[2],
                    client_data[3],
                    client_data[4]) = struct.unpack('fffff', conn.recv(20))

        except Exception:  # client disconnected
            break

        time.sleep(0.001)


def mkmotor(name, spin):

    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(spin)

    return motor


def mksensor(robot, name, timestep):

    sensor = robot.getDevice(name)
    sensor.enable(timestep)

    return sensor


def deadband(x):

    return 0 if abs(x) < 0.2 else +0.5 if x > 0 else -0.5


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
    threading.Thread(target=_threadfun,
                     args=(conn, pose, client_data)).start()

    robot = Robot()

    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = mkmotor('m1_motor', -1)
    m2_motor = mkmotor('m2_motor', +1)
    m3_motor = mkmotor('m3_motor', -1)
    m4_motor = mkmotor('m4_motor', +1)

    # Initialize sensors
    imu = mksensor(robot, 'inertial_unit', timestep)
    pos = mksensor(robot, 'position', timestep)
    gyro = mksensor(robot, 'gyro', timestep)
    camera = mksensor(robot, 'camera', timestep)

    # Initialize variables for computing timestep, horizontal velocity
    pose_x_prev = 0
    pose_y_prev = 0
    time_prev = 0

    # Main loop:
    while robot.step(timestep) != -1:

        # Read sensors
        phi, theta, psi = imu.getRollPitchYaw()
        dphi_dt, dtheta_dt, dpsi_dt = gyro.getValues()

        # We need a previous time to compute the timestep
        if time_prev != 0:

            dt = robot.getTime() - time_prev

            # Convert stick demands to [-1, +1]
            '''
            demands = {
                    key: val for key, val in zip(
                        DEMAND_NAMES,
                        (
                            client_data[1],
                            deadband(client_data[2] / 60),
                            deadband(client_data[3] / 60),
                            client_data[4] / -200)
                        )
                    }
            '''

            # Get desired assist mode
            mode = int(client_data[0])

            # Based on mode, run PID controllers on stick demands to get
            # modified demands

            if mode in (2, 3):  # Height hold or Hover
                print('altitude hold')

            if mode == 3:  # Hover
                print('hover')

            # Run mixer on modified demand to get motor vlaues
            m1, m2, m3, m4 = 0, 0, 0, 0 # mix(demands)

            # Set motor values
            m1_motor.setVelocity(m1)
            m2_motor.setVelocity(m2)
            m3_motor.setVelocity(m3)
            m4_motor.setVelocity(m4)

        # Track previous time and vehicle location
        time_prev = robot.getTime()
        pose_x_prev = pose['x']
        pose_y_prev = pose['y']
