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

import math
import socket
import threading
import time
import struct
import sys

from controller import Robot

import cffirmware

sys.path.append('../../../controllers/python_based/')
from pid_controller import pid_velocity_fixed_height_controller

HOST = '127.0.0.1'
PORT = 5000

PID_TICK = 100
MOTOR_SCALEDOWN = 1000
THROTTLE_SCALEDOWN = 100
CYCLIC_SCALEDOWN = 60
YAW_SCALEUP = 5


def mix(thrust, roll, pitch, yaw):
    '''Mixer converts demands to motor values'''

    m1 = thrust - roll + pitch - yaw
    m2 = thrust - roll - pitch + yaw
    m3 = thrust + roll - pitch - yaw
    m4 = thrust + roll + pitch + yaw

    return -m1, +m2, -m3, +m4


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

        time.sleep(0.001)


def mkmotor(name, spin):
    '''Helper'''

    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    motor.setVelocity(spin)

    return motor


def mksensor(robot, name, timestep):
    '''Helper'''

    sensor = robot.getDevice(name)
    sensor.enable(timestep)

    return sensor


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
    m1_motor = mkmotor('m1_motor', -1)
    m2_motor = mkmotor('m2_motor', +1)
    m3_motor = mkmotor('m3_motor', -1)
    m4_motor = mkmotor('m4_motor', +1)

    # Initialize sensors
    imu = mksensor(robot, 'inertial_unit', timestep)
    loco = mksensor(robot, 'position', timestep)
    gyro = mksensor(robot, 'gyro', timestep)

    # Init firmware PID controller
    cffirmware.controllerPidInit()

    # Initialize previous position, time
    loco_x_prev, loco_y_prev, loco_z_prev = 0, 0, 0
    time_prev = robot.getTime()

    # In altitude-hold or hover mode, we'll maintain this altitude
    altitude_target = 1

    # Main loop:
    while robot.step(timestep) != -1:

        # Read sensors
        phi, theta, psi = imu.getRollPitchYaw()
        dphi_dt, dtheta_dt, dpsi_dt = gyro.getValues()

        # First-difference current and previous times to get DeltaT
        robot_time = robot.getTime()
        dt = robot_time - time_prev
        time_prev = robot_time

        # First-difference position to get inertial-frame velocity
        loco_x, loco_y, loco_z = loco.getValues()
        loco_vx = (loco_x - loco_x_prev) / dt
        loco_vy = (loco_y - loco_y_prev) / dt
        loco_vz = (loco_z - loco_z_prev) / dt
        loco_x_prev = loco_x
        loco_y_prev = loco_y
        loco_z_prev = loco_z

        # Set up state vector
        state = cffirmware.state_t()
        state.attitude.roll = math.degrees(phi)
        state.attitude.pitch = -math.degrees(theta)  # note negation
        state.attitude.yaw = math.degrees(psi)

        # Put location into state
        state.position.z = loco_z
        state.velocity.z = loco_vz
        state.velocity.x = loco_vx
        state.velocity.y = loco_vy

        # Put rotational velocity into state
        sensors = cffirmware.sensorData_t()
        sensors.gyro.x = math.degrees(dphi_dt)
        sensors.gyro.y = math.degrees(dtheta_dt)
        sensors.gyro.z = math.degrees(dpsi_dt)

        # Get stick demands from GUI client

        # Get desired assist mode: 1 = none; 2 = altitude-hold; 3 = hover
        # XXX We currently ignore this and stay in mode 3
        mode = int(client_data[0])

        # Use stick demands from GUI client for setpoints
        altitude_target += client_data[1] / THROTTLE_SCALEDOWN
        sideways_demand = -deadband(client_data[2])  # note negation
        forward_demand = deadband(client_data[3])
        yaw_demand = -client_data[4] * YAW_SCALEUP  # note negation

        # Fill in setpoints
        setpoint = cffirmware.setpoint_t()
        setpoint.mode.z = cffirmware.modeAbs
        setpoint.position.z = altitude_target
        setpoint.mode.yaw = cffirmware.modeVelocity
        setpoint.attitudeRate.yaw = math.degrees(psi) + yaw_demand
        setpoint.mode.x = cffirmware.modeVelocity
        setpoint.mode.y = cffirmware.modeVelocity
        setpoint.velocity.x = forward_demand
        setpoint.velocity.y = sideways_demand
        setpoint.velocity_body = True

        # Make firmware PID bindings
        control = cffirmware.control_t()
        cffirmware.controllerPid(control, setpoint, sensors, state, PID_TICK)

        # Run mixer on modified demands to get motor values
        m1, m2, m3, m4 = mix(
                control.thrust,
                math.radians(control.roll),
                math.radians(control.pitch),
                math.radians(control.yaw))

        # Set motor values
        m1_motor.setVelocity(m1 / MOTOR_SCALEDOWN)
        m2_motor.setVelocity(m2 / MOTOR_SCALEDOWN)
        m3_motor.setVelocity(m3 / MOTOR_SCALEDOWN)
        m4_motor.setVelocity(m4 / MOTOR_SCALEDOWN)
