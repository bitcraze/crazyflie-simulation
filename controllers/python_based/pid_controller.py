# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  MIT Licence
#
#  Copyright (C) 2023 Bitcraze AB
#

"""
file: pid_controller.py

A simple PID controller for the Crazyflie
ported from pid_controller.c in the c-based controller of the Crazyflie
in Webots
"""

import numpy as np


class pid_position_controller():
    def __init__(self):
        self.past_x_error = 0.0
        self.past_y_error = 0.0
        self.past_alt_error = 0.0
        self.past_pitch_error = 0.0
        self.past_roll_error = 0.0
        self.past_yaw_error = 0.0
        self.altitude_integrator = 0.0
        self.last_time = 0.0
        self.pid_velocity_controller = pid_velocity_controller()

    def pid(self, dt, ctrl_mode, desired_state, actual_state, gains):
        '''
        PID controller for position control

        dt = time difference in seconds
        ctrl_mode = [ctrl_mode_xy, ctrl_mode_z, ctrl_mode_yaw]
        desired_state = [x, y, z, yaw] or [vx, vy, vz, yaw] (depending on ctrl_mode)
        actual_state = [x, y, z, vx, vy, vz, roll, pitch, yaw, yaw_rate]
        '''

        ctrl_mode_xy = ctrl_mode["ctrl_mode_xy"]
        ctrl_mode_z = ctrl_mode["ctrl_mode_z"]
        ctrl_mode_yaw = ctrl_mode["ctrl_mode_yaw"]

        actual_x = actual_state['x']
        actual_y = actual_state['y']
        actual_altitude = actual_state['z']
        actual_vx = actual_state['v_x']
        actual_vy = actual_state['v_y']
        actual_altitude_velocity = actual_state['v_z']
        actual_roll = actual_state['roll']
        actual_pitch = actual_state['pitch']
        actual_yaw = actual_state['yaw']
        actual_yaw_rate = actual_state['yaw_rate']

        # Horizontal position controller
        if ctrl_mode_xy == 0:
            desired_x = desired_state['x']
            desired_y = desired_state['y']

            x_error = desired_x - actual_x
            x_deriv = (x_error - self.past_x_error) / dt
            y_error = desired_y - actual_y
            y_deriv = (y_error - self.past_y_error) / dt
            desired_vx = gains["kp_pos_xy"] * np.clip(x_error, -1, 1) + gains["kd_pos_xy"] * x_deriv
            desired_vy = gains["kp_pos_xy"] * np.clip(y_error, -1, 1) - gains["kd_pos_xy"] * y_deriv
            self.past_x_error = x_error
            self.past_y_error = y_error
        elif ctrl_mode_xy == 1:
            desired_vx = desired_state['x']
            desired_vy = desired_state['y']

        # Vertical position controller
        if ctrl_mode_z == 0:
            desired_altitude = desired_state['z']
            alt_error = desired_altitude - actual_altitude
            alt_deriv = (alt_error - self.past_alt_error) / dt
            self.altitude_integrator += alt_error * dt
            desired_altitude_velocity = gains["kp_z"] * alt_error + gains["kd_z"] * alt_deriv + \
                gains["ki_z"] * np.clip(self.altitude_integrator, -2, 2) + 48
            print(desired_altitude_velocity, alt_error)
            self.past_alt_error = alt_error
        elif ctrl_mode_z == 1:
            desired_altitude_velocity = desired_state['z']

        # Yaw controller
        if ctrl_mode_yaw == 0:
            desired_yaw = desired_state['yaw']
            yaw_error = desired_yaw - actual_yaw
            yaw_deriv = (yaw_error - self.past_yaw_error) / dt
            desired_yaw_rate = gains["kp_yaw"] * yaw_error + gains["kd_yaw"] * yaw_deriv
            self.past_yaw_error = yaw_error
        elif ctrl_mode_yaw == 1:
            desired_yaw_rate = desired_state['yaw']

        # Now run the velocity controller with the desired velocities
        return self.pid_velocity_controller.pid(dt, desired_vx, desired_vy, desired_yaw_rate, desired_altitude_velocity,
                                                                actual_roll, actual_pitch, actual_yaw_rate,
                                                                actual_altitude_velocity, actual_vx, actual_vy,
                                                                gains)

class pid_velocity_controller():
    def __init__(self):
        self.past_vx_error = 0.0
        self.past_vy_error = 0.0
        self.past_alt_error = 0.0
        self.past_pitch_error = 0.0
        self.past_roll_error = 0.0
        self.altitude_integrator = 0.0
        self.last_time = 0.0

    def pid(self, dt, desired_vx, desired_vy, desired_yaw_rate, desired_altitude, actual_roll, actual_pitch, actual_yaw_rate,
            actual_altitude, actual_vx, actual_vy, gains):



        # Velocity PID control
        vx_error = desired_vx - actual_vx
        vx_deriv = (vx_error - self.past_vx_error) / dt
        vy_error = desired_vy - actual_vy
        vy_deriv = (vy_error - self.past_vy_error) / dt
        desired_pitch = gains["kp_vel_xy"] * np.clip(vx_error, -1, 1) + gains["kd_vel_xy"] * vx_deriv
        desired_roll = -gains["kp_vel_xy"] * np.clip(vy_error, -1, 1) - gains["kd_vel_xy"] * vy_deriv
        self.past_vx_error = vx_error
        self.past_vy_error = vy_error

        # Altitude velocity PID control
        alt_error = desired_altitude - actual_altitude
        alt_deriv = (alt_error - self.past_alt_error) / dt
        self.altitude_integrator += alt_error * dt
        alt_command = gains["kp_vel_z"] * alt_error + gains["kd_vel_z"] * alt_deriv + \
            gains["ki_vel_z"] * np.clip(self.altitude_integrator, -2, 2) + 30
        self.past_alt_error = alt_error

        # Attitude PID control
        pitch_error = desired_pitch - actual_pitch
        pitch_deriv = (pitch_error - self.past_pitch_error) / dt
        roll_error = desired_roll - actual_roll
        roll_deriv = (roll_error - self.past_roll_error) / dt
        roll_command = gains["kp_att_rp"] * np.clip(roll_error, -1, 1) + gains["kd_att_rp"] * roll_deriv
        pitch_command = -gains["kp_att_rp"] * np.clip(pitch_error, -1, 1) - gains["kd_att_rp"] * pitch_deriv
        self.past_pitch_error = pitch_error
        self.past_roll_error = roll_error

        # Yaw rate PID control
        yaw_rate_error = desired_yaw_rate - actual_yaw_rate
        yaw_command = gains["kp_att_yaw"] * np.clip(yaw_rate_error, -1, 1)

        # Motor mixing
        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]