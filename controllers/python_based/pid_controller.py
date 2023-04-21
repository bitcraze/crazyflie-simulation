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
# pid_controller.py
# A simple PID controller for the Crazyflie
#
# Credit: Shushuai Li (EPFL) created the port
#        


import numpy as np

class pid_velocity_fixed_height_controller():
    def __init__(self):
        self.pastVxError = 0
        self.pastVyError = 0
        self.pastAltError = 0
        self.pastPitchError = 0
        self.pastRollError = 0
        self.last_time = 0.0

    def pid(self, dt, desired_vx, desired_vy, desired_yaw_rate, desired_alt, actual_roll, actual_pitch, actual_yaw_rate,
            actual_alt, actual_vx, actual_vy):
        # Velocity PID control (converted from Crazyflie c code)
        gains = {"kp_att_y": 1, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
                "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 10, "ki_z": 50, "kd_z": 5}

        # Velocity PID control
        vxError = desired_vx - actual_vx
        vxDeriv = (vxError - self.pastVxError) / dt
        vyError = desired_vy - actual_vy
        vyDeriv = (vyError - self.pastVyError) / dt
        desired_pitch = gains["kp_vel_xy"] * np.clip(vxError, -1, 1) + gains["kd_vel_xy"] * vxDeriv
        desired_roll = -gains["kp_vel_xy"] * np.clip(vyError, -1, 1) - gains["kd_vel_xy"] * vyDeriv
        self.pastVxError = vxError
        self.pastVyError = vyError

        # Altitude PID control
        altError = desired_alt - actual_alt
        altDeriv = (altError - self.pastAltError) / dt
        altCommand = gains["kp_z"] * np.clip(altError, -1, 1) + gains["kd_z"] * altDeriv + gains["ki_z"]
        self.pastAltError = altError

        # Attitude PID control
        pitchError = desired_pitch - actual_pitch
        pitchDeriv = (pitchError - self.pastPitchError) / dt
        rollError = desired_roll - actual_roll
        rollDeriv = (rollError - self.pastRollError) / dt
        yawRateError = desired_yaw_rate - actual_yaw_rate
        rollCommand = gains["kp_att_rp"] * np.clip(rollError, -1, 1) + gains["kd_att_rp"] * rollDeriv
        pitchCommand = -gains["kp_att_rp"] * np.clip(pitchError, -1, 1) - gains["kd_att_rp"] * pitchDeriv
        yawCommand = gains["kp_att_y"] * np.clip(yawRateError, -1, 1)
        self.pastPitchError = pitchError
        self.pastRollError = rollError

        # Motor mixing
        m1 =  altCommand - rollCommand + pitchCommand + yawCommand
        m2 =  altCommand - rollCommand - pitchCommand - yawCommand
        m3 =  altCommand + rollCommand - pitchCommand + yawCommand
        m4 =  altCommand + rollCommand + pitchCommand - yawCommand

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]