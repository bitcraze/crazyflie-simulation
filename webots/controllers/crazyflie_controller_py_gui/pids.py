'''
PID controllers for simulated Crazyflie

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


def mkdemands(t, r, p, y):

    return {'throttle': t, 'roll': r, 'pitch': p, 'yaw': y}


class _AxisPidController:

    def __init__(self, sgn):

        self.sgn = sgn
        self.past_error = 0

    def run(self, kp, kd, dt, demand, actual):

        error = demand - actual
        deriv = (error - self.past_error) / dt
        demand = self.sgn * (kp * np.clip(error, -1, +1) + kd * deriv)
        self.past_error = error

        return demand


class _CyclicPidController:

    def __init__(self, kp, kd, rsign, psign):

        self.kp = kp
        self.kd = kd

        self.roll_controller = _AxisPidController(rsign)
        self.pitch_controller = _AxisPidController(psign)

        self.rdsign = rsign

    def run_axes(self, dt, demands, roll_actual, pitch_actual):

        roll_demand = self.run_axis(
                self.roll_controller,
                dt,
                self.rdsign * demands['roll'],
                roll_actual)

        pitch_demand = self.run_axis(
                self.pitch_controller, dt, demands['pitch'], pitch_actual)

        return roll_demand, pitch_demand

    def run_axis(self, controller, dt, demand, velocity):

        return controller.run(self.kp, self.kd, dt, demand, velocity)


class AltitudePidController:

    def __init__(
            self,
            kp=1.0,
            ki=0.5,
            kd=0.5,
            windup_max=2,
            throttle_scale=0.1,
            minimal_altitude=0.2):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.windup_max = windup_max
        self.throttle_scale = throttle_scale
        self.minimal_altitude = minimal_altitude

        self.past_alt_error = 0
        self.altitude_integrator = 0
        self.desired_altitude = 0

    def run(self, dt, demands, state):
        '''
        Returns modified demands
        '''

        altitude = state['z']

        height_diff_desired = demands['throttle'] * (
                self.throttle_scale
                if altitude > self.minimal_altitude
                else 1)

        self.desired_altitude += height_diff_desired * dt

        # Altitude PID control
        alt_error = self.desired_altitude - altitude
        alt_deriv = (alt_error - self.past_alt_error) / dt
        self.altitude_integrator += alt_error * dt

        alt_demand = (self.kp * alt_error +
                      self.kd * alt_deriv +
                      self.ki * np.clip(
                          self.altitude_integrator,
                          -self.windup_max,
                          +self.windup_max))

        self.past_alt_error = alt_error

        return mkdemands(
                alt_demand, demands['roll'], demands['pitch'], demands['yaw'])


class PositionPidController(_CyclicPidController):

    def __init__(self, kp=2, kd=0.5):

        _CyclicPidController.__init__(self, kp, kd, -1, +1)

    def run(self, dt, demands, state):

        # Rotate world-coordinate velocities into vehicle coordinates
        cpsi = np.cos(state['psi'])
        spsi = np.sin(state['psi'])
        vx = state['dx'] * cpsi + state['dy'] * spsi
        vy = - state['dx'] * spsi + state['dy'] * cpsi

        roll_demand, pitch_demand = self.run_axes(dt, demands, vy, vx)

        return mkdemands(
                demands['throttle'], roll_demand, pitch_demand, demands['yaw'])


class AnglePidController(_CyclicPidController):

    def __init__(self, kp_cyclic=0.5, kd_cyclic=0.1, kp_yaw=.25):

        _CyclicPidController.__init__(self, kp_cyclic, kd_cyclic, +1, -1)

        self.kp_yaw = kp_yaw

    def run(self, dt, demands, state):

        roll_demand, pitch_demand = self.run_axes(
                dt, demands, state['phi'], state['theta'])

        yaw_rate_error = demands['yaw'] - state['dpsi']
        yaw_demand = self.kp_yaw * np.clip(yaw_rate_error, -1, +1)

        return mkdemands(
                demands['throttle'], roll_demand, pitch_demand, yaw_demand)
