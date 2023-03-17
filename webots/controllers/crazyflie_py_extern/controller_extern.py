from __future__ import annotations





import os
import sys

import numpy as np

os.environ['WEBOTS_HOME'] = 'C:\Program Files\Webots'
os.environ['PYTHONPATH'] = os.path.expandvars('${WEBOTS_HOME}\lib\controller\python:$PYTHONPATH')
os.environ['PYTHONIOENCODING'] = 'UTF-8'

sys.path.append('C:\Program Files\Webots\lib\controller\python')
from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

sys.path.append('../../../controllers/')
from pid_controller import init_pid_attitude_fixed_height_controller, pid_velocity_fixed_height_controller
from pid_controller import MotorPower_t, ActualState_t, GainsPID_t, DesiredState_t

from PIL import Image
import io
from math import cos, sin

class CrazyflieWebotsDriver:
    def __init__(self, name):
        #os.environ['WEBOTS_CONTROLLER_URL'] = name
        self.name = name
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)
        self.keyboard = Keyboard()
        self.keyboard.enable(self.time_step)
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.time_step)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.time_step)

        self.actualState = ActualState_t()
        self.desiredState = DesiredState_t()
        self.motorPower = MotorPower_t()
        self.pastXGlobal = 0
        self.pastYGlobal = 0
        self.gainsPID = GainsPID_t()
        self.gainsPID.kp_att_y = 1
        self.gainsPID.kd_att_y = 0.5
        self.gainsPID.kp_att_rp =0.5
        self.gainsPID.kd_att_rp = 0.1
        self.gainsPID.kp_vel_xy = 2
        self.gainsPID.kd_vel_xy = 0.5
        self.gainsPID.kp_z = 10
        self.gainsPID.ki_z = 49
        self.gainsPID.kd_z = 0
        init_pid_attitude_fixed_height_controller()

        self.past_time = self.robot.getTime()-0.1



    def step(self):


        dt = self.robot.getTime() - self.past_time



        ## Get measurements
        self.actualState.roll = self.imu.getRollPitchYaw()[0]
        self.actualState.pitch = self.imu.getRollPitchYaw()[1]
        self.actualState.yaw_rate = self.gyro.getValues()[2];
        self.actualState.altitude = self.gps.getValues()[2];
        xGlobal = self.gps.getValues()[0]
        vxGlobal = (xGlobal - self.pastXGlobal)/dt
        yGlobal = self.gps.getValues()[1]
        vyGlobal = (yGlobal - self.pastYGlobal)/dt

        ## Get body fixed velocities
        actualYaw = self.imu.getRollPitchYaw()[2];
        cosyaw = cos(actualYaw)
        sinyaw = sin(actualYaw)
        self.actualState.vx = vxGlobal * cosyaw + vyGlobal * sinyaw
        self.actualState.vy = - vxGlobal * sinyaw + vyGlobal * cosyaw


        ## Initialize values
        self.desiredState.roll = 0
        self.desiredState.pitch = 0
        self.desiredState.vx = 0
        self.desiredState.vy = 0
        self.desiredState.yaw_rate = 0
        self.desiredState.altitude = 1.0

        forwardDesired = 0
        sidewaysDesired = 0
        yawDesired = 0


        init_pid_attitude_fixed_height_controller();

        key = self.keyboard.getKey()
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

            key = self.keyboard.getKey()

        ## Example how to get sensor data
        ##range_front_value = range_front.getValue();
        #cameraData = self.camera.getImage()
        #buf = io.BytesIO(cameraData)

        #img = Image.open(buf)
        #img.show()
        self.desiredState.yaw_rate = yawDesired;

        ## PID velocity controller with fixed height
        self.desiredState.vy = sidewaysDesired;
        self.desiredState.vx = forwardDesired;
        pid_velocity_fixed_height_controller(self.actualState, self.desiredState,
        self.gainsPID, dt, self.motorPower);

        self.m1_motor.setVelocity(-self.motorPower.m1)
        self.m2_motor.setVelocity(self.motorPower.m2)
        self.m3_motor.setVelocity(-self.motorPower.m3)
        self.m4_motor.setVelocity(self.motorPower.m4)

        print(self.motorPower.m1, self.motorPower.m2, self.motorPower.m3, self.motorPower.m4)

        self.past_time = self.robot.getTime()
        self.pastXGlobal = xGlobal
        self.pastYGlobal = yGlobal


if __name__ == '__main__':


    webots_driver = CrazyflieWebotsDriver('name')
    time_step = int(webots_driver.robot.getBasicTimeStep())
    webots_driver.step()

    # Keep looping until the simulation is over
    # This step goes before the supervisor step
    while webots_driver.robot.step(time_step) != -1:
        webots_driver.step()
