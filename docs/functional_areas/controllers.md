---
title: Shared Controllers
page_id: controllers
sort_order: 3
---

This page explains about the shared control files of the different simulators to fly a simulated Crazyflie.

## Simple PID controller

A very simple height fixed and horizontal velocity based was build for this simulator that is written in both c and python. These can be found in the folder `/shared_controllers` and the idea is that they can be used interchangably between de Gazebo and Webots simulation. They are therefore meant to be stand alone.

In webots, change controller to either `crazyflie_controller_c` or `crazyflie_controller_py` to try out a simple controller in different languages.

## Firmware python bindings
As of this [Pull request in the Crazyflie firmware repo](https://github.com/bitcraze/crazyflie-firmware/pull/1021) it is possible to use the python bindings of the controllers of the actual crazyflie controller directly in webots.

> The python bindings only works in Webots for now.

[Clone the crazyflie-firmware](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#cloning) in another folder, then make the python bindings [via these instructions](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings).

Change the controller in the crazyflie robot model in webots to crazyflie_controller_py_firmware_pid, and adjust the following line to point to your crazyflie-firmware repo:

    sys.path.append('../../../../../C/crazyflie-firmware')

Press play with the simulator and use your keyboard to  control it.

## External socket control

There is also an example that shows controlling the crazyflie with an socket connection.

Change the controller to `crazyflie_controller_py_socket` and start the simulation. Then in a seperate terminal outside of the webots simulator run:

    python3 webots/controllers/crazyflie_controller_py_socket/socket_control.py

You can use the arrow keys and q e w s to control the drone in velocity mode.
