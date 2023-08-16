# crazyflie_simulation

Hi! Welkom to the crazyflie simulation repo. This repo is still at the early stages and still in development so just ask if you are looking for anything in particular.

Currently it contains low resolution meshes based on the design of the actual [crazyflie 2.1](https://www.bitcraze.io/products/crazyflie-2-1/) but heavily simplified to contain as little vertices as possible, so it can be used to simulate a lot of them.


## Installation

see the  [installation instructions](/docs/index.md) in the Github docs folder.

## Official documentation

ToDo

## Contribute

Go to the [contribute page](https://www.bitcraze.io/development/contribute/) on our website to learn more.



## External socket control
TO BE MOVED TO REPO DOC

There is also an example that shows controlling the crazyflie with an socket connection.

Change the controller to `crazyflie_controller_py_socket` and start the simulation. Then in a seperate terminal outside of the webots simulator run:

    python3 webots/controllers/crazyflie_controller_py_socket/socket_control.py

You can use the arrow keys and q e w s to control the drone in velocity mode.

