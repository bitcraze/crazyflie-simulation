# crazyflie_simulation

Hi! Welkom to the crazyflie simulation repo. This repo is still at the early stages and still in development so just ask if you are looking for anything in particular.

Currently it contains low resolution meshes based on the design of the actual [crazyflie 2.1](https://www.bitcraze.io/products/crazyflie-2-1/) but heavily simplified to contain as little vertices as possible, so it can be used to simulate a lot of them!

## Content

Currently contains:
* Meshes
    * Blender files
    * STL files
* [Webots](https://cyberbotics.com/) simulation files
    * Currently only attitude control with fixed height

So the mesh files looks a bit like this:


<img src="/meshes/blender_files/cf2_render.png" width="400" />

And the webots simulation looks like this:
![webots](crazyflie_webots.gif)



Near future plans:
* [Gazebo ignition](https://ignitionrobotics.org/) urdf files

Wish list (?):
* SITL and or HITL
* Simple physics model
* Deck models
* Intergration with Crazyflie-lib
* ?? (let us know in the issue tracker for any feature requests)

## Measurements Crazyflie Assembly

The location of the parts

* Propeller origin from center (0.031, 0.031, 0.022) m
    * Mirrored in the x and y axis
* Motor mount + motor mounts (0.031, 0.031, 0.014) m
    * Mirrored in the x and y axis
* Body origin height from center ( 0, 0, 0.015) m
* Battery holder height ( 0, 0,0.025) m
* Battery height (0,0,0.02) m
* Pin headers (0, 0.011, 0.014) m
    * Mirrored in the y axis
