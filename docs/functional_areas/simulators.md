---
title: Simulators
page_id: simulators
sort_order: 2
---

The folder `/simulator_files/`contain files that enables the simulators to fly a crazyflie in a particular simulator.

## Webots

In the folder `/simulator_files/webots` there are the following folders:

* `/protos/`
* `/worlds/` 
* `/controllers/` 

See each section for the explanation

### protos

`/protos/`This contains the source file of the [PROTO](https://cyberbotics.com/doc/reference/proto) of the crazyflie in Webots. A PROTO node (fileformat is *.proto) is a description framework for objects in that simulator. This contains the Crazyflie basics, like propperor actuation and IMU, as well as a ground truth position, camera and multiranger.

### worlds
`/worlds/` This contains world files in the format *.wbt and it an file that describes the webots world with all objects in it with certain initalization variables for the physics. 

* `crazyflie_world.wbt` only contains a floor with one Crazyflie. 
* `crazyflie_apartement.jpg` contains a Crazyflie initialized in an apartement with interactable walls and objects. 

### controllers

`/controllers/` contain all the controller files of the webots simulator for the Crazyflie, which acts likes wrappers around the files of `/controllers_shared/`, such that only webots specific functions exist in that wrapper.

  * crazyflie_controller_c: wraps around `/controllers_shared/c_based` with simple keybased velocity reference input.
  * crazyflie_controller_py: wraps around `/controllers_shared/python_based` with simple keybased velocity reference input.
  * crazyflie_controller_py_firmware_pid: wraps around crazyflie-firmware pythonbindings. Check out the [controller](/docs/functional_areas/controllers.md) page.
  * crazyflie_controller_py: wraps around `/controllers_shared/python_based` with an wall following state machine giving the reference velocity based on multiranger data.

## Gazebo

This folder contains crazyflie controllers, source and world files for Gazebo (not gazebo classic). This simulator is still underdeveloped so the controllers can only rotate the propellers for now. 

* `crazyflie` This folder contains the .sdf file that describes the crazyflie. This relies on the collada files in `\meshes`.
* `worlds` This contains only an empty world with a floor and a Crazyflie model.
* `controllers` This contains the source files and cmake file for the Crazyflie to rotate the propellers based on an ign transport message.