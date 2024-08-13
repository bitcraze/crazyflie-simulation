---
title: Gazebo Velocity Control
page_id: gazebo_velocity_control
sort_order: 3
---

> *Just mind that this model does not fly great yet and is considered experimental*

1- First install gazebo harmonic: https://ignitionrobotics.org/docs/harmonic/install

2- Clone this repo: 

    git clone https://github.com/bitcraze/crazyflie-simulation.git

3- Put this repo in your ~/.bashrc and source it in your terminal

    export GZ_SIM_RESOURCE_PATH="{PATH ON YOUR COMPUTER}/crazyflie-simulation/simulator_files/gazebo/"

4- Try out the crazyflie world with: 

    gz sim -r worlds/crazyflie_world.sdf

5- Go to the right side of the gazebo gui and find the teleop plugin. Subscribe to `/crazyflie/gazebo/command/twist`.

6- Make it take off with the top arrow, and press top to make it hover.
