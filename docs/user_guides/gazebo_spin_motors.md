---
title: Gazebo Spin Motors
page_id: gazebo_spin_motors
sort_order: 3
---

> *Just mind that this model does not fly properly yet and is considered experimental*

1- First install gazebo harmonic: https://ignitionrobotics.org/docs/harmonic/install

2- Clone this repo:
    git clone https://github.com/bitcraze/crazyflie_simulation.git

3- Put this repo in your ~/.bashrc and source it in your terminal

    export GZ_SIM_RESOURCE_PATH="{PATH ON YOUR COMPUTER}/crazyflie-simulation/simulator_files/gazebo/"

*Note that indeed this is still IGN_GAZEBO_RESOURCE_PATH, eventhough gazebo dropped the name IGN so this should be updated at one point*

4- Try out the crazyflie world with:
    gz sim worlds/crazyflie_world.sdf

5- Spin motors

    gz topic -t /crazyflie/gazebo/command/motor_speed --msgtype ignition.msgs.Actuators -p 'velocity:[250,250,250,250]'


gz topic -t "/crazyflie/gazebo/command/twist" -m gz.msgs.Twist -p "linear: {x:0 y: 0 z: 0.0} angular {z: 0}"