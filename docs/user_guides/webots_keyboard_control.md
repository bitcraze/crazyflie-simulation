---
title: Webots keyboard control
page_id: webots_keyboard_control
sort_order: 1
---

>This example runs in Webots 2025a

First you have to build the controller. From the command line:

    WEBOTS_HOME=/usr/local/webots make

Or from within Webots: Open the text file `crazyflie_controller.c` and the run the menu item `Build`>`Build` (F7).


Then run the following:

    webots webots/worlds/crazyflie_world.wbt


Or open up the world by the menu item `file`>`open world...`.


You'll see a crazyflie take off. You can now click the 3d world to make sure it is active, and use your keyboard to control it.
The script in the console should provide further instructions. 


If you want something more fun, try out the [webots wall following example](/docs/user_guides/webots_wall_following.md)

