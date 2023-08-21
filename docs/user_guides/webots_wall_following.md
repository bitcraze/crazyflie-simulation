---
title: Webots Wall following
page_id: webots_wall_following
sort_order: 2
---

>This example runs in Webots 2023b.

Run the following world

    webots webots/worlds/crazyflie_appartement.wbt

Or open up the world by the menu item `file`>`open world...`.


Go to the `webots/controller/crazyflie_controller_py_wallfollowing/` and look into crazyflie_controller_py_wallfollowing.py. 

Make sure this line is pointing to the cflib's wallfollowing script (make sure to update the [crazyflie-lib-python repository](https://github.com/bitcraze/crazyflie-lib-python))

    sys.path.append('../../../../../python/crazyflie-lib-python/examples/multiranger/wall_following')


You can toggle wallfollowing with with the 'a' key on your keyboard. 

You will now see the Crazyflie following the wall in the apartement by means of the simulated multiranger.
