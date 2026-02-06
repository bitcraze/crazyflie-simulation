#!/usr/bin/env bash

# This script builds the controller in the current folder
# Environment variable needs to be set which is otherwise set within Webots

export WEBOTS_HOME=/usr/local/webots
rm -rf build crazyflie_controller_c
make

