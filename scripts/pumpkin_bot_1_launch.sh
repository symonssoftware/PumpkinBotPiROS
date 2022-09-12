#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pumpkin_bot_bringup pumpkin_bot_launch.py
ros2 run pumpkin_py_pkg robot_ui

# exit gracefully by returning a status 
exit 0
