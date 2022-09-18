#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pumpkin_bot_bringup pumpkin_bot_launch.py

# exit gracefully by returning a status 
exit 0
