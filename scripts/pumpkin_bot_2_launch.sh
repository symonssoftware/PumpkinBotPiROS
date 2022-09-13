#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pumpkin_py_pkg pi2_ui

# exit gracefully by returning a status 
exit 0
