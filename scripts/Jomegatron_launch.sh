#!/bin/bash
#this script is used to run the Jomegatron.launch file to set this up to run on startup on the robot add the following comand to the startup aplications list in the systems > Propertys > Startup Aplications menu.
# gnome-terminal -x /home/robot/Code/Jomegatron_IGVC/scripts/Jomegatron_launch.sh
source /opt/ros/electric/setup.sh
export ROS_PACKAGE_PATH=/home/robot/Code:/opt/ros/electric/stacks
source /home/robot/.bashrc
roslaunch MST_Control Jomegatron.launch


