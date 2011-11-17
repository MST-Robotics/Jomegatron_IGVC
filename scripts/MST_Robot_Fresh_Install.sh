#!/bin/sh
#this script is currently incompleat
#This script will install all of the code that the robotics team uses. When done all of the code should run the same as it does on the robot. It asumes a current Ubuntu 11.04 instalation 

#First we install ROS
#setting up sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu natty main" > /etc/apt/sources.list.d/ros-latest.list'
#set up keys
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
#update the pagkage lists
sudo apt-get update
#desktop full install
sudo apt-get install ros-electric-desktop-full
#now setup the enviroment in .bashrc so you can use ros comands
echo "source /opt/ros/electric/setup.bash" >> ~/.bashrc
. ~/.bashrc

