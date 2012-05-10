#!/bin/sh

#this script will dump all the parameters for the software

rosrun dynamic_reconfigure dynparam dump /Control Control.yaml

rosrun dynamic_reconfigure dynparam dump /Edge_Detection Edge_Detection.yaml

rosrun dynamic_reconfigure dynparam dump /Pot_Nav Pot_Nav.yaml

rosrun dynamic_reconfigure dynparam dump /ColorStat ColorStat.yaml

rosrun dynamic_reconfigure dynparam dump /Homography Homography.yaml




