#!/bin/bash

sudo killall rosmaster
sudo killall gzserver
sudo killall gzclient

sudo apt update && sudo apt upgrade
sudo apt install ros-melodic-uol-cmp3103m ros-melodic-desktop
sudo apt install ros-melodic-uol-cmp3103m

sudo apt update && sudo apt install ros-melodic-ros-book-line-follower
source /opt/ros/melodic/setup.bash

roslaunch uol_turtlebot_simulator turtlebot-rviz.launch
