#!/bin/bash -xve

#install packages
sudo apt-get install ros-indigo-cv-bridge
sudo apt-get install ros-indigo-perception-pcl
sudo apt-get install ros-indigo-turtlebot-gazebo

#sync and make
rsync -av ./ ~/catkin_ws/src/ros_tutorial_pkgs_20170907
cd ~/catkin_ws
catkin_make
