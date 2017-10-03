#!/bin/bash -xve

#install packages
sudo apt-get install ros-indigo-cv-bridge
sudo apt-get install ros-indigo-perception-pcl
sudo apt-get install ros-indigo-turtlebot-gazebo
sudo apt-get install ros-indigo-tf2-geometry-msgs

#sync and make
rsync -av ./ ~/catkin_ws/src/ros_tutorial_pkgs
cd ~/catkin_ws
catkin_make
