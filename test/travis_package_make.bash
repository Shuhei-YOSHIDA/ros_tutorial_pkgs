#!/bin/bash -xve

#sync and make
rsync -av ./ ~/catkin_ws/src/ros_tutorial_pkgs_20170907
cd ~/catkin_ws
catkin_make
