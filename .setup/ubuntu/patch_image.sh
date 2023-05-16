#!/usr/bin/env bash

# TODO this is only to patch the old image. Should be replaced later
sudo apt-get update -y
sudo apt-get install git -y
pip3 install catkin_tools

# Setup workspace
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin config --merge-devel