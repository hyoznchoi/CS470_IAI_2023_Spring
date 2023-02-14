#!/bin/sh

source /opt/ros/noetic/setup.bash

export ROBOT=sim
#export DISPLAY=:0.0
export ROS_MASTER_URI=http://localhost:11311
export TURTLEBOT3_MODEL=waffle


source ./devel/setup.bash


