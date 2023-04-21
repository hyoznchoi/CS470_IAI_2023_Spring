# Setup

## Requirements
This repository requires Ubuntu 20.04 and ROS Noetic (Desktop-Full version is recommended to install). 

You can install the ROS Noetic following the instructions on http://wiki.ros.org/noetic/Installation/Ubuntu.

### Docker Option
Install docker for ubuntu
~~~~bash
sudo apt install docker-ce docker-ce-cli containerd.io
~~~~
If you are on MacOS, please visit [docker homepage](https://docs.docker.com/desktop/install/mac-install/) for the more information.


Then, pull ROS Noetic image for docker.
~~~bash
docker pull osrf/ros:noetic-desktop-full
~~~

To enable the visualization for docker, 
~~~bash
sudo xhost +local:docker
docker run -it --privileged -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --name CS470 osrf/ros:noetic-desktop-full
~~~
You may use any other name that you want instead of CS470.

To open a new terminal,
~~~bash
docker exec -it CS470 /bin/bash
~~~

Before working on the simulated Turtlebot, first please follow this tutorial [ROS Tutorial](README_ROS.md)

# Setting up this repository 
This is a manual to simulate and control our mobile robots: 
[Robotis e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
~~~~bash
sudo apt update
sudo apt upgrade
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
~~~~

Install the following dependencies:
~~~bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
~~~

~~~bash
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
~~~

Install Simulation Package
~~~bash
cd ~
mkdir catkin_ws/ && mkdir catkin_ws/src/
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
~~~


## Check the installation

Gazebo: A simulator for this tutorial
~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
~~~

Teleoperation

This launches the node that convert keyboard input into velocity command.
~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~~~

RViz: a visualization tool for ROS.
~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
~~~

# SLAM

Turn off all the previous processes (Gazebo, RViz, Teleoperation)

Please open 4 terminal windows, then

On Terminal 1: Restart the world

~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
~~~

On Terminal 2: SLAM node 
~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
~~~

On Terminal 3: Teleoperation
~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~~~

On Terminal 4: Save map
~~~bash
rosrun map_server map_saver -f ~/map
~~~


# Navigation

Turn off all the previous processes (Gazebo, RViz, Teleoperation, etc)

Please open 3 terminal windows, then

On Terminal 1: Restart the world

~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
~~~

On Terminal 2: SLAM node 
~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
~~~

On Terminal 3: Teleoperation
~~~bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~~~

## In case alignment is not automatically done

1. Click the "2D Pose Estimate button" in the RViz menu.
2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
3. Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.


## Quiz

1. Click the "2D Nav Goal button" in the RViz menu.
2. Click on the map to set the destination of the robot and drag the pink arrow toward the direction where the robot will be facing.
3. As soon as the goal is set, the path will be generated TurtleBot3 will start moving to the destination immediately.

Please capture the RViz window of showing navigation goal, generated path and your student ID/number.
