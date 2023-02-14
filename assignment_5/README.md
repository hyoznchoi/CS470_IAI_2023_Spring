# Installation Guide
Before followings, you should have Ubuntu 20.04 and ROS Noetic.

## Install dependencies for TurtleBot3 Packages
~~~~bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
sudo apt-get install python3-tk python-numpy
sudo apt install ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3
~~~~

## Install Simulation Package

Before installing the simulation and compile the necessary packages, let's move to your assignment5 folder
~~~~bash
cd your assignment5 folder
~~~~

Then, clone the TurtleBot3 Simulation Package
~~~~bash
cd src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..
catkin_make
~~~~

Note that you have source the environment setup for ROS before running catkin_make. For example,
~~~~bash
source /opt/ros/noetic/setup.bash
~~~~

# PROBLEM 1: Run ASTAR on gridworld

~~~~bash
source ./robot.sh
roscd py_astar_planner/src/py_astar_planner
python3 astar.py
~~~~


# PROBLEM 2: Run Turtulebot3 Simulation

We first load necessary environment variables pre-defined in the following bash script. Note that you have to run the following script when you often any new terminal from now!
~~~~bash
source ./robot.sh
~~~~

## Terminal 1
Let's run the TurtleBot3 World
~~~~bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
~~~~

## Terminal 2
Let's run the ASTAR planner with RViZ
~~~~bash
roslaunch py_astar_planner turtlebot3_navigation.launch
~~~~

## Align the map with LDS sensor input

1. Click the "2D Pose Estimate button" in the RViz menu.
2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
3. Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.

## 2D Navigation with ASTAR planner

1. Click the "2D Nav Goal button" in the RViz menu.
2. Click on the map to set the destination of the robot and drag the pink arrow toward the direction where the robot will be facing.
3. As soon as the goal is set, the path will be generated TurtleBot3 will start moving to the destination immediately.


## Q&A

1. Why "NO PATH!" message after completion?
This py_astar_planner package iterlatively run the ASTAR algorithm. So, after reaching the goal, it still tries to find a path though it does not require path.

2. 
