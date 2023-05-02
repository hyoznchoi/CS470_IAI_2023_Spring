# Tutorial 3: Turtlebot3 

# ROS Installation
## Pre-requites for this tutorial
Please, install Ubuntu 20.04 and ROS Noetic by following the instrcutions on http://wiki.ros.org/noetic/Installation/Ubuntu 

## Installation of your project repository
~~~~bash
source /opt/ros/noetic/setup.sh
~~~~

Move to anyfolder you want to place the class code. Then, you can create the workspace,
~~~~bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
catkin_init_workspace
~~~~

Let's copy the the class repo, install dependencies, and build it!
~~~~bash
cd ..
catkin_make
~~~~
If you want catkin_build, you can use it instead of catkin_make.

Make sure that if ROS_MASTER_URI and ROS_ROOT, ETC are in your bashrc.
~~~~bash
export ROS_MASTER_URI=http://localhost:11311
~~~~


# Sub-tutorial Links
- [Tutorial ROS](https://github.com/pidipidi/cs470_IAI_2023_Spring/blob/main/tutorial_3/README_ROS.md)
- [Tutorial Turtlebot3](https://github.com/pidipidi/cs470_IAI_2023_Spring/blob/main/tutorial_3/README_TURTLEBOT.md)

# Error
If you get following error:
~~~~bash
CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
  Unable to find either executable 'empy' or Python module 'em'...  try
  installing the package 'python3-empy'
~~~~
Please use following instead of just 'catkin_make' 
~~~~bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
~~~~
