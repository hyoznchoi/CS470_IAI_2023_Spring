#!/usr/bin/env python
import rospy

# Initialization of ROS Node
rospy.init_node("parameter_client")
rospy.loginfo("Tutorial-parameter Start!")

# Set a ROS parameter
rospy.set_param("/tutorial3", "test")
rospy.loginfo("Set a ROS parameter: test => /tutorial3")

# Get a ROS parameter
resp = rospy.get_param("/tutorial3")
rospy.loginfo("Get a ROS parameter: /tutorial3 => {}".format(resp))
