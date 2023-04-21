#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

def server_request(req):
    return

# Initialization of ROS Node
rospy.init_node("service_client")
rospy.loginfo("Service Client Start!")

# Declare the service client
rospy.wait_for_service('tutorial3')
tutorial_client = rospy.ServiceProxy('tutorial3', SetBool)

# Request the response
req = SetBoolRequest()
req.data = True
resp = tutorial_client(req)

# Printout
rospy.loginfo("Service returns: {}".format(resp.success))
