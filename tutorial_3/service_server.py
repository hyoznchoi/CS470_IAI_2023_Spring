#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def set_bool_srv(req):
    """ Return the same boolean data."""
    print ("Request : ", req.data)
    resp = SetBoolResponse()
    if req:
        resp.success = True
        resp.message = "test"
    else:
        resp.success = False
        resp.message = "test"
    return resp

# Initialization of ROS Node        
rospy.init_node("service_server")
rospy.loginfo("Service Server Node Start!")

# Declare the service server
rospy.Service('tutorial3', SetBool, set_bool_srv)
rospy.loginfo("--------------------------")

# Spin
rate = rospy.Rate(10)
while not rospy.is_shutdown():            
    rate.sleep()
