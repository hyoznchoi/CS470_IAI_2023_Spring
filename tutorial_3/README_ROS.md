# This is a ROS Tutorial
## Pre-requites for this tutorial
Please install Ubuntu 20.04 and ROS Noetic.

## Create Packages
You can create a package to run your own nodes
~~~~bash
cd ~/catkin_ws/src/
catkin_create_pkg CS470_IAI_2023_Spring std_msgs rospy roscpp
~~~~
This command creates a package named CS470_IAI_2023_Spring  that lists std_msgs, rospy, and roscpp as dependencies. Then you can build your catkin workspace
~~~~bash
cd ~/catkin_ws
catkin_make
~~~~
If you want catkin build, you can use it instead of catkin_make.
After then, you should source
~~~~bash
source ~/catkin_ws/devel/setup.bash
~~~~

Verify your package ahs installed by moving to the package folder
~~~~bash
roscd CS470_IAI_2023_Spring
~~~~

Lastly, load the new environmental variables and also add to path the environmental setup to your bashrc
~~~~bash
source ./devel/setup.bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
~~~~
You can verify your ROS install by confirming the environmental variables
~~~~bash
env| grep ROS
~~~~
Make sure that if ROS_MASTER_URI and ROS_ROOT, ETC are setup.

## 1. ROS Basics
Basic ROS Concepts (ROS packages, launch files, nodes)

Master: Name service for ROS (i.e. helps nodes find each other)
roscore: Master + rosout + parameter server (parameter server will be introduced later)

roscore is the first thing you should run when using ROS.
~~~~bash
roscore
~~~~
Nodes: A node is an executable that uses ROS to communicate with other nodes.

First open a new terminal and use rosnode to understand what roscore did. The rosnode list command lists shows the current running ROS nodes: 
~~~~bash
rosnode list
~~~~
rosout: ROS equivalent of stdout/stderr
To get information about a specific node, please use rosnode info command:
~~~~bash
rosnode info /rosout
~~~~
## 2. ROS Topic

Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
rostopic is a command-line tool for interacting with ROS topics. For example:
~~~~bash
rostopic list
~~~~
~~~~bash
rostopic echo /topic_name
~~~~
display Messages published to /topic_name.

Messages: ROS data type used when subscribing or publishing to a topic.
Let's test topic publisher and listener! First, check if you have the two files by changing your working directory:
~~~~bash
roscd CS470_IAI_2023_Spring/src
~~~~
Note that 'roscd' works after loading the ROS' environment setup.
<p>&nbsp;</p>
(https://github.com/pidipidi/cs470_IAI_2023_Spring/blob/main/tutorial_3/ros_topic.png )

## Python
Please, create the publisher file:
~~~~bash
gedit talker.py
~~~~
Write the following code:
~~~~bash
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello cs470 student. You are smart %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
~~~~
You will be able to see there is a publisher that sends a string format of message via 'chatter' topic. Then, let's create a subscriber file 
~~~~bash
gedit listener.py
~~~~
Write the following code:
~~~~bash
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
~~~~
This file subscribes the string format of messages via 'chatter' topic and print out the contents using the loginfo function.
<p>&nbsp;</p>

Please, open three terminals and then run one roscore and the talker&listener nodes. <p></p>
First, install rospkg and netifaces if you dont have them
~~~~bash
pip install rospkg
pip install netifaces
~~~~
Lets build our catkin package after adding new files.
~~~~bash
cd ~/catkin_ws
catkin_make
~~~~
On Terminal 1:
~~~~bash
roscore
~~~~
On Terminal 2: the talker node will publish a message via chatter topic.
~~~~bash
chmod +x talker.py
source ~/.bashrc
rosrun CS470_IAI_2023_Spring talker.py
~~~~
On Terminal 3: the listener node will subscribe and printout the message. 
~~~~bash
chmod +x listener.py
source ~/.bashrc
rosrun CS470_IAI_2023_Spring listener.py
~~~~
<p>&nbsp;</p>


## Command line Tools

On Terminal 1:
~~~~bash
roscore
~~~~
On Terminal 2: You can publish the chatter topic from your terminal.
~~~~bash
rostopic pub chatter std_msgs/String "data: 'test'"
~~~~
On Terminal 3: You can subscribe the chatter topic via your terminal.
~~~~bash
rostopic echo chatter
~~~~

## 3. Service

(https://github.com/pidipidi/cs470_IAI_2023_Spring/blob/main/tutorial_3/ros_service.png )
Let's test ROS service! ROS service is a kind of hand-shake system between a server and a client, where the service server responses given the service client's request. By using these, you can prevent to loose information during important communication. 
You will be able to see example service code from service_server.py and service_client.py. Please, try to run following commands after checking the code. 


## Python
Please, create the service client file:
~~~~bash
gedit service_client.py
~~~~
Write the following code:
~~~~bash
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
~~~~
You will be able to see there is a service client file. Then, let's create a service server file 
~~~~bash
gedit service_server.py
~~~~
Write the following code:
~~~~bash
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
~~~~

Lets build our catkin package after adding new files.
~~~~bash
cd ~/catkin_ws
catkin_make
~~~~
On Terminal 1:
~~~~bash
roscore
~~~~
On Terminal 2: the server node will response to the request of the service request.
~~~~bash
chmod +x service_server.py
source ~/.bashrc
rosrun CS470_IAI_2023_Spring service_server.py
~~~~
On Terminal 3: the service client node will request a response to the server.
~~~~bash
chmod +x service_client.py
source ~/.bashrc
rosrun CS470_IAI_2023_Spring service_client.py
~~~~

## Command line Tools

You can request to the server via your terminal:

~~~~bash
rosservice call /tutorial3 "data: false" 
~~~~

## 4. Parameter

(https://github.com/pidipidi/cs470_IAI_2023_Spring/blob/main/tutorial_3/param.png )

Let's test a ROS parameter server! In more detail, A ROS parameter server is a type of shared dictionary services over ROS nodes. This is often used for data sharing between nodes that do not require strict real-time constraints. Please, be aware that storing and retrieving are not such fast enough to communicate information at runtime. 


## Python

Please, create the parameter file:
~~~~bash
gedit ros_parameter.py
~~~~
Write the following code:
~~~~bash
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
~~~~
On Terminal 1:
~~~~bash
roscore
~~~~
Lets build our catkin package after adding new files.
~~~~bash
cd ~/catkin_ws
catkin_make
~~~~
On Terminal 2: the talker node will publish a message via chatter topic.
~~~~bash
chmod +x ros_parameter.py
source ~/.bashrc
rosrun CS470_IAI_2023_Spring ros_parameter.py
~~~~

## Commandline Tools

You can store information to the parameter namespace (i.e., dictionary key):
~~~~bash
rosparam set /tutorial3 "test"
~~~~

You can retrieve the information give the namespace:
~~~~bash
rosparam get /tutorial3
~~~~


## ETC
There are many useful command-line tools like rostopic, rqt_graph, rosbag, etc. Please, see http://wiki.ros.org/ROS/CommandLineTools

