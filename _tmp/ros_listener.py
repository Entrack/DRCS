#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class ROS_Listener():

    def __init__(self, topic, callback):
        self.sub = rospy.Subscriber(topic, String, callback)
###
