#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ros_parameters import ROS_Param

class ROS_talker():

    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, String, queue_size=ROS_Param.queue_size)
        self.rate = rospy.Rate(ROS_Param.rate)

    def publish(self, message):
        rospy.loginfo(message)
        self.pub.publish(message)

    def sleep(self):
        self.rate.sleep()
###