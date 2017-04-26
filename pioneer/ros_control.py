#!/usr/bin/env python
import rospy
from random import random
from geometry_msgs.msg import Twist


class RosControl:
    """
Control the rotob with differencial drive using ROS cmd_vel messages
    """

    def __init__(self, topic):
        self._topic = topic
        self._publisher = rospy.Publisher('/%s/cmd_vel' % topic,
                                          Twist, queue_size=8)
        rospy.init_node('roscontrol', anonymous=True)

    def __repr__(self):
        return 'RosControl(%s)' % self._topic

    def control_loop(self, arg):
        self.set_vel(random(), 1 - 2 * random());
        return True

    def set_vel(self, linear, angular):
        print linear, angular
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._publisher.publish(msg)

    def run(self):
        timer = rospy.Timer(rospy.Duration(2.0), self.control_loop)
        rospy.spin()


if __name__ == '__main__':
    RosControl('pioneer2dx_0').run()