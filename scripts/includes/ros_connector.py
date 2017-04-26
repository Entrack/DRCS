#!/usr/bin/env python

import rospy

#   for debug
import inspect
def debug_print_function(self, additional = ""):
    if debug: print self.__class__.__name__ + " " + inspect.stack()[1][3] + " " + str(additional)

def debug_print_name(self, additional = ""):
    if debug: print self.__class__.__name__ + " " + additional

def debug_print(line):
    if debug: print line

debug = True

#   TMP for line processing
def remove_first_word(line):
        #catch
        return line.split(' ', 1)[1]

def get_word(line, number):
    #catch
    return line.split()[number - 1]
#

#
#
#

# ROS wrapper
class ROS_connector:
    def __init__(self):
        #defaults
        self.rate = rospy.Rate(10)
        self.queue_size = 10
        self.latch_time = 0.1
        self.topics = {}
        self.subs = []

        self.init_from_ros_params()
        self.init_publishers()
        self.init_subscribers()

    def init_from_ros_params(self):
        try:
            self.queue_size = rospy.get_param("queue_size")
        except:
            pass
        try:
            self.rate = rospy.Rate(rospy.get_param("rate"))
        except:
            pass

    def add_subscribption(self, name, data_class, callback):
        self.subs.append(rospy.Subscriber(name, data_class, callback))

    def add_publisher(self, topic, data_class):
        self.topics[topic] = rospy.Publisher(topic, data_class, queue_size=self.queue_size)

    def publish(self, topic, message):
        if self.topics.has_key(topic):
            self.topics[topic].publish(message)
            # debug_print("Published to " + topic)

    def init_publishers(self):
        #should be overwritten
        pass

    def init_subscribers(self):
        #should be overwritten
        pass

    def deleteRosLinks(self):
        self.deleteSubs()
        self.deletePubs()

    def deleteSubs(self):
        for item in self.subs:
            item.unregister()

    def deletePubs(self):
        del self.topics
###

