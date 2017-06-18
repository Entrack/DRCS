#!/usr/bin/env python

#   all this is required to import ROS_connector from the same-level directory
import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("drcs") + '/scripts/includes')
from ros_connector import ROS_connector
from ros_connector import remove_first_word, get_word
del sys
del rospkg

#   includes
#ros
import rospy
#absctract classes
from abc import ABCMeta, abstractmethod
#msgs
from gazebo_msgs.srv import SpawnModel, DeleteModel, ApplyBodyWrench
from geometry_msgs.msg import Twist
#position
import tf
from geometry_msgs.msg import *
#imports
from os.path import expanduser
#random
import random
#serial
import serial

#   for debug
import inspect
def debug_print_function(self, additional = ""):
    if debug: print self.__class__.__name__ + " " + inspect.stack()[1][3] + " " + str(additional)

def debug_print_name(self, additional = ""):
    if debug: print self.__class__.__name__ + " " + additional

def debug_print(line):
    if debug: print line

debug = True
debug_node_create = True

#
#
#

# Template for senders
class VelocitySender():
    __metaclass__=ABCMeta

    @abstractmethod
    def send_velocity(self, angular, linear):
        pass

    def load_configuration(self):
        pass
###

# Gazebo ros velocity sender
class GazeboVS(VelocitySender, ROS_connector):
    def __init__(self):
        self.id = "0"
        self.name = ""
        self.range = 10.0
        self.load_configuration()
        self.name = "pioneer2dx_" + self.id
        self.topic_name = "/" + self.name + "/cmd_vel"
        self.spawn_robot()
        self.muiltiplier = 2.0 # 1.0 for non-navigtion mode
        ROS_connector.__init__(self)
        debug_print_function(self)

    def init_publishers(self):
        self.add_publisher(self.topic_name, Twist)

    def spawn_robot(self):
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        
        models_path = expanduser("~/.gazebo/models/")
        with open(models_path + "pioneer2dx/model_drive.sdf", "r") as f:
            product_xml = f.read()

        item_pose = Pose(self.random_point(), Quaternion())

        spawn_model(self.name, product_xml, "", item_pose, "world")

    def delete_robot(self):
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("gazebo/delete_model")
        delete_model(self.name)

    def random_point(self):
        return Point(random.uniform(-self.range, self.range), random.uniform(-self.range, self.range), 0)

    def load_configuration(self):
        try:
            self.id = rospy.get_param("id")
            debug_print_function(self, "id")
        except:
            pass

    def send_velocity(self, linear, angular):
        t = Twist()
        t.linear.x = linear / 1.5 * self.muiltiplier #1.43
        t.angular.z = angular / 1.6 * self.muiltiplier # 1.68
        # debug_print_function(self, t)
        self.publish(self.topic_name, t)
###

# STM32 serial port sender
class SerialVS(VelocitySender):
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyS0")
        self.ser.baudrate = 19200
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def load_configuration(self):
        pass

    def send_velocity(self, angular, linear):
        # self.ser.write("VELOCITY -l " + str(linear) + " -a " + str(angular))
        X = linear * 100.0
        Y = linear * 100.0
        X += -angular / 2.0 * 100.0 * (-linear / 2.0 + 1.0)
        Y += angular / 2.0 * 100.0 * (-linear / 2.0 + 1.0)
        if X > 100.0:
            X = 100.0
        if Y > 100.0:
            Y = 100.0
        self.ser.write("M" + str(X) + "," + str(Y) + '\n')

###


# if debug_node_create: rospy.init_node("VelocitySender")
# g = GazeboVS()
# while not rospy.is_shutdown():
#     g.send_velocity(2.0, 0.1)
#     rospy.sleep(1.0)