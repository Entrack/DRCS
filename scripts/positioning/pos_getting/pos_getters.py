#!/usr/bin/env python

#   includes
#ros
import rospy
#structs
from drcs.srv import PosSystCoord
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
#absctract classes
from abc import ABCMeta, abstractmethod
#internet
from twisted.internet import reactor
from twisted.protocols.basic import LineReceiver
from twisted.internet.endpoints import TCP4ClientEndpoint, connectProtocol
#quaternions
import tf
#pos
from vector import Vector
#parsing
import argparse, shlex

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

#Template for position getters
class PosGetter():
    __metaclass__=ABCMeta

    @abstractmethod
    def get_position(self):
        pass

    def load_configuration(self):
        #catch
        try:
            self.id = rospy.get_param("id")
            debug_print_function(self, "id")
        except:
            pass 
###

# Gazebo ros position getter
class GazeboPG(PosGetter):
    def __init__(self):
        self._id = "0"
        self.position = None
        self.load_configuration()
        self.init_services()
        debug_print_function(self)

    def get_position(self):
        pose = self.position("pioneer2dx_" + self.id, "world").pose
        quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        psc = PosSystCoord()
        psc.x = pose.position.x
        psc.y = pose.position.y
        # [-pi, pi]; corresponds to rotation sign
        psc.angle = euler[2]
        
        return psc.angle, psc.x, psc.y

    def init_services(self):
        self.position = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service("/gazebo/get_model_state")
        debug_print_function(self)
###

# Client of swistrack computer vision system
class SwistrackPG(PosGetter, LineReceiver):
    def __init__(self):
        self._id = "0"
        self.server_ip = "127.0.0.1"
        self.server_port = 9000
        self.critical_time = 0.01
        self.delimiter = '\n'
        self.point = None
        self.coords = Vector()
        self.angle = 0.0
        self.is_working = False
        self.load_configuration()
        debug_print_function(self)

    def get_position(self):
        # critical section
        while True:
            if self.is_working:
                rospy.sleep(self.critical_time)
            else:
                psc = PosSystCoord()
                self.is_working = True
                psc.x = self.coords.x()
                psc.y = self.coords.y()
                psc.angle = self.angle
                self.is_working = False
                return psc

    def lineReceived(self, line):
        args = self.parse_orient(line)
        args.func(args)

    def parse_orient(self, args):
        parser = argparse.ArgumentParser()
        subparser = parser.add_subparsers()

        parser_orient = subparser.addparser("ORIENT")
        parser_orient.add_argument("-x", dest = "x")
        parser_orient.add_argument("-y", dest = "y")
        parser_orient.add_argument("-a", dest = "angle")
        parser_orient.set_defaults(func = self.handle_orient)

        return parser.parse_args(shlex.split(args))

    def handle_orient(self, args):
        self.set_new_coords(args)

    def set_new_coords(self, args):
        # critical section
        has_changed = False
        while not has_changed:
            if self.is_working:
                rospy.sleep(self.critical_time)
            else:
                self.is_working = True
                self.coords = Vector(args.x, args.y)
                self.angle = args.angle
                self.is_working = False
                has_changed = True

    def connectToServer(self):
        self.point = TCP4ClientEndpoint(reactor, self.server_ip, self.server_port)
        connectProtocol(self.point, self)
###





























# while not rospy.is_shutdown():
#     gpg.test_velocity()
#     rospy.sleep(0.1)
    # def test_velocity(self):
    #     ms = ModelState()
    #     ms.model_name = "pioneer2dx"

    #     vel = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    #     ms.twist.linear.x = 8.0

    #     vel(ms)