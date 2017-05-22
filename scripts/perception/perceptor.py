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
#msgs
from std_msgs.msg import String
from drcs.msg import UnitInfo
#internet
from twisted.protocols.basic import LineReceiver
from twisted.internet.protocol import Factory
from twisted.internet import reactor
from twisted.internet.endpoints import TCP4ServerEndpoint
#parsing
import argparse, shlex

#for Ctrl-C
import signal
def signal_handler(signal, frame):
    print('\b\bYou pressed Ctrl+C.\nExiting...')
    reactor.stop()
signal.signal(signal.SIGINT, signal_handler)

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

# Socket instance for listening 
class Perceptor(LineReceiver):
    def __init__(self, factory):
        self.factory = factory
        self.delimiter = '\n'
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def connectionMade(self):
        debug_print_function(self)
        self.sendLine("SERVER " + self.factory.id)

    def connectionLost(self, reason):
        debug_print_function(self)

    def lineReceived(self, line):
        debug_print_function(self, line)
        #catch
        topic = get_word(line, 1)
        #catch
        if topic == "GROUP_INFO":
            args = self.factory.GROUP_INFO_parse(line)
            args.func(args)
        else:
            debug_print_function(self, line)
            self.factory.publish(topic, remove_first_word(line))

    
###

# Produces Perceptors to handle socket if someone connects
class PerceptorFactory(Factory, ROS_connector):
    def __init__(self):
        if debug_node_create: rospy.init_node("Perceptor")
        ROS_connector.__init__(self)
        self.id = "0"
        self.is_sim = True
        self.port = 8256
        self.load_configuration()
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def buildProtocol(self, addr):
        debug_print_function(self)
        return Perceptor(factory=self)

    def init_publishers(self):
        self.add_publisher("IN_S_GOAL", String)
        self.add_publisher("M_GOAL", String)
        self.add_publisher("GROUP_INFO", UnitInfo)

    def load_configuration(self):
        #catch
        try:
            self.is_sim = rospy.get_param("/is_sim")
            debug_print_function(self, "/is_sim")
        except:
            pass
        try:
            self.id = rospy.get_param("id")
            debug_print_function(self, "id")
        except:
            pass
        try:
            if self.is_sim:
                self.port = rospy.get_param("port")
                debug_print_function(self, "port")
        except:
            pass
        
    def GROUP_INFO_parse(self, args):
        parser = argparse.ArgumentParser()
        subparser = parser.add_subparsers()
        parser_GROUP_INFO = subparser.add_parser("GROUP_INFO")
        parser_GROUP_INFO.add_argument("-id", dest = "id", type=str)
        parser_GROUP_INFO.add_argument("-x", dest = "x")
        parser_GROUP_INFO.add_argument("-y", dest = "y")
        parser_GROUP_INFO.set_defaults(func = self.GROUP_INFO_handle)

        return parser.parse_args(shlex.split(args))

    def GROUP_INFO_handle(self, args):
        # debug_print_function(self)
        ui = UnitInfo()
        ui.id = str(args.id)
        ui.online = bool(True)
        ui.x = float(args.x)
        ui.y = float(args.y)

        self.publish("GROUP_INFO", ui)
###

# pf = PerceptorFactory()
# if pf.is_sim:
#     endpoint = TCP4ServerEndpoint(reactor, pf.port)
# else:
#     endpoint = TCP4ServerEndpoint(reactor, 8256)
# endpoint.listen(pf)

# reactor.run()