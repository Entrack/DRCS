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
#msgs
from drcs.msg import PolarVel
#internet
from twisted.protocols.basic import LineReceiver
from twisted.internet.protocol import Factory
from twisted.internet import reactor
from twisted.internet.endpoints import TCP4ServerEndpoint
#parsing
import argparse, shlex
# velocity sender, debug
from velocity_sending.velocity_senders import *

#for Ctrl-C
import signal
def signal_handler(signal, frame):
    print('\b\bYou pressed Ctrl+C.\nExiting...')
    reactor.stop()
signal.signal(signal.SIGINT, signal_handler)

#
#
#

# Handles manual drive connection
class Mover(LineReceiver):
    def __init__(self, factory):
        self.factory = factory
        self.delimiter = '\n'
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def connectionMade(self):
        debug_print_function(self)
        self.sendLine("MOVER " + self.factory.id)
        self.factory.is_manual = True

    def connectionLost(self, reason):
        debug_print_function(self)
        self.factory.is_manual = False

    def lineReceived(self, line):
        debug_print_function(self, line)
        args = self.factory.manual_control_parse(line)
        args.func(args)
###

# Produces Movers to handle socket if someone connects
class MoverFactory(Factory, ROS_connector):
    def __init__(self):
        if debug_node_create: rospy.init_node("Mover")
        ROS_connector.__init__(self)
        self.is_manual = False
        self.is_sim = False
        self.id = "0"
        self.port = 8512
        self.sender = None
        self.load_configuration()
        self.init_sender()
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def init_sender(self):
        if self.is_sim:
            self.sender = GazeboVS()
        else:
            self.sender = SerialVS()

    def buildProtocol(self, addr):
        debug_print_function(self)
        return Mover(factory=self)

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

    def init_subscribers(self):
        self.add_subscribption("MOTION_VEL", PolarVel, self.MOTION_VEL)

    def MOTION_VEL(self, data):
        debug_print_function(self, str(data.linear) + " " + str(data.angular))
        if not self.is_manual:
            self.sender.send_velocity(data.linear, data.angular)

    def manual_control_parse(self, args):
        parser = argparse.ArgumentParser()
        subparser = parser.add_subparsers()
        parser_manual_control = subparser.addparser("VELOCITY")
        parser_manual_control.add_argument("-a", dest = "angular")
        parser_manual_control.add_argument("-l", dest = "linear")
        parser_manual_control.set_defaults(func = self.manual_control_handle)

        return parser.parse_args(shlex.split(args))

    def manual_control_handle(self, args):
        self.sender.send_velocity(args.angular, args.linear)
###

mf = MoverFactory()
if mf.is_sim:
    endpoint = TCP4ServerEndpoint(reactor, mf.port + 256)
else:
    endpoint = TCP4ServerEndpoint(reactor, 8256 + 256)
endpoint.listen(mf)

reactor.run()