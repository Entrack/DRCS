#!/usr/bin/env python

#   all this is required to import ROS_connector from the same-level directory
import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("drcs") + '/scripts/includes')
from ros_connector import ROS_connector
from ros_connector import get_word
del sys
del rospkg

#   includes
#ros
import rospy
#msgs
from std_msgs.msg import String
from drcs.msg import ConnectionInfo
from drcs.msg import UnitInfo
from drcs.msg import OutSlaveGoal
#internet
from twisted.internet import reactor
from twisted.internet.protocol import Protocol, Factory, ClientFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet.endpoints import TCP4ClientEndpoint, connectProtocol



#   for Ctrl-C
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

#   Protocol instance that is binded to particular socket
class Declamator(LineReceiver):
    def __init__(self, factory, addr, _id = "0"):
        self.addr = addr
        self.id = _id
        self.factory = factory
        self.delimiter = '\n'
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def connectionMade(self):
        debug_print_function(self)

    def connectionLost(self, reason):
        debug_print("Declamator connectionLost on socket: ")
        debug_print(self.addr.host + " " + str(self.addr.port))
        self.factory.delete_socket(self.id, self)

    def sendLine(self, line):
        try:
            self.transport.write(line + '\n')
        except:
            self.transport.write(line.data + '\n')

    def send_with_name(self, topic, line):
        try:
            self.transport.write(topic + " " + line + '\n')
        except:
            self.transport.write(topic + line.data + '\n')

    def lineReceived(self, line):
        debug_print_function(self, line)
        #catch state
        self.register_protocol_socket(line)

    def register_protocol_socket(self, line):
        debug_print_function(self)
        if get_word(line, 1) == "SERVER":
            #catch
            _id = get_word(line, 2)
            self.id = _id
            self.factory.add_socket(_id, self)

###

#   Establishes and stores sockets, which then are used by it's ROS callbacks
class DeclamatorFactory(ClientFactory, ROS_connector):
    def __init__(self):
        if debug_node_create: rospy.init_node("Declamator")
        ROS_connector.__init__(self)
        # id - int : socket - Protocol
        self.sockets = {}
        self.addressses = []
        self.is_sim = True
        self.load_configuration()
        self.net_rescan_time = 5
        self.latch_time = 0.1
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def buildProtocol(self, addr):
        return Declamator(self, addr)

    def connectToPeer(self, host, port):
        reactor.connectTCP(host, port, self)

    def rescan_network(self):
        #for all network range connectToPeer()
        # debug_print_function(self)
        if self.is_sim:
            for port in range(8000, 8256):
                if not port in self.addressses:
                    self.connectToPeer("localhost", port)
        else:
            for ip in range(0, 256):
                if not "192.168.0." + str(ip) in self.addressses:
                    self.connectToPeer("192.168.0." + str(ip), 8256)
                        

    def loop_network_scaning(self):
        self.rescan_network()
        reactor.callLater(self.net_rescan_time, self.loop_network_scaning)

    def add_socket(self, _id, protocol):
        #catch
        self.sockets[_id] = protocol
        if self.is_sim:
            self.addressses.append(protocol.addr.port)
        else:
            self.addressses.append(protocol.addr.host)
        debug_print_function(self, _id + " " + protocol.addr.host + " " + str(protocol.addr.port))
        ci = ConnectionInfo()
        ci.id = _id
        ci.state = "Connected"
        self.publish("CONNECTION_INFO", ci)

    def delete_socket(self, _id, protocol):
        del self.sockets[_id]
        if self.is_sim:
            self.addressses.remove(protocol.addr.port)
        else:
            self.addressses.remove(protocol.addr.host)
        debug_print_function(self, _id + " " + protocol.addr.host + " " + str(protocol.addr.port))
        ci = ConnectionInfo()
        ci.id = _id
        ci.state = "Disconnected"
        self.publish("CONNECTION_INFO", ci)

    def send_message(self, _id, topic, line):
        #catch
        s = self.sockets[_id]
        try:
            s.send_with_name(topic, line)
        except:
            pass

    def init_subscribers(self):
        self.add_subscribption("OUT_S_GOAL", OutSlaveGoal, self.OUT_S_GOAL)
        self.add_subscribption("UNIT_INFO", UnitInfo, self.UNIT_INFO)

    def init_publishers(self):
        self.add_publisher("CONNECTION_INFO", ConnectionInfo)

    def OUT_S_GOAL(self, data):
        # debug_print_function(self, data.id + data.goal)
        self.send_message(data.id, "IN_S_GOAL", data.goal)

    def UNIT_INFO(self, data):
        debug_print_function(self)
        for _id in self.sockets:
            self.send_message(_id, "GROUP_INFO", "-id " + data.id + " -x " + str(data.x) + " -y " + str(data.y))

    def load_configuration(self):
        #catch
        try:
            self.is_sim = rospy.get_param("/is_sim")
            debug_print_function(self, "/is_sim")
        except:
            pass
###

# factory = DeclamatorFactory()
# rospy.sleep(factory.latch_time)
# factory.loop_network_scaning()

# reactor.run()