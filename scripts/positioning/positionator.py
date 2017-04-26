#!/usr/bin/env python

#   includes
#ros
import rospy
#services
from drcs.srv import PosSystCoord
#getters
from pos_getting.pos_getters import *

#   for Ctrl-C
import signal
def signal_handler(signal, frame):
    print('\b\bYou pressed Ctrl+C.\nExiting...')
signal.signal(signal.SIGINT, signal_handler)

#
#
#

# Handles getting coords from pos systems
class Positionator():
    def __init__(self):
        if debug_node_create: rospy.init_node("Positionator")
        self.pos_system = None
        self.is_sim = False
        self.service = None
        self.load_configuration()
        self.init_pos_system()
        self.init_services()
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def load_configuration(self):
        #catch
        try:
            self.is_sim = rospy.get_param("/is_sim")
            debug_print_function(self, "/is_sim")
        except:
            pass

    def init_pos_system(self):
        if self.is_sim:
            self.pos_system = GazeboPG()
        else:
            self.pos_system = SwistrackPG()

    def init_services(self):
        self.service = rospy.Service("POS_SYST_COORD", PosSystCoord, self.handle_coord_request)

    def handle_coord_request(self, req):
        return self.pos_system.get_position()
###

p = Positionator()

if p.is_sim:
    rospy.spin()
else:
    reactor.run()