#!/usr/bin/env python

#   all this is required to import ROS_connector from the same-level directory
import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("drcs") + '/scripts/includes')
from ros_connector import ROS_connector
del sys
del rospkg

#   includes
#ros
import rospy
#msgs
from drcs.msg import TranslocCoord
from drcs.msg import UnitInfo
from drcs.msg import DecartCoord
from drcs.msg import PolarVel
from drcs.msg import RobotOrient
# coords
from vector import Vector
# math
import math
# scheduler
from threading import Timer

#   for Ctrl-C
import signal
def signal_handler(signal, frame):
    print('\b\bYou pressed Ctrl+C.\nExiting...')
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

# Handles reaching GOTO destination
class Translocator(ROS_connector):
    def __init__(self):
        #id - int : coord - Vector
        if debug_node_create: rospy.init_node("Translocator")
        ROS_connector.__init__(self)
        self.units = {}
        self.id = "0"
        self.angle = 0.0
        self.coords = Vector()
        self.command_id = 0
        self.latch_time = 0.01 #0.05
        self.moving_period = 0.2
        self.init_sleep_time = 5.0
        self.eps_r = 0.2
        self.min_repuls_r = 2.5
        self.max_repuls_r = 3.0
        self.repulsion_coeff = 1.0#2.0
        self.r_deceleration = 1.0
        self.eps_deceler = 0.05
        self.chase_r = 4.0
        self.chase_speed = 0.7
        self.load_configuration()
        debug_print_function(self, str(self.id))
        self.load_coords()

    def __del__(self):
        debug_print_function(self)

    def init_subscribers(self):
        self.add_subscribption("TRANSLOC_COORD", TranslocCoord, self.TRANSLOC_COORD)
        self.add_subscribption("UNITS_INFO", UnitInfo, self.UNITS_INFO)
        self.add_subscribption("CURR_ORIENT", RobotOrient, self.CURR_ORIENT)

    def init_publishers(self):
        self.add_publisher("SELF_COORD", DecartCoord)
        self.add_publisher("MOTION_VEL", PolarVel)
        self.add_publisher("CURR_VEL", PolarVel)

    def load_configuration(self):
        #catch
        try:
            self.id = rospy.get_param("id")
            debug_print_function(self, "id")
        except:
            pass

    def load_coords(self):
        rospy.sleep(self.init_sleep_time)
        rospy.wait_for_service("POS_SYST_COORD")
        self.publish("CURR_VEL", PolarVel())

    def TRANSLOC_COORD(self, data):
        debug_print_function(self, "-x " + str(data.x) + " -y " + str(data.y) + " -r " + str(data.r))
        self.command_id += 1
        self.loop_moving(self.command_id, data.x, data.y, data.r)

    def loop_moving(self, command_id, x, y, r):
        debug_print_function(self, str(command_id) + " "  + str(x) + " " + str(y) + " " + str(r))

        if not command_id == self.command_id:
            return

        if self.has_reached(x, y, r):
            self.publish("MOTION_VEL", PolarVel())
            self.publish("CURR_VEL", PolarVel())
            return

        self.move(x, y)
        Timer(self.moving_period, self.loop_moving, (command_id, x, y, r)).start()

    def has_reached(self, x, y, r):
        if self.delta_equal(self.coords, Vector(x, y), r + self.eps_r):
            return True
        else:
            return False

    def delta_equal(self, vector_1, vector_2, r):
        if vector_1.__sub__(vector_2).norm() <= r:
            return True
        else:
            return False

    def move(self, x, y):
        debug_print_function(self, "---------------------------------------")
        
        # vector pointing at the destination
        destination = Vector()
        try:
            destination = Vector(x, y).__sub__(self.coords).normalize()
        except:
            pass


        # get current heading
        current = self.get_current()

        # count local vector of repulsion (norm=[0, 1])
        repulsion = self.get_repulsion(current)
        debug_print(repulsion)
        debug_print(repulsion.norm())

        # sum them
        resulting = Vector()
        try:
            resulting = self.get_resulting(destination, repulsion).normalize()
        except:
            pass



        # get local vector of our direction
        distance = Vector(x, y).__sub__(self.coords).norm()

        linear_velocity = self.get_linear_vel(resulting, current, distance)
        angular_velocity = self.get_angle_vel(resulting, current)

        pv = PolarVel()
        pv.angular = angular_velocity
        pv.linear = linear_velocity
        # send it to the mover
        self.publish("MOTION_VEL", pv)
        # send it to the CURR_VEL
        self.publish("CURR_VEL", pv)

    def get_repulsion(self, current):
        repulsion = Vector()
        for _id in self.units:

            if not self.id == _id:
                _r = self.coords.__sub__(self.units[_id])
                r = self.units[_id].__sub__(self.coords)
                distance = r.norm()
                # debug_print(distance)

                unit_repulsion = self.unit_repulsion(r, current, distance)

                try:
                    repulsion = repulsion.__add__(_r.normalize().__mul__(unit_repulsion))
                except:
                    pass
                

        if repulsion.norm() <= 0.01:
            return Vector()
        else:
            return repulsion

    def get_resulting(self, destination, repulsion):
        return destination.__add__(repulsion.__mul__(self.repulsion_coeff))

    def get_current(self):
        current = Vector(1, 0)
        return current._rotate2D(self.angle / math.pi * 180)

    def get_angle_vel(self, resulting, current):
        if self.almost_cross_product(resulting, current) >= 0:
            return resulting.__sub__(current).norm() / 2.0
        else:
            return -resulting.__sub__(current).norm() / 2.0

    def get_linear_vel(self, resulting, current, distance):
        print resulting
        print current
        angle_correction = (math.sqrt(2.0) - resulting.__sub__(current).norm()) / math.sqrt(2.0)
        #angle_correction = (2.0 - resulting.__sub__(current).norm()) / 2.0

        distance_correction = 1.0
        if distance >= self.r_deceleration - self.eps_deceler * self.r_deceleration:
            distance_correction = 1.0
        else:
            distance_correction = math.sqrt(self.r_deceleration) * math.sqrt(distance + self.eps_deceler * self.r_deceleration)
            distance_correction = distance_correction / self.r_deceleration

        chase_correction = 1.0
        if distance <= self.chase_r:
            chase_correction = math.pow((1.0 - self.chase_speed) / self.chase_r * distance, 2) + self.chase_speed
            # chase_correction = 1.0 / self.chase_r * distance + self.chase_speed

        return  angle_correction * distance_correction * chase_correction

    def unit_repulsion(self, resulting, current, distance):
        angle_correction = 1.0
        try:
            angle_correction = (2.0 - resulting.normalize().__sub__(current).norm() * 2.5) / 2.0
            if angle_correction < 0: angle_correction = 0.0
            pass
        except:
            pass
        distance_correction = 0.0
        if distance >= self.max_repuls_r:
            pass
        else:
            try:
                distance_correction = math.pow(self.min_repuls_r, 1) / math.pow(distance, 1)
            except:
                pass
        return angle_correction * distance_correction

    def almost_cross_product(self, vector_1, vector_2):
        return vector_1.x() * vector_2.y() - vector_1.y() * vector_2.x()

    def UNITS_INFO(self, data):
        debug_print_function(self)
        print data
        if data.online:
            self.units[data.id] = Vector(data.x, data.y)
        else:
            if self.units.has_key(data.id):
                self.units.pop(data.id)

    def CURR_ORIENT(self, data):
        debug_print_function(self, str(data.angle) + " " + str(data.x) + " " +  str(data.y))
        self.angle = data.angle
        self.coords = Vector(data.x, data.y)
        self.publish("SELF_COORD", DecartCoord(data.x, data.y))
###

# t = Translocator()

# rospy.spin()







# if length <= self.max_repuls_r:
                #     tmp = repulsion.__add__(distance.__mul__(self.max_repuls_r / math.pow(length, 2)))
                #     repulsion = tmp.__mul__(self.get_match_coeff(distance, current)) * 5.0