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
from std_msgs.msg import String
from drcs.msg import ConnectionInfo
from drcs.msg import UnitInfo
from drcs.msg import OutSlaveGoal
from drcs.msg import TranslocCoord
from drcs.msg import DecartCoord
#forms
from forms import *
#parsing
import argparse, shlex
# scheduler
from threading import Timer

import numpy as np
from math import factorial

import matplotlib.pyplot as plt

#   for Ctrl-C
import signal
def signal_handler(signal, frame):
    print('\b\bYou pressed Ctrl+C.\nExiting...')
signal.signal(signal.SIGINT, signal_handler)

#
#
#

# Reacts and does main algorithms
class Analyser(ROS_connector):
    def __init__(self):
        if debug_node_create: rospy.init_node("Analyser")
        ROS_connector.__init__(self)
        self.form_manager = FormationManager(self)
        #id - int : coord - Vector
        self.units = {}
        self.id = "0"
        self.load_configuration()
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def init_subscribers(self):
        self.add_subscribption("IN_S_GOAL", String, self.IN_S_GOAL)
        self.add_subscribption("M_GOAL", String, self.M_GOAL)
        self.add_subscribption("GROUP_INFO", UnitInfo, self.GROUP_INFO)
        self.add_subscribption("CONNECTION_INFO", ConnectionInfo, self.CONNECTION_INFO)
        self.add_subscribption("SELF_COORD", DecartCoord, self.SELF_COORD)

    def init_publishers(self):
        self.add_publisher("OUT_S_GOAL", OutSlaveGoal)
        self.add_publisher("UNIT_INFO", UnitInfo)
        self.add_publisher("TRANSLOC_COORD", TranslocCoord)
        self.add_publisher("UNITS_INFO", UnitInfo)

    def load_configuration(self):
        #catch
        try:
            self.id = rospy.get_param("id")
            debug_print_function(self, "id")
        except:
            pass

    def OUT_S_GOAL_broadcast_GOTO(self, x, y, r):
        for _id in self.units:
            self.publish("OUT_S_GOAL", self.pack_OutSlaveGoal(_id, x, y, r))

    def OUT_S_GOAL_publish(self, _id, x, y):
        self.publish("OUT_S_GOAL", self.pack_OutSlaveGoal(_id, x, y, 0.1))

    def pack_OutSlaveGoal(self, _id, x, y, r):
        return OutSlaveGoal(_id, "GOTO -x " + str(x) + " -y " + str(y) + " -r " + str(r))

    def IN_S_GOAL(self, data):
        # debug_print_function(self, data.data)
        args = self.IN_S_GOAL_parse(data.data)
        args.func(args)

    def IN_S_GOAL_parse(self, args):
        parser = argparse.ArgumentParser()
        subparser = parser.add_subparsers()
        parser_IN_S_GOAL = subparser.add_parser("GOTO")
        parser_IN_S_GOAL.add_argument("-x", dest = "x", type=float)
        parser_IN_S_GOAL.add_argument("-y", dest = "y", type=float)
        parser_IN_S_GOAL.add_argument("-r", dest = "r", type=float, default=0.0)
        parser_IN_S_GOAL.set_defaults(func = self.IN_S_GOAL_handle_GOTO)

        return parser.parse_args(shlex.split(args))

    def IN_S_GOAL_handle_GOTO(self, args):
        # debug_print_function(self)

        tc = TranslocCoord()
        tc.move = True
        tc.x = args.x
        tc.y = args.y
        tc.r = args.r

        self.publish("TRANSLOC_COORD", tc)

    def M_GOAL(self, data):
        debug_print_function(self, data.data)
        args = self.M_GOAL_parse(data.data)
        args.func(args)

    def M_GOAL_parse(self, args):
        parser = argparse.ArgumentParser()
        subparser = parser.add_subparsers()

        parser_M_GOAL = subparser.add_parser("FORM")
        parser_M_GOAL.add_argument("-f", dest = "form")
        parser_M_GOAL.set_defaults(func = self.M_GOAL_handle_FORM)

        parser_M_GOAL = subparser.add_parser("MOVE_TO")
        parser_M_GOAL.add_argument("-x", dest = "x", type=float)
        parser_M_GOAL.add_argument("-y", dest = "y", type=float)
        parser_M_GOAL.add_argument("-r", dest = "r", type=float, default=0.0)
        parser_M_GOAL.set_defaults(func = self.M_GOAL_handle_MOVE_TO)

        parser_M_GOAL = subparser.add_parser("STOP")
        parser_M_GOAL.set_defaults(func = self.M_GOAL_handle_STOP)

        return parser.parse_args(shlex.split(args))

    def M_GOAL_handle_FORM(self, args):
        debug_print_function(self)
        self.form_manager.set_formation(args.form)

    def M_GOAL_handle_MOVE_TO(self, args):
        debug_print_function(self)
        self.form_manager.move(args.x, args.y, args.r)

    def M_GOAL_handle_STOP(self, args):
        debug_print_function(self)
        self.form_manager.stop()

    def GROUP_INFO(self, data):
        # debug_print_function(self, data.id)
        if self.units.has_key(data.id):
            self.units[data.id] = Vector(data.x, data.y)
            self.publish("UNITS_INFO", UnitInfo(data.id, True, data.x, data.y))

        # debug_print(self.units[data.id])

    def CONNECTION_INFO(self, data):
        debug_print_function(self, data.id + " " + data.state)
        if data.state == "Connected":
            if not self.units.has_key(data.id):
                self.units[data.id] = Vector()
                self.publish("UNITS_INFO", UnitInfo(data.id, True, 0, 0))
        if data.state == "Disconnected":
            if  self.units.has_key(data.id):
                self.units.pop(data.id)
                self.publish("UNITS_INFO", UnitInfo(data.id, False, 0, 0))
        debug_print("Unit ids:")
        debug_print(self.units.keys())

    def SELF_COORD(self, data):
        # debug_print_function(self)

        ui = UnitInfo()
        ui.id = self.id
        ui.x = data.x
        ui.y = data.y

        self.publish("UNIT_INFO", ui)

###

# Handles crowd formation 
class FormationManager():
    def __init__(self, analyser):
        debug_print_function(self)
        self.formation = None
        self.command_id = 0
        self.forming_period = 0.5
        self.robot_diameter = 1
        self.opt_distance = 4
        self.r_eps = 0.2
        self.r_result = 10.0
        self.form_coeff = 0.5
        self.r_form_max = 1.6
        self.analyser = analyser

        self.x_1 = np.array([])
        self.y_1 = np.array([])
        self.x_2 = np.array([])
        self.y_2 = np.array([])
        self.x_3 = np.array([])
        self.y_3 = np.array([])
        self.x_4 = np.array([])
        self.y_4 = np.array([])
        self.x_5 = np.array([])
        self.y_5 = np.array([])
        self.x_6 = np.array([])
        self.y_6 = np.array([])

        self.count = 0


    def __del__(self):
        debug_print_function(self)

    def move(self, x, y, r):
        debug_print_function(self, str(self.command_id))
        self.command_id += 1
        if self.formation is None:
            self.analyser.OUT_S_GOAL_broadcast_GOTO(x, y, r)
        else:
            self.loop_forming(self.command_id, x, y)

    def loop_forming(self, command_id, x, y):
        debug_print_function(self, str(command_id) + " "  + str(x) + " " + str(y))

        if not command_id == self.command_id:
            return

        results = self.get_result_positions(x, y)

        self.count += 1

        #save
        if self.count > 5:
            try:
                self.x_1 = np.append(self.x_1, self.analyser.units["1"].x())
                self.y_1 = np.append(self.y_1, self.analyser.units["1"].y())
            except:
                pass
            try:
                self.x_2 = np.append(self.x_2, self.analyser.units["2"].x())
                self.y_2 = np.append(self.y_2, self.analyser.units["2"].y())
            except:
                pass
            try:
                self.x_3 = np.append(self.x_3, self.analyser.units["3"].x())
                self.y_3 = np.append(self.y_3, self.analyser.units["3"].y())
            except:
                pass
            try:
                self.x_4 = np.append(self.x_4, self.analyser.units["4"].x())
                self.y_4 = np.append(self.y_4, self.analyser.units["4"].y())
            except:
                pass
            try:
                self.x_5 = np.append(self.x_5, self.analyser.units["5"].x())
                self.y_5 = np.append(self.y_5, self.analyser.units["5"].y())
            except:
                pass
            try:
                self.x_6 = np.append(self.x_6, self.analyser.units["6"].x())
                self.y_6 = np.append(self.y_6, self.analyser.units["6"].y())
            except:
                pass

        if self.group_in_position(x, y):# and self.units_in_position(results):
            self.stop()

            print self.x_1.size
            print self.y_1.size
            print self.x_2.size
            print self.y_2.size
            print self.x_3.size
            print self.y_3.size
            print self.x_4.size
            print self.y_4.size
            print self.x_5.size
            print self.y_5.size
            print self.x_6.size
            print self.y_6.size

            #for plots in xy plane
            plt.plot(self.x_1, self.y_1, color='red')
            plt.plot(self.x_2, self.y_2, color='blue')
            plt.plot(self.x_3, self.y_3, color='green')
            plt.plot(self.x_4, self.y_4, color='purple')
            plt.plot(self.x_5, self.y_5, color='yellow')
            plt.plot(self.x_6, self.y_6, color='aqua')

            plt.xlim(-10, 20)
            plt.ylim(-10, 20)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.grid()
            plt.show()
            return

        self.print_units_target_distances(results)

        for _id in self.analyser.units:
            self.analyser.OUT_S_GOAL_publish(_id, results[_id].x(), results[_id].y())

        Timer(self.forming_period, self.loop_forming, (command_id, x, y)).start()

    def group_in_position(self, x, y):
        if self.delta_equal(self.mass_center(), Vector(x, y), self.r_eps):
            return True
        else:
            return False

    def units_in_position(self, results):
        for _id in results:
            if not self.delta_equal(results[_id], self.analyser.units[_id], self.r_eps):
                return False
        return True

    def print_units_target_distances(self, results):
        for _id in results:
            debug_print(str(_id) + " " + str(results[_id].__sub__(self.analyser.units[_id]).norm()))

    def mass_center(self):
        mc = Vector()
        for _id in self.analyser.units:
            mc = mc.__add__(self.analyser.units[_id])
        mc = mc.__mul__(1.0 / len(self.analyser.units))
        debug_print_function(self, str(mc))
        return mc

    def delta_equal(self, vector_1, vector_2, r_eps):
        if vector_1.__sub__(vector_2).norm() <= r_eps:
            return True
        else:
            return False

    def get_result_positions(self, x, y):
        # debug_print_function(self)
        # id - int : position - Vector
        result = {}

        formation = self.get_formation_positions()

        for _id in self.analyser.units:
            result[_id] = self.get_result_position(_id, self.get_target_pos(x, y), formation[_id])

        return result

    def get_formation_positions(self):
        debug_print_function(self)
        return self.formation.get_formation_positions(self.analyser.units, self.robot_diameter, self.opt_distance)

    def get_result_position(self, _id, target_pos, form_pos):
        debug_print_function(self)
        # transform to local
        target = target_pos.__sub__(self.analyser.units[_id])
        formation = form_pos.__sub__(self.analyser.units[_id])

        r_to_target = target.norm()
        r_to_form = formation.norm()

        print str(_id) + " " + str(r_to_target) + " " + str(r_to_form)

        # potential sum

        # if r_to_target > self.r_result - self.opt_distance:
            # target = target.__mul__(0.5)#.__mul__(math.pow(r_to_target, 2))
        # target = target

        # try:
        #     target = target.__mul__(1.0 / math.pow(r_to_target, 0.25))
        # except:
        #     pass
        # formation = formation.__mul__(2 * math.pow(r_to_form, 3))

        # if target_pos / self.opt_distance > form_pos:
        # try:
        #     target = target.normalize().__mul__(1.0 / r_to_form)
        # except:
        #     pass
        # formation = formation.normalize().__mul__(math.pow(r_to_form, 2))
        # else:
        try:
            target = target.normalize().__mul__(math.pow(r_to_target, 0.25) + 1.0)
            # target = target.normalize().__mul__(20.0 * r_to_form / r_to_target)
            formation = formation.normalize().__mul__(2 * math.pow(r_to_form, 3))
            
        except:
            pass


        # try:
        #     target = target#.__mul__(1.0 / math.pow(r_to_target, 0.25) + 2.0)
        # except:
        #     pass
        # formation = formation.__mul__(2 * math.pow(r_to_form, 2))

        """
        try:
            target = target.__mul__(math.pow(r_to_target, 0.25))
        except:
            pass
        formation = formation.__mul__(2 * math.pow(r_to_form, 2))
        """

        local_result = target.__add__(formation)
        #back to global

        formation_correction = 1.0
        # if r_to_form < self.r_form_max:
            # formation_correction = math.pow((1.0 - self.form_coeff) / self.r_form_max * r_to_form, 2) + self.form_coeff
        # angle_correction = 1.0 - (2.0 - target.normalize().__sub__(formation.normalize()).norm()) / 2.0
        # formation_correction *= angle_correction

        distance = local_result.norm()
        if distance > self.r_result:
            local_result = local_result.__mul__( 1.0 / distance * self.r_result * formation_correction * 3.0)
        # else:
            # local_result = local_result.__mul__(3.0)

        return self.analyser.units[_id].__add__(local_result)

    def get_target_pos(self, x, y):
        return Vector(x, y)

    def set_formation(self, form):
        debug_print_function(self, form)
        if form == "FREE":
            self.formation = None
        if form == "CIRCLE":
            self.formation = Circle()

    def stop(self):
        debug_print_function(self)
        for _id in self.analyser.units:
            self.analyser.OUT_S_GOAL_publish(_id, self.analyser.units[_id].x(), self.analyser.units[_id].y())
###

# a = Analyser()

# rospy.spin()