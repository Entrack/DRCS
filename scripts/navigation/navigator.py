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
from drcs.msg import DecartCoord
from drcs.msg import PolarVel
from drcs.msg import RobotOrient
#services
from drcs.srv import PosSystCoord
#callman
from callman.model import Model
#math
import math
import numpy as np
from math import factorial

import matplotlib.pyplot as plt
import time


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

# Provides current coord based on filtrating positioning system's data 
class Navigator(ROS_connector):
    def __init__(self):
        if debug_node_create: rospy.init_node("Navigator")
        ROS_connector.__init__(self)
        self.model = Model()
        self.id = "0"
        self.position = None
        #moving period
        self.latch_time = 0.2 #0.05
        self.exec_time = 0.0001
        self.positionation_up = True
        self.filter = False
        self.x = np.array([])
        self.s_x = np.array([])
        self.y = np.array([])
        self.angle = np.array([])
        self.measure_count = 0
        self.linspace = np.array([])
        self.orient = RobotOrient()
        self.init_services()
        self.load_configuration()
        debug_print_function(self)

    def __del__(self):
        debug_print_function(self)

    def init_subscribers(self):
        self.add_subscribption("CURR_VEL", PolarVel, self.CURR_VEL)

    def init_publishers(self):
        self.add_publisher("CURR_ORIENT", RobotOrient)

    def init_services(self):
        self.position = rospy.ServiceProxy("POS_SYST_COORD", PosSystCoord)
        rospy.wait_for_service("POS_SYST_COORD")
        debug_print_function(self)

    def load_configuration(self):
        #catch
        try:
            self.id = rospy.get_param("id")
            debug_print_function(self, "id")
        except:
            pass

    def CURR_VEL(self, data):
        debug_print_function(self)
        #ask abot coord

        if data.linear < 0.001 and math.fabs(data.angular) < 0.001:

            #for plots in xy plane
            # color = 'black'
            # if self.id == 1: color = 'red'
            # if self.id == 2: color = 'blue'
            # if self.id == 3: color = 'green'
            # if self.id == 4: color = 'purple'
            # if self.id == 5: color = 'yellow'
            # if self.id == 6: color = 'red'
            # if self.id == 7: color = 'aqua'
            # plt.plot(self.x, self.y, color=color)
            # plt.xlim(-10, 25)
            # plt.ylim(-10, 25)
            # plt.gca().set_aspect('equal', adjustable='box')
            # plt.grid()
            # plt.show()

            # self.measure_count = 0
            # self.x = np.array([])
            # self.s_x = np.array([])
            # self.y = np.array([])

            pass

            

        if self.positionation_up:

            # if self.measure_count > 5:
            #     plt.ion()   
            #     plt.plot(self.linspace, self.x)
            #     plt.plot(self.linspace, self.s_x, color='red')
            #     plt.pause(0.0001)
            #     plt.show()
            #     time.sleep(0.05)
            #     plt.close()

            orient = self.position()

            #save  xy for plots in xy plane
            # self.x = np.append(self.x, orient.x)
            # self.y = np.append(self.y, orient.y)

            if self.filter:
                # self.x = np.append(self.x, orient.x)
                # self.y = np.append(self.y, orient.y)
                ## self.angle = np.append(self.angle, orient.angle)

                self.measure_count += 1
                self.linspace = np.linspace(0, self.measure_count, self.measure_count)

                window = 5
                polynom = 3

                if self.measure_count < 2:
                    pass
                else:
                    if self.measure_count < 50:
                        window = 5
                    else:
                        window = self.measure_count / 10
                        if window % 2 == 0: window += 1

                self.s_x = self.savitzky_golay(self.x, window, polynom)
                self.orient.x = self.s_x[self.measure_count - 1]
                self.orient.y = self.savitzky_golay(self.y, window, polynom)[self.measure_count - 1]
                #self.orient.angle = self.savitzky_golay(self.angle, window, polynom)[self.measure_count - 1]
                self.orient.angle = orient.angle
            else:
                self.orient.x = orient.x
                self.orient.y = orient.y
                self.orient.angle = orient.angle
        else:
            orient = self.orient
            #get pos and send it to curr orient
            self.model.set_vel(data.linear, data.angular, self.latch_time - self.exec_time)
            new_pos = self.model.get_position()
            self.orient = RobotOrient()
            self.orient.x = new_pos[0]
            self.orient.y = -new_pos[1]
            self.orient.angle = -new_pos[2]
        
        self.publish("CURR_ORIENT", self.orient)

    def savitzky_golay(self, y, window_size, order, deriv=0, rate=1):
        """Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
        The Savitzky-Golay filter removes high frequency noise from data.
        It has the advantage of preserving the original shape and
        features of the signal better than other types of filtering
        approaches, such as moving averages techniques.
        Parameters
        ----------
        y : array_like, shape (N,)
            the values of the time history of the signal.
        window_size : int
            the length of the window. Must be an odd integer number.
        order : int
            the order of the polynomial used in the filtering.
            Must be less then `window_size` - 1.
        deriv: int
            the order of the derivative to compute (default = 0 means only smoothing)
        Returns
        -------
        ys : ndarray, shape (N)
            the smoothed signal (or it's n-th derivative).
        Notes
        -----
        The Savitzky-Golay is a type of low-pass filter, particularly
        suited for smoothing noisy data. The main idea behind this
        approach is to make for each point a least-square fit with a
        polynomial of high order over a odd-sized window centered at
        the point.
        Examples
        --------
        t = np.linspace(-4, 4, 500)
        y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
        ysg = savitzky_golay(y, window_size=31, order=4)
        import matplotlib.pyplot as plt
        plt.plot(t, y, label='Noisy signal')
        plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
        plt.plot(t, ysg, 'r', label='Filtered signal')
        plt.legend()
        plt.show()
        References
        ----------
        .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
        Data by Simplified Least Squares Procedures. Analytical
        Chemistry, 1964, 36 (8), pp 1627-1639.
        .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
        W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
        Cambridge University Press ISBN-13: 9780521880688
        """
        
        
        try:
            window_size = np.abs(np.int(window_size))
            order = np.abs(np.int(order))
        except ValueError, msg:
            raise ValueError("window_size and order have to be of type int")
        if window_size % 2 != 1 or window_size < 1:
            raise TypeError("window_size size must be a positive odd number")
        if window_size < order + 2:
            raise TypeError("window_size is too small for the polynomials order")
        order_range = range(order+1)
        half_window = (window_size -1) // 2
        # precompute coefficients
        b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
        m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
        # pad the signal at the extremes with
        # values taken from the signal itself
        firstvals = y[0] - np.abs(y[1:half_window+1][::-1] - y[0])
        lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
        y = np.concatenate((firstvals, y, lastvals))
        return np.convolve(m[::-1], y, mode='valid')
###

# Navigator()

# rospy.spin()