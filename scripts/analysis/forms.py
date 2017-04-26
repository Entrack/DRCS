# absctract classes
from abc import ABCMeta, abstractmethod
# vector
from vector import Vector
# Math
import math
# min in dictionary
import operator

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

# Template for forms
class Form():
    __metaclass__=ABCMeta

    @abstractmethod
    def get_formation_positions(self, units):
        debug_print_function(self)
        # units -- id - int : pos - Vector

    def mass_center(self, units):
        center = Vector()
        for _id in units:
            center = center.__add__(units[_id])
            # debug_print(center)
        center = center.__mul__(1.0 / len(units))
        return center
###

# Circle Form
class Circle(Form):
    def __init__(self):
        debug_print_function(self)

    def get_formation_positions(self, units, robot_diameter, opt_distance):
        # debug_print_function(self)
        
        # count keys
        count = len(units.keys())
        # debug_print(count)

        if count == 0:
            single_formation = {}
            for _id in units:
                single_formation[_id] = self.mass_center(units)
            return single_formation
        # opt_distance => radius multiplier
        angle = 2 * math.pi / count
        # debug_print(angle)
        zero = Vector(1, 0)
        one = Vector(math.cos(angle), math.sin(angle))
        distance = one.__sub__(zero).norm()
        # debug_print(distance)
        muiltiplier = (opt_distance + robot_diameter) / distance
        # debug_print("muiltiplier")
        # debug_print(muiltiplier)
        # find local circle coords
        local_points = []
        for i in range(1, count + 1):
            local_points.append(Vector(muiltiplier * math.cos(angle * i), muiltiplier *  math.sin(angle * i)))
        # debug_print("local_points")
        # debug_print(local_points)
        # transform to global
        mass_center = self.mass_center(units)
        global_points = []
        for point in local_points:
            global_points.append(Vector(point.x() + mass_center.x(), point.y() + mass_center.y()))
        # debug_print("global_points")
        # debug_print(global_points)
        
        # fill
        # id - int : pos - Vector
        formation = {}

        distances = {}
        for point in global_points:
            # debug_print(point)
            for _id in units:
                # debug_print(_id)
                if not formation.has_key(_id):
                    # debug_print("norm")
                    distances[_id] = point.__sub__(units[_id]).norm()
            min_dist_id = min(distances.iteritems(), key=operator.itemgetter(1))[0]
            # debug_print("min_dist_id -- " + str(min_dist_id))
            # debug_print(distances)
            distances.clear()
            formation[min_dist_id] = point

        # debug_print("formation")
        # debug_print(formation)
        return formation
###

# c = Circle()
# units = {'1':Vector(40, 0), '2':Vector(0,20), '3':Vector(-20, 0), '4':Vector(0,-20)}
# formation = c.get_formation_positions(units, 4, 2)