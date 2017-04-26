"""Dynamic model of robot with differential drive

TODO:
    * limitation of linear and angular velocity depend on each other
"""

from math import sin, cos
from random import gauss
from numpy import matrix

from disturbance import Disturbance
from utils import limitation, wrap_to_pi


class Model:
    """"Dynamic model of robot with differential drive

    Class provide calculation or robot position depend on itinial position
    and control commands. Control commands are linear and angular velocity

    Examples:

    >>> model = Model((2.0, 1.0, 0.0))
    >>> model
    Model(2.0, 1.0, 0.0)
    >>> model.set_vel(0.1, 1.0, 0.2)
    >>> model
    Model(2.02, 1.0, 0.2)
    >>> model.get_position()
    (2.02, 1.0, 0.2)
    """

    def __init__(self, X=(0., 0., 0.), disturbance=0):
        """Initial position and disturbance value

        Args:
            X (tuple): Initial robot position (default (0.,0.,0.))
            disturbance (float): Disturbance amplitude (default 0)
        """

        self._X = matrix([[X[0]],
                          [X[1]],
                          [X[2]]])

        self._linean_vel_max = None
        self._angular_vel_max = None
        self._disturbance = Disturbance(disturbance)

    def __repr__(self):
        return 'Model({0}, {1}, {2})'.format(*[x.item() for x in self._X])

    def set_position(self, X):
        """Set position of robot

        Args:
            X (tuple): Robot position
        """
        self._X = matrix([[X[0]],
                          [X[1]],
                          [X[2]]])

    def set_vel_max(self, linean_vel_max, angular_vel_max):
        """Set maximum available linear and angular velocity

        Args:
            linear_vel_max (float) : maximum linear velocity
            angular_vel_max (float): maximum angular velocity
        """
        self._linean_vel_max = linean_vel_max
        self._angular_vel_max = angular_vel_max

    def set_vel(self, linear_vel_cmd, angular_vel_cmd, dt):
        """Calculate robot position

        Calculate robot position using equation of dynamic model
        dot X = F(X) * (U - M),
        where X is state vector, F(X) is matrix function, U is control vector,
        M is disturbance vector

        Args:
            linear_vel_cmd (float):  linear velocity command
            angular_vel_cmd (floar): angular velocity command
            dt (float):              time step
        """

        linear_vel_cmd = limitation(linear_vel_cmd, self._linean_vel_max)
        angular_vel_cmd = limitation(angular_vel_cmd, self._angular_vel_max)

        F = matrix([[cos(self._X.item(2)), 0.],
                    [sin(self._X.item(2)), 0.],
                    [                  0., 1.]])

        U = matrix([[linear_vel_cmd],
                    [angular_vel_cmd]])

        M = self._disturbance.get(U)

        # Calculate robot position
        self._X = self._X + F * (U - M) * dt
        self._X[2] = wrap_to_pi(self._X.item(2))

    def get_position(self, noise=0.):
        """ Return position of robot

        Args:
            noise (float): gauss noise for return values (default 0.)

        Returns:
            (tuple): Robot position
        """
        return self._X.item(0) + gauss(0., noise),\
               self._X.item(1) + gauss(0., noise),\
               self._X.item(2) + gauss(0., noise)

""""Dynamic model of robot with differential drive

    Class provide calculation or robot position depend on itinial position
    and control commands. Control commands are linear and angular velocity

    Examples:

    >>> model = Model((2.0, 1.0, 0.0))
    >>> model
    Model(2.0, 1.0, 0.0)
    >>> model.set_vel(0.1, 1.0, 0.2)
    >>> model
    Model(2.02, 1.0, 0.2)
    >>> model.get_position()
    (2.02, 1.0, 0.2)
    """
# model = Model((0.0, 0.0, 0.0))
# model.set_vel(1.0, 0.0, 2.0)
# print model.get_position()