"""Dynamic model of disturbance
"""

from math import sin
from time import time
from numpy import matrix
from random import random


class Disturbance:
    """Dynamic model of harmonic disturbance

    The value of disturbance depends on control command and affects to
    linear and angular velocity

    Examples:

    >>> Disturbance(0.1)
    Disturbance(0.1)
    """

    def __init__(self, amplitude):
        """Init disturbance params

        Args:
             amplitude (float): amplitude of harmonic disturbance
        """
        self._amplitude = amplitude
        self._frequency = 0.5 + random()

    def __repr__(self):
        return 'Disturbance({0})'.format(self._amplitude)

    def get(self, U):
        """Return the vector of disturbance

        The vector of disturbance depend on control vector and sin function

        Args:
            U (matrix): control vector

        Returns:
            (matrix): disturbance vector
        """
        if self._amplitude == 0:
            M = matrix([[0.],
                        [0.]])
        else:
            a = self._amplitude
            t = self._frequency * time()
            D = matrix([[a * (1. + sin(t)) / 2., 0.],
                        [10. * a * sin(t)      , a * sin(t)]])
            M = D * U
        return M

