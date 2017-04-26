from math import pi

_2PI = 2. * pi

def limitation(value, limit):
    if limit is None:
      return value
    if value > limit:
        value = limit
    elif value < -limit:
        value = -limit
    return value

def wrap_to_pi(value):
    while value > pi:
        value -= _2PI
    while value < -pi:
        value += _2PI
    return value

