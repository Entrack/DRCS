#!/usr/bin/env python

from positionator import *

p = Positionator()

if p.is_sim:
    rospy.spin()
else:
    reactor.run()