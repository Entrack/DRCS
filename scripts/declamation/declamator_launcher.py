#!/usr/bin/env python

from declamator import *

factory = DeclamatorFactory()
rospy.sleep(factory.latch_time)
factory.loop_network_scaning()

reactor.run()