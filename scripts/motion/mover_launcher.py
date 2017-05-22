#!/usr/bin/env python

from mover import *

mf = MoverFactory()
if mf.is_sim:
    endpoint = TCP4ServerEndpoint(reactor, mf.port + 256)
else:
    endpoint = TCP4ServerEndpoint(reactor, 8256 + 256)
endpoint.listen(mf)

reactor.run()