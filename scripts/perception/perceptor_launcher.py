#!/usr/bin/env python

from perceptor import *

pf = PerceptorFactory()
if pf.is_sim:
    endpoint = TCP4ServerEndpoint(reactor, pf.port)
else:
    endpoint = TCP4ServerEndpoint(reactor, 8256)
endpoint.listen(pf)

reactor.run()