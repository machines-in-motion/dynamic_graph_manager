# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

import numpy as np

from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace

# Internal helper tool.
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

class Robot(object):
    """
    This class instantiates a Hrp2 robot
    """

    init_pos = (0.0)
    init_vel = (0.0)
    init_acc = (0.0)

    """
    Tracer used to log data.
    """
    tracer = None

    """
    How much data will be logged.
    """
    tracerSize = 2**20

    """
    Automatically recomputed signals through the use
    of device.after.
    This list is maintained in order to clean the
    signal list device.after before exiting.
    """
    autoRecomputedSignals = []

    """
    Robot timestep
    """
    timeStep = 0.005

    def __init__(self, name, device = None, tracer = None):
        self.name = name
        self.device = device
        # Initialize tracer if necessary.
        if tracer:
            self.tracer = tracer
        self.initializeTracer()

        # Which signals should be traced.
        self.tracedSignals = {'device': []}
        for signal in self.device.signals():
          signal_name = signal.name.split('::')[-1]
          print(signal_name)
          self.tracedSignals['device'].append(signal_name)

        # Device
        for s in self.tracedSignals['device']:
            self.addTrace(self.device.name, s)

    def __del__(self):
        if self.tracer:
            self.stopTracer()

    def addTrace(self, entityName, signalName):
        if self.tracer:
            self.autoRecomputedSignals.append(
                '{0}.{1}'.format(entityName, signalName))
            addTrace(self, self.tracer, entityName, signalName)

    def initializeTracer(self):
        if not self.tracer:
            self.tracer = TracerRealTime('trace')
            self.tracer.setBufferSize(self.tracerSize)
            self.tracer.open('/tmp/','dg_','.dat')
            # Recompute trace.triger at each iteration to enable tracing.
            self.device.after.addSignal('{0}.triger'.format(self.tracer.name))

    def startTracer(self):
        """
        Start the tracer if it does not already been stopped.
        """
        if self.tracer:
            self.tracer.start()

    def stopTracer(self):
        """
        Stop and destroy tracer.
        """
        if self.tracer:
            self.tracer.dump()
            self.tracer.stop()
            self.tracer.close()
            self.tracer.clear()
            for s in self.autoRecomputedSignals:
                self.device.after.rmSignal(s)
            self.tracer = None


__all__ = ["Robot"]
