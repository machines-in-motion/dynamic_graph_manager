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

from dynamic_graph import plug
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace
from dynamic_graph_manager.ros import Ros

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
        self.initialize_tracer()

        # We trace by default all signals of the device.
        self.tracedSignals = {'device': []}
        self.device_signals_names = []
        for signal in self.device.signals():
          signal_name = signal.name.split('::')[-1]
          self.tracedSignals['device'].append(signal_name)
          self.device_signals_names.append(signal_name)

        # Device
        for s in self.tracedSignals['device']:
            self.add_trace(self.device.name, s)

        # Prepare potential ros import/export
        self.ros = Ros(self)
        self.device.after.addDownsampledSignal('rosPublish.trigger',1);
        self.export_device_dg_to_ros()

    def __del__(self):
        if self.tracer:
            self.stop_tracer()

    def add_trace(self, entityName, signalName):
        if self.tracer:
            self.autoRecomputedSignals.append(
                '{0}.{1}'.format(entityName, signalName))
            addTrace(self, self.tracer, entityName, signalName)

    def initialize_tracer(self):
        if not self.tracer:
            self.tracer = TracerRealTime('trace')
            self.tracer.setBufferSize(self.tracerSize)
            self.tracer.open('/tmp/','dg_','.dat')
            # Recompute trace.triger at each iteration to enable tracing.
            self.device.after.addSignal('{0}.triger'.format(self.tracer.name))

    def start_tracer(self):
        """
        Start the tracer if it has not already been stopped.
        """
        if self.tracer:
            self.tracer.start()

    def stop_tracer(self):
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

    def export_device_dg_to_ros(self):
        """
        Import in ROS the signal from the dynamic graph device.
        """
        for sig_name in self.device_signals_names:
            # arguments: type of data, signal name, rostopic name where to publish
            self.ros.rosPublish.add ("vector", sig_name,
                                       "/dynamic_graph/device/" + sig_name)
            plug(self.device.signal(sig_name),
                 self.ros.rosPublish.signal(sig_name))

        #self.ros.rosPublish.displaySignals()

__all__ = ["Robot"]
