"""@package dynamic_graph_manager

@file robot.py
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-05-22

@brief This file Defines a robot class containing all the utilities on the
robot: ros export/import logging utilities...

"""

from __future__ import print_function

import time
from copy import deepcopy
from pathlib import Path

from dynamic_graph import plug
from dynamic_graph.entity import Entity
from dynamic_graph.tools import addTrace
from dynamic_graph.tracer_real_time import TracerRealTime

from dynamic_graph_manager.ros.ros import Ros


class Robot(object):
    """This class instantiates a robot."""

    # Tracer used to log data.
    tracer = None
    # How much data will be logged.
    tracerSize = 2 ** 22

    # Automatically recomputed signals through the use
    # of device.after.
    # This list is maintained in order to clean the
    # signal list device.after before exiting.
    autoRecomputedSignals = []

    def __init__(self, name, device=None, tracer=None):
        """Create the robot."""
        self.name = name
        self.device = device
        # Initialize tracer if necessary.
        self.tracer = tracer
        self.initialize_tracer()

        # We trace by default all signals of the device.
        self.device_signals_names = []
        for signal in self.device.signals():
            signal_name = signal.name.split("::")[-1]
            self.add_trace(self.device.name, signal_name)
            self.device_signals_names.append(signal_name)

        # Prepare potential ros import/export
        self.ros = Ros(self.device)
        # self.export_device_dg_to_ros()

    def __del__(self):
        """By default dump the data."""
        self.stop_tracer()

    def add_trace(self, entityName, signalName):
        """Register a signal for logging."""
        if self.tracer:
            addTrace(self, self.tracer, entityName, signalName)

    def get_new_tracer_log_dir(self):
        """Create a log directory name."""
        log_dir = (
            Path.home() /
            "dynamic_graph_manager" /
            deepcopy(time.strftime("%Y-%m-%d_%H-%M-%S"))
        )
        log_dir.mkdir(parents=True, exist_ok=True)
        return str(log_dir)

    def initialize_tracer(self):
        """Initialize the tracer.

        By default dump the files in ~/dynamic_graph/[date_time]/
        """
        if self.tracer is None:
            self.tracer_log_dir = self.get_new_tracer_log_dir()
            self.tracer = TracerRealTime("rt_trace")
            self.tracer.setBufferSize(self.tracerSize)
            self.tracer.close()
            # Recompute trace.triger at each iteration to enable tracing.
            self.device.after.addSignal("{0}.triger".format(self.tracer.name))
        else:
            self.tracer_log_dir = self.get_new_tracer_log_dir()
            self.tracer.close()
            self.tracer.open(self.tracer_log_dir, "dg_", ".dat")
        print("Storing trace in:", self.tracer_log_dir)

    def start_tracer(self):
        """Start the tracer if it has not already been stopped."""
        self.initialize_tracer()
        self.tracer.start()

    def stop_tracer(self):
        """Stop and destroy tracer."""
        if self.tracer is not None:
            self.tracer.stop()
            self.tracer.dump()
            self.tracer.close()
            print("Stored trace in:", self.tracer_log_dir)

    def add_to_ros(
        self, entityName, signalName, topic_name=None, topic_type=None
    ):
        """Publish signals into ROS topics.

        arg: topic_type is a string among:
              ['double', 'matrix', 'vector', 'vector3', 'vector3Stamped',
              'matrixHomo', 'matrixHomoStamped', 'twist', 'twistStamped',
              'joint_states'].
             Each different strings correspond to a ros message. For rviz
             support please use joint_states which correspond to the joint
             states including the potential free flyer joint.
        """
        # Lookup the entity's signal by name
        signal = Entity.entities[entityName].signal(signalName)

        if topic_name is None:
            topic_name = "/dg__" + entityName + "__" + signalName
            new_signal_name = "dg__" + entityName + "__" + signalName
        if topic_type is None:
            topic_type = "vector"

        self.ros.ros_publish.add(topic_type, new_signal_name, topic_name)
        plug(signal, self.ros.ros_publish.signal(new_signal_name))

    def add_ros_and_trace(
        self, entityName, signalName, topic_name=None, topic_type=None
    ):
        """Publish signal to ROS and log them locally."""
        self.add_trace(entityName, signalName)
        self.add_to_ros(
            entityName,
            signalName,
            topic_name=topic_name,
            topic_type=topic_type,
        )

    def export_device_dg_to_ros(self):
        """Import in ROS the signal from the dynamic graph device."""
        for sig_name in self.device_signals_names:
            self.add_to_ros(self.device.name, sig_name)


__all__ = ['Robot']
