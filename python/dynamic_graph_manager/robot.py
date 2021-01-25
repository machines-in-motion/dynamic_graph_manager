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

from dynamic_graph import plug
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace
from dynamic_graph_manager.ros.ros import Ros
from dynamic_graph.entity import Entity

# Internal helper tool.
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)


class Robot(object):
    """
    This class instantiates a robot
    """

    init_pos = 0.0
    init_vel = 0.0
    init_acc = 0.0

    """
    Tracer used to log data.
    """
    tracer = None

    """
    How much data will be logged.
    """
    tracerSize = 2 ** 22

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

    def __init__(self, name, device=None, tracer=None):
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
        self.stop_tracer()

    def add_trace(self, entityName, signalName):
        if self.tracer:
            addTrace(self, self.tracer, entityName, signalName)

    def get_new_tracer_log_dir(self):
        import os
        import os.path
        import time

        log_dir = os.path.join(
            os.path.expanduser("~"),
            "dynamic_graph_manager",
            time.strftime("%Y-%m-%d_%H-%M-%S"),
        )

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        return log_dir

    def initialize_tracer(self):
        """
        Initialize the tracer and by default dump the files in
         ~/dynamic_graph/[date_time]/
        """
        if self.tracer is None:
            self.tracer_log_dir = self.get_new_tracer_log_dir()
            self.tracer = TracerRealTime("trace")
            self.tracer.setBufferSize(self.tracerSize)
            self.tracer.open(self.tracer_log_dir, "dg_", ".dat")

        # Recompute trace.triger at each iteration to enable tracing.
        self.device.after.addSignal("{0}.triger".format(self.tracer.name))

    def start_tracer(self):
        """
        Start the tracer if it has not already been stopped.
        """
        if self.tracer is not None:
            self.tracer.start()

    def stop_tracer(self):
        """
        Stop and destroy tracer.
        """
        if self.tracer is not None:
            self.tracer.dump()
            self.tracer.stop()
            self.tracer.close()
            print("Stored trace in:", self.tracer_log_dir)

            # NOTE: Not calling self.tracer.clear() here, as by default the
            # tracer should keep it's traced signals attached.

            # Null the tracer object, such that initialize_tracer() will reopen it.
            self.tracer = None

            self.initialize_tracer()

    def add_to_ros(
        self, entityName, signalName, topic_name=None, topic_type=None
    ):
        """
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
        self.add_trace(entityName, signalName)
        self.add_to_ros(
            entityName,
            signalName,
            topic_name=topic_name,
            topic_type=topic_type,
        )

    def export_device_dg_to_ros(self):
        """
        Import in ROS the signal from the dynamic graph device.
        """
        for sig_name in self.device_signals_names:
            self.add_to_ros(self.device.name, sig_name)


__all__ = ["Robot"]
