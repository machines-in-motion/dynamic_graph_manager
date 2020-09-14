"""@package dynamic_graph_manager

@file ros.py
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-05-22

@brief This file defines a class that create ros entities for the interaction
between ROS and the dynamic graph

"""

from dynamic_graph import plug
# from dynamic_graph_manager.dynamic_graph.ros_publish import RosPublish
# from dynamic_graph_manager.dynamic_graph.ros_subscribe import RosSubscribe
# from dynamic_graph_manager.dynamic_graph.ros_time import RosTime
# from dynamic_graph_manager.dynamic_graph.ros_robot_state_publisher import (
#     RosRobotStatePublisher,
# )


class Ros(object):
    def __init__(self, device, suffix=""):
        self.device = device
        # self.rosPublish = RosPublish("rosPublish{0}".format(suffix))
        # self.rosSubscribe = RosSubscribe("rosSubscribe{0}".format(suffix))
        # self.rosTime = RosTime("rosTime{0}".format(suffix))
        # self.rosRobotStatePublisher = RosRobotStatePublisher(
        #     "rosRobotStatePublisher{0}".format(suffix)
        # )

        # # aliases, for retro compatibility
        # self.rosImport = self.rosPublish
        # self.rosExport = self.rosSubscribe

        # # make sure that the publishing is done by plugging the refresh
        # # (trigger) to the device periodic call system
        # self.device.after.addDownsampledSignal(
        #     self.rosRobotStatePublisher.name + ".trigger", 1
        # )
        # self.device.after.addDownsampledSignal(self.rosPublish.name + ".trigger", 1)
        # # not needed for self.rosSubscribe and self.rosSubscribe as they get
        # # the information from ROS, they do not have output signals
