"""@package dynamic_graph_manager

@file prologue.py
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-05-22

@brief This file prepares the python interpretor so it contains a pointer to the
Device and some tracers

"""

from dynamic_graph.entity import PyEntityFactoryClass
from .robot import Robot

# Get the declaration of the Device class.
DeviceClass = PyEntityFactoryClass('Device')

# Get the name of the device from the configuration file.
import rospy
device_name = rospy.get_param("/dynamic_graph/device_name")

# Create the robot using the device.
device_cpp_object = DeviceClass(device_name)

# We create a small class that will own the device object and a real time tracer.
robot = Robot(name = device_name, device = device_cpp_object)

__all__ = ["robot"]

####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
