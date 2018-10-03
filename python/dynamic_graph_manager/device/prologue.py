#
# @file prologue.py
# @brief The robot entity in python
# @author Maximilien Naveau
# @date 2018
#
# This file prepares the pyhton interpretor so ite contains a pointer to the
# Device and some tracers
#
from dynamic_graph.entity import PyEntityFactoryClass
from .robot import Robot

# Get the declaration of the Device class.
DeviceClass = PyEntityFactoryClass('Device')

# Get the name of the device from the configuration file.
import rospy
device_name = rospy.get_param("/hardware_communication/device_name")

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
