"""@package dynamic_graph_manager

@file prologue.py
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-05-22

@brief This file prepares the python interpretor so it contains a pointer to the
Device and some tracers

"""

import shared_memory
from dynamic_graph_manager.dynamic_graph.device import Device
from .robot import Robot

# Get the name of the device from the shared_memory
shared_memory_name = shared_memory.get_string("dgm_shm_name", "shared_memory_name")
device_name = shared_memory.get_string(shared_memory_name, "device_name")

# Create the robot using the device.
device_cpp_object = Device(device_name)

# We create a small class that will own the device object and a real time tracer.
robot = Robot(name=device_name, device=device_cpp_object)

__all__ = ["robot"]

####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
