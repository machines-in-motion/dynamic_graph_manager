# Plan for making the control process re-launchable

Author: Julian Viereck
Date: 26 Jan 2020

## Objective

The goal of this work is to add the ability to relaunch the dynamic graph control process while keeping the motor process running. Especially, this includes:
- (re)launching a new control processs
- read values from the new control

Idea:
- have per control-process shared memory area
- when launching a new control process, the cond_var_ and shared memory area get updated

Required work:
- change the type of the shared memory entities const static to changeable / per class object
- add a "relaunch" method on the dynamic graph process / hardware process part
- have ability to put the hardware process from safe-mode back into runable mode
- add ros command to spawn new control process


```
// get the hardware communication ros node handle
ros::NodeHandle& ros_node_handle = dynamic_graph::ros_init(
    dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);
/** initialize the user commands */
ros_user_commands_.push_back(ros_node_handle.advertiseService(
    "set_a_boolean", &SimpleDGM::user_command_callback, this));
```
