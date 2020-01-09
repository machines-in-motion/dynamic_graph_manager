#!/usr/bin/env python

"""@package dynamic_graph_manager

@file run_command.py
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-05-22

@brief This python module defines some very useful tools, notably a ROS node
that is a client to the Dynamic Graph Python Terminal

"""

import sys
import signal
import rospy
import optparse
import os.path

from dynamic_graph_manager.ros.shell_client import RosShell


def signal_handler(sig, frame):
    """
    Catch Ctrl+C and quit.
    """
    print('')
    print('You pressed Ctrl+C! Closing ros client and shell.')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    """
    Start a shell that communicate with the current instance of the
    dynamic_graph_manager.
    """    
    rospy.init_node('run_command', argv=sys.argv)
    sys.argv = rospy.myargv(argv=None)
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]')
    (options, args) = parser.parse_args(sys.argv[1:])

    sh = RosShell()

    if args[:]:
        infile = args[0]
        sh.rosservice_call_run_python_script(infile)

    sh.interact("Interacting with remote server.")
