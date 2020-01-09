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

# Python 3 compatibility. It has to be the first import.
from __future__ import print_function, division

# Standard import.
import sys
import os

# Used to compile the instruction given in the python terminal
import code
from dynamic_graph_manager.ros.dgcompleter import DGCompleter

# Used to connect to ROS services
import rospy
from dynamic_graph_manager.srv import RunCommand, RunPythonFile

# Used to deal with the python history
# try:
#     import readline
#     is_readline_import = True
# except ImportError:
#     print("Module readline not available.")
#     is_readline_import = False

import atexit


class RosShell(code.InteractiveConsole):
    """
    For the subtilities please read https://docs.python.org/3/library/code.html
    """
    def __init__(self):
        
        # Create the python terminal
        code.InteractiveConsole.__init__(self)

        # Command lines from the terminal.
        self.lined_pushed = ""

        # Create a client for the single python command service of the
        # dynamic_graph_manager
        self.run_command_service_name = '/dynamic_graph/run_python_command'
        self.command_client = self.connect_to_ros_service_run_python_command()

        # Create a client for the python script reading service of the
        # dynamic_graph_manager
        self.run_script_service_name = '/dynamic_graph/run_python_script'
        self.script_client = self.connect_to_ros_service_run_python_script()

        # Initialize the python completion
        # self.completer = DGCompleter(self)
        # readline.set_completer(self.completer.complete)
        # readline.parse_and_bind("tab: complete")

        # Read the existing history if there is one
        # Save the python command history.
        # self.python_history = os.path.join(os.environ["HOME"], ".dynamic_gragh_manager_client_python_history")
        # print("The python history of command is located in: ", self.python_history)
        # if os.path.exists(self.python_history):
        #     readline.read_history_file(self.python_history)
        # # Set maximum number of items that will be written to the history file
        # readline.set_history_length(300)
        
        atexit.register(self.atexit)

    def atexit(self):
        """
        Execute a couple of instruction upon exit
        """
        # if is_readline_import:
        #     readline.write_history_file(self.python_history)

        if self.command_client:
            self.command_client.close()
        
        if self.script_client:
            self.script_client .close()
    
    @staticmethod
    def connect_to_ros_service(service_name, ros_service, timeout):
        try:
            rospy.loginfo('waiting for service' + service_name + '...')
            rospy.wait_for_service(service_name, timeout=timeout)
            client = rospy.ServiceProxy(service_name, ros_service, persistent=False)
            if client:
                rospy.loginfo("Successfully connected to " + service_name)
                return client
            else:
                return None
        except:
            raise RuntimeError(service_name + " not available.")

    def connect_to_ros_service_run_python_command(self, timeout=None):
        return RosShell.connect_to_ros_service(self.run_command_service_name, RunCommand, timeout)

    def connect_to_ros_service_run_python_script(self, timeout=None):
        return RosShell.connect_to_ros_service(self.run_script_service_name, RunPythonFile, timeout)

    def rosservice_call_run_python_command(self, code_string):
        """
        Call the rosservice of the current running dynamic graph manager
        """
        print (code_string)
        if code_string == "" or code_string == "\n":
            return

        try:
            if not self.command_client:
                print("Connection to remote server lost. Reconnecting...")
                self.connect_to_ros_service_run_python_command(2.0)
                return

            response = self.command_client(code_string)
            
            if not response:
                print("Error while parsing command.")
            else:
                if response.standard_output != "":
                    print (response.standard_output[:-1])
                if response.standard_error != "":
                    print (response.standard_error[:-1])
                elif response.result != "None":
                    print (response.result)
        except rospy.ServiceException as ex:
            print(ex)
            print("Connection to remote server lost. Reconnecting...")
            self.command_client = self.connect_to_ros_service_run_python_command(2.0)
    
    def rosservice_call_run_python_script(self, filename):
        """
        Call the rosservice of the current running dynamic graph manager
        """
        if not os.path.isfile(filename):
            print("Provided file does not exist: %s"%(filename))
            return

        try:
            if not self.script_client:
                print("Connection to remote server lost. Reconnecting...")
                self.connect_to_ros_service_run_python_script(2.0)
                return

            print("Executing script at: " + filename)
            response = self.script_client(os.path.abspath(filename))
            
            if not response:
                print("Error while parsing file.")
            else:
                if response.standard_error:
                    print(response.standard_error)

        except rospy.ServiceException as ex:
            print(ex)
            print("Connection to remote server lost. Reconnecting...")
            self.script_client = self.connect_to_ros_service_run_python_script(2.0)

    def runcode(self, code):
        """
        Inherited from code.InteractiveConsole

        We execute the code pushed in the cache `self.lined_pushed`. The code is
        pushed whenever the user press enter during the interactive session.
        see https://docs.python.org/3/library/code.html
        """
        try:
            # we copy the line in a tmp var
            code_string = self.lined_pushed[:-1] 
            self.rosservice_call_run_python_command(code_string)

            # we reset thec cache here
            self.lined_pushed = ""
        except Exception as e:
            print(e)

    def runsource(self, source, filename = '<input>', symbol = 'single'):
        """
        Inherited from code.InteractiveConsole

        see https://docs.python.org/3/library/code.html
        """
        try:
            c = code.compile_command(source, filename, symbol)
            if c:
                return self.runcode(c)
            else:
                return True
        except SyntaxError, OverflowError:
            # self.showsyntaxerror()
            # self.lined_pushed = ""
            return False

    def push(self, line):
        """
        Upon pressing enter in the interactove shell the user "push" a string.
        This method is then called with the string pushed.
        We catch thte string to send it via the rosservice.
        """
        self.lined_pushed += line + "\n"
        return code.InteractiveConsole.push(self, line)