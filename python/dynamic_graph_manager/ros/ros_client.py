#!/usr/bin/env python

"""@package dynamic_graph_manager

@file
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.

@brief This python module defines some very useful tools, notably a ROS node
that is a client to the Dynamic Graph Python Interpreter

"""

# Python 3 compatibility. It has to be the first import.
from __future__ import print_function, division

import os
import rospy
from dynamic_graph_manager.srv import RunCommand, RunPythonFile

import atexit


class RosPythonInterpreter(object):

    def __init__(self):
        # Create a client for the single python command service of the
        # dynamic_graph_manager
        self.run_command_service_name = '/dynamic_graph/run_python_command'
        self.command_client = self._connect_to_rosservice_run_python_command()

        # Create a client for the python script reading service of the
        # dynamic_graph_manager
        self.run_script_service_name = '/dynamic_graph/run_python_script'
        self.script_client = self._connect_to_rosservice_run_python_script()

        # Timeout during the connection to the rosservice.
        self.timeout_connection = 2.0

        atexit.register(self._atexit)

    def run_python_command(self, code_string):
        """
        Call the rosservice of the current running dynamic graph manager
        """
        return_string = ""

        if code_string == "" or code_string == "\n":
            return return_string

        try:
            if not self.command_client:
                return_string += "Connection to remote server lost."
                " Reconnecting..."
                self._connect_to_rosservice_run_python_command(
                    self.timeout_connection)
                return return_string

            response = self.command_client(code_string)

            if not response:
                return_string += "Error while parsing command."
            else:
                if response.standard_output != "":
                    return_string += response.standard_output[:]
                if response.standard_error != "":
                    return_string += response.standard_error[:]
                elif response.result != "None":
                    return_string += response.result[:]
        except rospy.ServiceException as ex:
            return_string += str(ex) + ""
            return_string += "Connection to remote server lost. Reconnecting..."
            self.command_client = self._connect_to_rosservice_run_python_command(
                self.timeout_connection)

        return return_string

    def run_python_script(self, filename):
        """
        Call the rosservice of the current running dynamic graph manager
        """
        if not os.path.isfile(filename):
            print("Provided file does not exist: %s" % (filename))
            return

        try:
            if not self.script_client:
                print("Connection to remote server lost. Reconnecting...")
                self._connect_to_rosservice_run_python_script(
                    self.timeout_connection)
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
            self.script_client = self._connect_to_rosservice_run_python_script(
                self.timeout_connection)

    @staticmethod
    def _connect_to_rosservice(service_name, rosservice, timeout):
        try:
            rospy.loginfo('Waiting for service' + service_name + '...')
            rospy.wait_for_service(service_name, timeout=timeout)
            client = rospy.ServiceProxy(
                service_name, rosservice, persistent=False)
            if client:
                rospy.loginfo("Successfully connected to " + service_name)
                return client
            else:
                return None
        except:
            raise RuntimeError(service_name + " not available.")

    def _connect_to_rosservice_run_python_command(self, timeout=None):
        return RosPythonInterpreter._connect_to_rosservice(
            self.run_command_service_name, RunCommand, timeout)

    def _connect_to_rosservice_run_python_script(self, timeout=None):
        return RosPythonInterpreter._connect_to_rosservice(
            self.run_script_service_name, RunPythonFile, timeout)

    def _atexit(self):
        """
        Execute a couple of instruction upon exit
        """
        # if is_readline_import:
        #     readline.write_history_file(self.python_history)

        if self.command_client:
            self.command_client.close()

        if self.script_client:
            self.script_client .close()
