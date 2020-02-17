#!/usr/bin/env python

"""@package dynamic_graph_manager

@file
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.

@brief This python module defines some very useful tools, notably a ROS node
that is a client to the Dynamic Graph Python Terminal

"""

# Python 3 compatibility. It has to be the first import.
from __future__ import print_function, division

# Standard import.
import os

# Used to compile the instruction given in the python terminal
import code

# Used to connect to ROS services
from dynamic_graph_manager.ros.ros_client import RosPythonInterpreter


class DynamicGraphInteractiveConsole(code.InteractiveConsole):
    """
    For the subtilities please read https://docs.python.org/3/library/code.html
    """

    def __init__(self):

        # Create the python terminal
        code.InteractiveConsole.__init__(self)

        # Command lines from the terminal.
        self.lined_pushed = ""

        self.ros_python_interpreter = RosPythonInterpreter()

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
            self.ros_python_interpreter.run_python_command(code_string)

            # we reset the cache here
            self.lined_pushed = ""
            return True
        except Exception as e:
            self.write(e)
            return False

    def runsource(self, source, filename='<input>', symbol='single'):
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
        except SyntaxError:
            self.showsyntaxerror()
            self.lined_pushed = ""
            return False
        except OverflowError:
            self.showsyntaxerror()
            self.write("OverflowError")
            self.lined_pushed = ""
            return False
        except Exception as e:
            self.write(e)
            return False
        except:
            return False

    def push(self, line):
        """
        Upon pressing enter in the interactive shell the user "push" a string.
        This method is then called with the string pushed.
        We catch the string to send it via the rosservice.
        """
        self.lined_pushed += line + "\n"
        return code.InteractiveConsole.push(self, line)
