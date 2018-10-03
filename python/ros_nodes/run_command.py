#!/usr/bin/env python

# standard import
import sys
import os

# Used to compile the instruction given in the python terminal
import code
from code import InteractiveConsole
from dynamic_graph.ros.dgcompleter import DGCompleter

# Used to connect to ROS services
import rospy
from dynamic_graph_manager.srv import RunCommand, RunPythonFile

# Used to deal with the python history
try:
    import readline
    is_readline_import = True
except ImportError:
    print("Module readline not available.")
    is_readline_import = False
import atexit


HISTFILE="%s/.pyhistory" % os.environ["HOME"]


def savehist():
    if is_readline_import:
        readline.write_history_file(HISTFILE)


class RosShell(InteractiveConsole):
    def __init__(self):

        # Create some cache to save the non executed commands
        self.cache = ""

        # Create the python terminal
        InteractiveConsole.__init__(self)

        # Create a client for the single python command service of the
        # dynamic_graph_manager
        service_name = '/dynamic_graph/run_python_command'
        rospy.loginfo('waiting for service' + service_name + '...')
        rospy.wait_for_service(service_name)
        self.client = rospy.ServiceProxy(service_name, RunCommand, True)

        # Create a client for the python script reading service of the
        # dynamic_graph_manager
        service_name = '/dynamic_graph/run_python_script'
        rospy.loginfo('waiting for service' + service_name + '...')
        rospy.wait_for_service(service_name)
        self.scriptClient = rospy.ServiceProxy(service_name, RunPythonFile, True)
        
        # Initialize the python completion
        readline.set_completer(DGCompleter(self.client).complete)
        readline.parse_and_bind("tab: complete")

        # Read the existing history if there is one
        if os.path.exists(HISTFILE):
            readline.read_history_file(HISTFILE)
            
        # Set maximum number of items that will be written to the history file
        readline.set_history_length(300)

    # save the python command history
    atexit.register(savehist)

    def runcode(self, code, retry = True):
        source = self.cache[:-1]
        self.cache = ""
        if source != "":
            try:
                if not self.client:
                    print("Connection to remote server lost. Reconnecting...")
                    self.client = rospy.ServiceProxy(
                        'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
                    if retry:
                        print("Retrying previous command...")
                        self.cache = source
                        return self.runcode(code, False)
                response = self.client(str(source))
                if response.standardoutput != "":
                    print response.standardoutput[:-1]
                if response.standarderror != "":
                    print response.standarderror[:-1]
                elif response.result != "None":
                    print response.result
            except rospy.ServiceException, e:
                print("Connection to remote server lost. Reconnecting...")
                self.client = rospy.ServiceProxy(
                    'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
                if retry:
                    print("Retrying previous command...")
                    self.cache = source
                    self.runcode(code, False)

    def runsource(self, source, filename = '<input>', symbol = 'single'):
        try:
            c = code.compile_command(source, filename, symbol)
            if c:
                return self.runcode(c)
            else:
                return True
        except SyntaxError, OverflowError:
            self.showsyntaxerror()
            self.cache = ""
            return False

    def push(self,line):
        self.cache += line + "\n"
        return InteractiveConsole.push(self,line)

if __name__ == '__main__':
    import optparse
    import os.path
    rospy.init_node('run_command', argv=sys.argv)
    sys.argv = rospy.myargv(argv=None)
    parser = optparse.OptionParser(
        usage='\n\t%prog [options]')
    (options, args) = parser.parse_args(sys.argv[1:])

    sh = RosShell()

    if args[:]:
        infile = args[0]
        if os.path.isfile(infile):
            if not sh.client:
                print("Connection to remote server has been lost.")
                sys.exit(1)
            response = sh.scriptClient(os.path.abspath(infile))
            if not response:
                print("Error while file parsing ")
        else:
            print("Provided file does not exist: %s"%(infile))
    else:
        sh.interact("Interacting with remote server.")
