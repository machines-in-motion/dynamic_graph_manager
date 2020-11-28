"""Word completion for GNU readline.

The completer completes keywords, built-ins and globals in a selectable
namespace (which defaults to __main__); when completing NAME.NAME..., it
evaluates (!) the expression up to the last dot and completes its attributes.

It's very cool to do "import sys" type "sys.", hit the completion key (twice),
and see the list of names defined by the sys module!

Tip: to use the tab key as the completion key, call

    readline.parse_and_bind("tab: complete")

Notes:

- Exceptions raised by the completer function are *ignored* (and generally cause
  the completion to fail).  This is a feature -- since readline sets the tty
  device in raw (or cbreak) mode, printing a traceback wouldn't work well
  without some complicated hoopla to save, reset and restore the tty state.

- The evaluation of the NAME.NAME... form may cause arbitrary application
  defined code to be executed if an object with a __getattr__ hook is found.
  Since it is the responsibility of the application (or the user) to enable this
  feature, I consider this an acceptable risk.  More complicated expressions
  (e.g. function calls or indexing operations) are *not* evaluated.

- GNU readline is also used by the built-in functions input() and
raw_input(), and thus these also benefit/suffer from the completer
features.  Clearly an interactive application can benefit by
specifying its own completer function and using raw_input() for all
its input.

- When the original stdin is not a tty device, GNU readline is never
  used, and this module (and the readline module) are silently inactive.

"""

import ast
from dynamic_graph_manager_cpp_bindings import RosPythonInterpreterClient

__all__ = ["DGCompleter"]


class DGCompleter:
    def __init__(self, ros_python_interpreter_client=None):
        """Create a new completer for the command line.

        Completer([client]) -> completer instance.

        Client is a ROS proxy to dynamic_graph run_python_command service.

        Completer instances should be used as the completion mechanism of
        readline via the set_completer() call:

        readline.set_completer(Completer(client).complete)
        """
        if ros_python_interpreter_client is None:
            self.client = RosPythonInterpreterClient()
        else:
            self.client = ros_python_interpreter_client

        cmd = (["if \"local_completer\" not in globals():\n"
                "    print(\"Load the dg_completer\")\n"
                "    from rlcompleter import Completer\n"
                "    local_completer=Completer()\n"
                "    import readline\n"
                "    readline.set_completer(local_completer.complete)\n"
                "    readline.parse_and_bind(\"tab: complete\")"
        ])
       
        for python_command in cmd:
            print(python_command)
            self.client.run_python_command(python_command)

        self.buffer = []

    def complete(self, text, state):
        """Return the next possible completion for 'text'.readline.parse_and_bind("tab: complete")

        This is called successively with state == 0, 1, 2, ... until it
        returns None. The completion should begin with 'text'.
        """
        # self.client.run_python_command("global local_completer")
        cmd = "local_completer.complete(\""+text+"\","+str(state)+")"
        response = self.client.run_python_command(cmd)
        if not response or response is "\n" or response is None:
            response = None
            self.buffer = []
        else:
            self.buffer += [response]
        return response
