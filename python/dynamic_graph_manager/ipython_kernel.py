from IPython.lib.kernel import find_connection_file
from IPython.zmq.blockingkernelmanager import BlockingKernelManager


class IPythonKernelServer:
    def __init__(self):
        IPython.start_kernel(argv=[])


class IPythonKernelClient:
    def __init__(self):
        # this is a helper method for turning a fraction of a connection-file name
        # into a full path.  If you already know the full path, you can just use that
        cf = find_connection_file("6759")

        km = BlockingKernelManager(connection_file=cf)
        # load connection info and init communication
        km.load_connection_file()
        km.start_channels()

        def run_cell(km, code):
            # now we can run code.  This is done on the shell channel
            shell = km.shell_channel
            print
            print "running:"
            print code

            # execution is immediate and async, returning a UUID
            msg_id = shell.execute(code)
            # get_msg can block for a reply
            reply = shell.get_msg()

            status = reply["content"]["status"]
            if status == "ok":
                print "succeeded!"
            elif status == "error":
                print "failed!"
                for line in reply["content"]["traceback"]:
                    print line

        run_cell(km, "a=5")
        run_cell(km, "b=0")
        run_cell(km, "c=a/b")
