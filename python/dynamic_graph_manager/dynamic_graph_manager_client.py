"""dynamic_graph_manager_client

License BSD-3-Clause
Copyright (c) 2019, New York University and Max Planck Gesellschaft.
2019-....

This defines a remote python client to help sequencing the operations using
the DynamicGraphManager.

"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from dynamic_graph_manager_cpp_bindings import RosPythonInterpreterClient


class DynamicGraphManagerClient(Node):
    """ Defines a helper to programmatically interact with a DGM instance. """

    def __init__(self, node_name="dgm_client"):
        super().__init__(node_name)

        # Create the remote python client.
        self.remote_python_client = RosPythonInterpreterClient()

        # Create the remote start client.
        self.start_dg_client = self.create_client(
            Empty, "/dynamic_graph_manager/start_dynamic_graph"
        )
        while not self.start_dg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "'/dynamic_graph_manager/start_dynamic_graph' "
                "service not available, waiting again..."
            )
        self.start_dg_request = Empty.Request()

        # Create the remote stop client.
        self.stop_dg_client = self.create_client(
            Empty, "/dynamic_graph_manager/stop_dynamic_graph"
        )
        while not self.stop_dg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "'/dynamic_graph_manager/stop_dynamic_graph' "
                "service not available, waiting again..."
            )
        self.stop_dg_request = Empty.Request()

    def run_python_command(self, python_command):
        """ Execute a python command in the embedded python interpreter """
        self.remote_python_client.run_python_command(python_command)

    def run_python_script(self, file_path):
        """ Execute a python file in the embedded python interpreter """
        self.remote_python_client.run_python_script(file_path)

    def start_dynamic_graph(self):
        future = self.start_dg_client.call_async(self.start_dg_request)
        rclpy.spin_until_future_complete(self, future)

    def stop_dynamic_graph(self):
        future = self.start_dg_client.call_async(self.start_dg_request)
        rclpy.spin_until_future_complete(self, future)

    def start_tracer(self):
        self.run_python_command("robot.start_tracer()")

    def stop_tracer(self):
        self.run_python_command("robot.stop_tracer()")
