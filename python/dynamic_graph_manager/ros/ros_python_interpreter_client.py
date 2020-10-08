#!/usr/bin/env python3
import sys

from dynamic_graph_manager.srv import RunPythonCommand
import rclpy
from rclpy.node import Node


class RosPythonInterpreterClient(Node):
    def __init__(self):
        super().__init__("ros_python_interpreter_client")
        self.run_python_command_client = self.create_client(RunPythonCommand, "/dynamic_graph_manager/run_python_command")
        while not self.run_python_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.run_python_command_request = RunPythonCommand.Request()

    def send_request(self, python_command):
        self.run_python_command_request.input = str(python_command)
        self.future = self.run_python_command_client.call_async(self.run_python_command_request)


def main(args=None):
    rclpy.init(args=args)

    print("create the client")
    minimal_client = RosPythonInterpreterClient()

    cmd = "print('banana')"
    print("send ", cmd)
    minimal_client.send_request(cmd)

    print("request sent.")
    while rclpy.ok():
        print("Spin once.")
        rclpy.spin_once(minimal_client)
        print("Spin done.")
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info("Service call failed %r" % (e,))
            else:
                minimal_client.get_logger().info(
                    "Result of RunPythonCommand: input(%s), result(%s), standard_output(%s), standard_error(%s)"
                    % (minimal_client.run_python_command_request.input,
                       response.result, response.standard_output, response.standard_error)
                )
            break
        else:
            print("Spin once failed...")

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()