"""
license BSD 3-clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft

Unit-tests for the python API of the DynamicGraphManager
"""

import random
import string
import time
import os.path
import unittest
import multiprocessing
from multiprocessing import Process, Manager
import subprocess
import signal
import numpy as np
import dynamic_graph
import rclpy
from mim_msgs.msg import Vector
from dynamic_graph_manager.dynamic_graph.device import Device
from dynamic_graph_manager.ros.ros import Ros
from dynamic_graph_manager.robot import Robot
from dynamic_graph_manager_cpp_bindings import RosPythonInterpreterClient


class DgmProcess:
    """Create a class to spawn the DemoDGM in another process, to test the
    run_command C++ client and it's Python bindings.
    """

    def __init__(self):
        self.manager = Manager()
        self.condition = self.manager.Condition()
        self.another_process = Process(
            target=DgmProcess._run_demo_dynamic_graph_manager, args=(self.condition,)
        )

    def start_dgm(self):
        self.condition.acquire()
        self.another_process.start()
        self.condition.wait()

    def stop_dgm(self):
        self.condition.notify()
        self.condition.release()
        self.another_process.join()

    @staticmethod
    def _run_demo_dynamic_graph_manager(condition):

        # start the demo dgm
        bash_command = "ros2 run dynamic_graph_manager demo_dynamic_graph_manager"
        dgm_subprocess = subprocess.Popen(
            bash_command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

        condition.acquire()
        condition.notify()
        condition.wait()

        # kill the demo dgm
        dgm_subprocess.send_signal(signal.SIGINT)
        dgm_subprocess.wait()


def random_string():
    return "".join(
        random.SystemRandom().choice(string.ascii_letters + string.digits)
        for _ in range(10)
    )


class TestDynamicGraphManager(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        self.ros_node = rclpy.create_node(
            "unit_test_node", namespace="dynamic_graph_manager"
        )

        self.device_yaml_file = os.path.join(
            os.path.dirname(__file__), "config", "simple_robot.yaml"
        )

    def tearDown(self):
        self.ros_node.destroy_node()

    def test_import_device(self):
        device_name = "simple_robot"
        d = Device(device_name)
        self.assertEqual(device_name, d.name)

    def test_import_ros_entity(self):
        device_name = "simple_robot"
        d = Device(device_name)
        ros_interface = Ros(d, suffix="_ut")
        self.assertEqual(ros_interface.ros_publish.name, "ros_publish_ut")
        self.assertEqual(ros_interface.ros_subscribe.name, "ros_subscribe_ut")

    def test_ros_publish(self):
        device_name = "simple_robot"
        d = Device(device_name)
        d.initialize(self.device_yaml_file)

        ros_publish = Ros(d, suffix="_ut").ros_publish

        signal = d.encoders
        topic_type = "vector"
        new_signal_name = d.name + "_" + d.encoders.name.split("::")[-1]
        topic_name = new_signal_name

        ros_publish.add(topic_type, new_signal_name, topic_name)
        dynamic_graph.plug(signal, ros_publish.signal(new_signal_name))

        time.sleep(0.1)
        self.assertTrue(
            ("/dynamic_graph_manager/" + topic_name, ["mim_msgs/msg/Vector"])
            in self.ros_node.get_topic_names_and_types()
        )

        msgs_rx = []
        sub = self.ros_node.create_subscription(
            Vector, topic_name, lambda msg: msgs_rx.append(msg), 10
        )

        i = 0
        while True:
            d.encoders.value = np.array([i, i + 1, i + 2])
            ros_publish.signal("trigger").recompute(i)
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            i += 1
            if len(msgs_rx) > 5 or i > 20:
                break

        self.assertTrue(len(msgs_rx) > 5)
        self.ros_node.destroy_subscription(sub)

    def test_ros_subscribe(self):
        # parameters
        type_name = "vector"
        signal_name = "subscribed_vector"
        topic_name = "/a_published_vector"
        msg = Vector()
        msg.data = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]

        # publish a topic
        pub = self.ros_node.create_publisher(Vector, topic_name, 10)

        # subscribe the above topic
        device_name = "simple_robot"
        device = Device(device_name)
        ros_subscribe = Ros(device, suffix="_ut").ros_subscribe
        ros_subscribe.add(type_name, signal_name, topic_name)

        time.sleep(0.1)
        self.assertTrue(
            (topic_name, ['mim_msgs/msg/Vector'])
            in self.ros_node.get_topic_names_and_types()
        )

        i = 0
        while ros_subscribe.signal(signal_name).value.size == 0:
            pub.publish(msg)
            ros_subscribe.signal(signal_name).recompute(i)
            i += 1
            if i > 50:
                self.fail("No messsage received by the subscription")

        np.testing.assert_equal(ros_subscribe.signal(signal_name).value, msg.data)

        self.ros_node.destroy_publisher(pub)

    # def test_run_command(self):
    #     # start the demo dgm
    #     dgm = DgmProcess()
    #     dgm.start_dgm()

    #     # create a python client
    #     dgm_python_client = RosPythonInterpreterClient()

    #     # call to the python client
    #     ret_cmd = dgm_python_client.run_python_command("1+1")
    #     ret_script = dgm_python_client.run_python_script(
    #         os.path.join(os.path.dirname(__file__), "config", "simple_add.py")
    #     )

    #     # Stop the demo dgm
    #     dgm.stop_dgm()

    #     # check the results
    #     self.assertEqual(ret_cmd, "2")
    #     self.assertEqual(ret_script, "2\n")
