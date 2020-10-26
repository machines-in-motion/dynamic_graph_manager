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
import rclpy
import dynamic_graph
import signal
from dynamic_graph_manager.msg import Vector
from dynamic_graph_manager.device import Device
from dynamic_graph_manager.ros import Ros
from dynamic_graph_manager.dynamic_graph.device.robot import Robot
from dynamic_graph_manager.wrapper import RosPythonInterpreterClient


class DgmProcess:
    """Create a class to spwan the DemoDGM in another process, to test the
    run_command C++ client and it's Python bindings.
    """

    def __init__(self):
        self.manager = Manager()
        self.condition = self.manager.Condition()
        self.another_process = Process(
            target=DgmProcess._run_demo_dynamic_graph_manager,
            args=(self.condition,))

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
            bash_command.split(), stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)

        condition.acquire()
        condition.notify()
        condition.wait()

        # kill the demo dgm
        dgm_subprocess.send_signal(signal.SIGINT)
        dgm_subprocess.wait()


def random_string():
    return ''.join(random.SystemRandom().choice(string.ascii_letters + string.digits) for _ in range(10))


class TestDynamicGraphManager(unittest.TestCase):

    def setUp(self):
        rospy.init_node("dgm_ut", anonymous=True, disable_signals=False)

        self.device_yaml_file = os.path.join(
            os.path.dirname(__file__), "config", "simple_robot.yaml")

    def tearDown(self):
        # rospy.signal_shutdown("dgm_ut tear down")
        pass

    def test_import_device(self):
        device_name = "simple_robot"
        d = Device(device_name)
        self.assertEqual(device_name, d.name)

    def test_import_ros_entity(self):
        device_name = "simple_robot"
        d = Device(device_name)
        ros_interface = Ros(d, suffix="_ut")
        self.assertEqual(ros_interface.rosPublish.name, "rosPublish_ut")
        self.assertEqual(ros_interface.rosSubscribe.name, "rosSubscribe_ut")
        self.assertEqual(ros_interface.rosTime.name, "rosTime_ut")
        self.assertEqual(ros_interface.rosRobotStatePublisher.name,
                         "rosRobotStatePublisher_ut")

    def test_rosPublish(self):
        device_name = "simple_robot"
        d = Device(device_name)
        d.initialize(self.device_yaml_file)
        ros_publish = Ros(d, suffix="_ut").rosPublish

        signal = d.encoders
        topic_type = "vector"
        new_signal_name = d.name + "_" + d.encoders.name.split("::")[-1]
        topic_name = new_signal_name

        ros_publish.add(topic_type, new_signal_name, topic_name)
        dynamic_graph.plug(signal, ros_publish.signal(new_signal_name))

        self.assertTrue(
            "/dynamic_graph/" + topic_name in rospy.get_published_topics("/dynamic_graph")[0])

    def test_rosSubscribe(self):
        # parameters
        type_name = "vector"
        signal_name = "subscribed_vector"
        topic_name = "/a_published_vector"
        # topic_name = "/" + random_string()
        msg = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)

        # publish a topic
        pub = rospy.Publisher(topic_name, Vector, queue_size=10)

        # subscribe the above topic
        device_name = "simple_robot"
        d = Device(device_name)
        ros_subscribe = Ros(d, suffix="_ut").rosSubscribe
        ros_subscribe.add(type_name, signal_name, topic_name)

        i = 0
        while not ros_subscribe.signal(signal_name).value:
            pub.publish(msg)
            ros_subscribe.signal(signal_name).recompute(i)
            time.sleep(0.5)
            i += 1
            if i > 50:
                break

        self.assertEqual(ros_subscribe.signal(signal_name).value, msg)

    def test_run_command(self):
        # start the demo dgm
        dgm = DgmProcess()
        dgm.start_dgm()

        # create a python client
        dgm_python_client = RosPythonInterpreterClient()

        # call to the python client
        ret_cmd = dgm_python_client.run_python_command("1+1")
        ret_script = dgm_python_client.run_python_script(
            os.path.join(os.path.dirname(__file__), "config", "simple_add.py"))

        # Stop the demo dgm
        dgm.stop_dgm()

        # check the results
        self.assertEqual(ret_cmd, "2")
        self.assertEqual(ret_script, "2\n")
