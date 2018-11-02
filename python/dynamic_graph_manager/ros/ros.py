from dynamic_graph import plug
from dynamic_graph.ros.ros_publish import RosPublish
from dynamic_graph.ros.ros_subscribe import RosSubscribe
from dynamic_graph.ros.ros_time import RosTime

class Ros(object):

    def __init__(self, robot, suffix = ''):
        self.robot = robot
        self.rosPublish = RosPublish('rosPublish{0}'.format(suffix))
        self.rosSubscribe = RosSubscribe('rosSubscribe{0}'.format(suffix))
        self.rosTime = RosTime ('rosTime{0}'.format(suffix))

        # aliases, for retro compatibility
        self.rosImport=self.rosPublish
        self.rosExport=self.rosSubscribe
