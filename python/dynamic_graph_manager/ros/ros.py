from dynamic_graph import plug

class Ros(object):
    device = None
    rosPublish = None
    rosSubscribe = None
    rosJointState = None

    # aliases, for retro compatibility
    rosImport = None
    rosExport = None

    def __init__(self, robot, suffix = ''):
        self.robot = robot
        self.rosPublish = RosPublish('rosPublish{0}'.format(suffix))
        self.rosSubscribe = RosSubscribe('rosSubscribe{0}'.format(suffix))
        self.rosJointState = RosJointState('rosJointState{0}'.format(suffix))
        self.rosJointState.retrieveJointNames(self.robot.dynamic.name)
        self.rosTime = RosTime ('rosTime{0}'.format(suffix))

        plug(self.robot.device.state, self.rosJointState.state)
        self.robot.device.after.addSignal(
            '{0}.trigger'.format(self.rosPublish.name))
        self.robot.device.after.addSignal(
            '{0}.trigger'.format(self.rosJointState.name))

        # aliases, for retro compatibility
        self.rosImport=self.rosPublish
        self.rosExport=self.rosSubscribe
