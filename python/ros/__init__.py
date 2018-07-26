#from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio
from ros_publish import RosPublish
from ros_subscribe import RosSubscribe
from ros_joint_state import RosJointState

from ros import Ros

# aliases, for retro compatibility
from ros import RosPublish as RosImport
from ros import RosSubscribe as RosExport


#class RosRobotModel(DynamicPinocchio):
#    def __init__(self, name):
#        DynamicPinocchio.__init__(self, name)
#        self.namespace = "sot_controller"
#        self.jointsParameterName_ = "jrl_map"

#    def setJointsNamesParameter(self):
#        import rospy
#        if self.model is not None:
#            parameter_name = self.namespace + "/" + jointsParameterName_
#            jointsName = []
#            for i in xrange(self.model.njoints):
#                jointsName.append(self.model.names[i])
#            rospy.set_param(parameter_name,jointsName)
#        return

#    def setNamespace(self, ns):
#        self.namespace = ns
#        return

#    def curConf(self):
#        return self.position.value
