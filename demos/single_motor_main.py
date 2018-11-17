# Import the required controllers / entities to build the control graph.
# Note that `robot.device` is provided by the Dynamic Graph Manager automatically.
import dynamic_graph as dg
from dynamic_graph_manager.demos.single_motor.dgm_single_motor_controller import MotorController

# Create the required controllers for the control graph. In this case, there is
# only a single controller.
mc = MotorController("mc")

# So far only the entities for the control graph are created. The next steps 
# builds the graph structure / combines the individual pieces. This is done
# by plugging the input and output signals from the robot.device towards the
# controller and vice verser. Here, the encoder/joint position of the robot.device
# is plugged towards the state of the controller. This way, the controller knows
# how to read the joint position when computing the torque. Eventually, the 
# computed torque is consumed by the robot.device.torque to figure out which
# command to apply in the next control cycle.
dg.plug(robot.device.signal('encoder'), mc.signal('state'))
dg.plug(mc.signal('torque'), robot.device.signal('torque'))

# The following is an example on how to adjust the gains by calling one of the
# commands defined on the motor controller entitiy.
gain = ((1.,), )
mc.setGain(gain, )

print("Create dynamic graph with robot.device <--> MotorController.")
