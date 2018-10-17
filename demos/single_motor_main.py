import dynamic_graph as dg
from dynamic_graph.demos.single_motor.dgm_single_motor_controller import MotorController

mc = MotorController("mc")

dg.plug(robot.device.signal('encoder'), mc.signal('state'))
dg.plug(mc.signal('torque'), robot.device.signal('torque'))

gain = ((1.,), )
mc.setGain(gain, )

print("Create dynamic graph with robot.device <--> MotorController.")
