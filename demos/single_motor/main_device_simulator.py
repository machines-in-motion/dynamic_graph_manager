import os, os.path
import numpy as np
import matplotlib.pyplot as plt

import dynamic_graph as dg
from dynamic_graph_manager.demos.single_motor.dgm_single_motor_controller import MotorController
from dynamic_graph_manager.demos.single_motor.single_motor_device_simulator import SingleMotorDeviceSimulator

device = SingleMotorDeviceSimulator('device_simulator')
device.initialize(os.path.join(os.getcwd(), 'dgm_single_motor_simu.yaml'))


mc = MotorController("mc")

dg.plug(device.signal('encoder'), mc.signal('state'))
dg.plug(mc.signal('torque'), device.signal('torque'))

# Configure the gain of the P-controller.
gain = ((1.,), )
mc.setGain(gain, )

# Configure the desired position for the P-controller.
mc.setDesired(((2.,), ), )

T = 1000
traj = np.zeros((T, 2))

# Run the simulation for T steps.
for it in range(T):
  # Need to call executeGraph and step.
  device.executeGraph()
  device.step()

  traj[it][0] = device.signal('encoder').value[0]
  traj[it][1] = device.signal('torque').value[0]

# Plotting.
plt.plot(traj[:, 0], label='Encoder')
plt.plot(traj[:, 1], label='Torque')
plt.xlabel("Time [ms]")
plt.grid(True)
plt.legend()
plt.show()

