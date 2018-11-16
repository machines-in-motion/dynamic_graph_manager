import os, os.path

import dynamic_graph_manager.demos.single_motor.single_motor_device_simulator as dev_sim


sim = dev_sim.SingleMotorDeviceSimulator('dev')
sim.initialize(os.path.join(os.getcwd(), 'dgm_single_motor_simu.yaml'))

sim.step()

