"""
Start a MuJoCo simulation with a viewer, stop it and let the process end right away, as
a script does once its simulation is done.

Takes the path of the scene file as its only argument.
"""

import sys

from physics_simulators.mujoco_simulator import MujocoSimulator

# %% stopping and ending the process

simulator = MujocoSimulator(_headless=False, file_path=sys.argv[1])
simulator.start(simulate_in_thread=False, render_in_thread=False)
for _ in range(100):
    simulator.step()
simulator.render()
simulator.stop()
