"""
The kitchen environment in a MuJoCo viewer. Run it from anywhere::

    python kitchen_mujoco.py

The program ends when the viewer window is closed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)

# %% the viewer


@dataclass
class KitchenInMujoco:
    """
    Simulates the kitchen environment until its viewer is closed.
    """

    viewer_check_period: float = 0.1
    """
    How often to check whether the viewer is still open, in seconds.
    """

    def run(self) -> None:
        """
        Open the viewer and keep the simulation going until it is closed.
        """
        simulation = MujocoSim(world=KitchenEnvironment().get_world())
        simulation.start_simulation()
        while simulation.is_running():
            time.sleep(self.viewer_check_period)
        simulation.stop_simulation()


if __name__ == "__main__":
    KitchenInMujoco().run()
