"""
The door to sofa demo carried out against MuJoCo physics, in a MuJoCo viewer.

Run it from this folder, naming the seat places a person sits on::

    python mujoco_demo.py RIGHT
"""

from __future__ import annotations

from coraplex.plans.executors import PhysicallySimulatedPlanExecutor
from demo import main, occupied_seats_from_command_line

# %% running it

if __name__ == "__main__":
    main(
        executor_type=PhysicallySimulatedPlanExecutor,
        occupied_seats=occupied_seats_from_command_line(),
    )
