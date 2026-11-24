"""
The door to sofa demo carried out against MuJoCo physics, in a MuJoCo viewer.

Run it from this folder, naming the seat places a person sits on::

    python find_free_seats_in_mujoco.py RIGHT
"""

from __future__ import annotations

from coraplex.plans.executors import PhysicallySimulatedPlanExecutor
from free_seats_demonstration import (
    DoorToSofaDemonstration,
    main,
    occupied_seats_from_command_line,
)

# %% running it

if __name__ == "__main__":
    main(
        DoorToSofaDemonstration(
            executor_type=PhysicallySimulatedPlanExecutor,
            occupied_seats=occupied_seats_from_command_line(),
        )
    )
