"""
The door to sofa demo carried out by the real HSR.

Put the robot at the door of the kitchen lab, then run it from this folder::

    python find_free_seats_on_hsr_with_giskard.py

The world is fetched from the running world server, and the people on the sofa are those
the perception pipeline sees.
"""

from __future__ import annotations

from coraplex.plans.executors import RealRobotPlanExecutor
from free_seats_demonstration import DoorToSofaDemonstration, main

# %% running it

if __name__ == "__main__":
    main(DoorToSofaDemonstration(executor_type=RealRobotPlanExecutor))
