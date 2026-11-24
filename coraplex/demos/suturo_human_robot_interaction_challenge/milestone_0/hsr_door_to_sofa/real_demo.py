"""
The door to sofa demo carried out by the real HSR.

Put the robot at the door of the kitchen lab, then run it from this folder::

    python real_demo.py

The world is fetched from the running world server, and the people on the sofa are those
the perception pipeline sees.
"""

from __future__ import annotations

from coraplex.plans.executors import RealRobotPlanExecutor
from demo import main

# %% running it

if __name__ == "__main__":
    main(executor_type=RealRobotPlanExecutor)
