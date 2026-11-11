"""
The kitchen to sofa demo carried out by the real HSR, which says the free seats aloud.

Put the robot in the kitchen, then run it from this folder::

    python real_demo.py

The world is fetched from the running world server, the people on the sofa are those the
perception pipeline sees, and the sentence goes to the HSR's voice.
"""

from __future__ import annotations

from coraplex.plans.executors import RealRobotPlanExecutor
from demo import FreeSeatAnnouncement, KitchenToSofaDemonstration, main

# %% running it

if __name__ == "__main__":
    main(
        KitchenToSofaDemonstration(executor_type=RealRobotPlanExecutor),
        FreeSeatAnnouncement(speak=True),
    )
