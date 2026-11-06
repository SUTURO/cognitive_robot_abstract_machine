"""
The door to sofa demo in kinematic simulation, shown in RViz.

Run it from this folder, naming the seat places a person sits on::

    python rviz_demo.py RIGHT

Watch it with the fixed frame ``root`` and a ``MarkerArray`` display on
``/semworld/viz_marker``.
"""

from __future__ import annotations

from coraplex.plans.executors import KinematicallySimulatedPlanExecutor
from demo import main, occupied_seats_from_command_line

# %% running it

if __name__ == "__main__":
    main(
        executor_type=KinematicallySimulatedPlanExecutor,
        occupied_seats=occupied_seats_from_command_line(),
    )
