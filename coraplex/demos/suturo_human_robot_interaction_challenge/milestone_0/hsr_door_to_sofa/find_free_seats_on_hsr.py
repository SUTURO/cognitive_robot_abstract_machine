"""
The door to sofa demo carried out by the real HSR, which Pumas drives to the sofa.

Start AMCL and Pumas on the map ``apartment_master1``, Giskard and RoboKudo, put the
robot at the door, then run it from this folder::

    source ~/ros_clean_env.sh
    python find_free_seats_on_hsr.py
"""

from __future__ import annotations

from coraplex.plans.executors import RealRobotPlanExecutor
from free_seats_demonstration import OneLookDoorToSofaDemonstration, main
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix

# %% running it

if __name__ == "__main__":
    main(
        OneLookDoorToSofaDemonstration(
            executor_type=RealRobotPlanExecutor,
            drives_with_pumas=True,
            map_T_scene=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=0.44, y=0.14, yaw=-0.058
            ),
        )
    )
