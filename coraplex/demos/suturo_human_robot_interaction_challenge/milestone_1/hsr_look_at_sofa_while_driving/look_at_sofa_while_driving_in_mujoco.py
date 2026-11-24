"""
The HSR drives from the door of the kitchen lab to the sofa and keeps looking at the
sofa on the way, in a MuJoCo viewer.

Run it from this folder::

    python look_at_sofa_while_driving_in_mujoco.py
"""

from __future__ import annotations

from dataclasses import dataclass

from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.composite.facing import NavigateAndLookAtAction
from coraplex.robot_plans.actions.core.navigation import (
    LookAtAction,
    LookAtWhileDrivingAction,
)
from coraplex.sofa_demonstrations import SofaDemonstration
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.semantic_annotations.semantic_annotations import Sofa
from semantic_digital_twin.world import World

# %% the demonstration


@dataclass(kw_only=True)
class LookAtSofaWhileDrivingDemonstration(SofaDemonstration):
    """
    The HSR drives from the door of the kitchen lab to the sofa while its head camera
    keeps looking at the sofa.
    """

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor
    """
    The executor running the plan, by default in MuJoCo.
    """

    ros_node_name: str = "hsr_look_at_sofa_while_driving_demo"
    """
    The name of the ROS node the demonstration runs as.
    """

    def look_target(self, *, world: World) -> Pose:
        """
        :param world: The world holding the sofa.
        :return: The origin of the sofa.
        """
        [sofa] = world.get_semantic_annotations_by_type(Sofa)
        return Pose.from_xyz_rpy(reference_frame=sofa.root)

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Do :meth:`departure_nodes`, look at the sofa, and drive to :meth:`scanning_pose`
        while looking at it.
        """
        look_target = self.look_target(world=context.world)
        return self.build_sequence(
            context=context,
            nodes=[
                *self.departure_nodes(context=context),
                LookAtAction(look_target=look_target),
                NavigateAndLookAtAction(
                    navigate=self.drive_to_sofa(world=context.world),
                    look_at=LookAtWhileDrivingAction(look_target=look_target),
                ),
            ],
        )


# %% running it

if __name__ == "__main__":
    LookAtSofaWhileDrivingDemonstration().run()
