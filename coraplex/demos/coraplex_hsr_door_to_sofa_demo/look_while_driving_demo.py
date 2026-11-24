"""
The HSR drives from the door of the kitchen lab to the sofa and keeps looking at the
sofa on the way, in a MuJoCo viewer.

Run it from this folder::

    python look_while_driving_demo.py
"""

from __future__ import annotations

from dataclasses import dataclass

from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.composite.facing import NavigateAndLookAtAction
from coraplex.robot_plans.actions.core.navigation import (
    LookAtAction,
    LookAtWhileDrivingAction,
    PathPlanningNavigateAcrossFloorsAction,
)
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from demo import DoorToSofaDemonstration
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor, Sofa
from semantic_digital_twin.world import World

# %% the demonstration


@dataclass(kw_only=True)
class LookAtSofaWhileDrivingDemonstration(DoorToSofaDemonstration):
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

    collision_avoidance: bool = True
    """
    Whether the robot keeps away from the furniture and from itself.
    """

    clearance: float = 0.05
    """
    How far, beyond its radius, the planned path keeps the base from obstacles, in
    meters.
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
        Park the arm, look at the sofa, and drive to :meth:`scanning_pose` along a path
        around the furniture while looking at it.
        """
        world = context.world
        hsrb = context.require_extension(extension_type=RobotAccess).robot
        look_target = self.look_target(world=world)
        statechart = Statechart(context=context)
        statechart.add_node(
            node=Sequence(
                nodes=[
                    ParkArmsAction(arms=hsrb.all_arms),
                    LookAtAction(look_target=look_target),
                    NavigateAndLookAtAction(
                        navigate=PathPlanningNavigateAcrossFloorsAction(
                            planar_goal_pose=self.scanning_pose(world=world),
                            floors=world.get_semantic_annotations_by_type(Floor),
                            clearance=self.clearance,
                        ),
                        look_at=LookAtWhileDrivingAction(look_target=look_target),
                    ),
                ]
            )
        )
        return statechart


# %% running it

if __name__ == "__main__":
    LookAtSofaWhileDrivingDemonstration().run()
