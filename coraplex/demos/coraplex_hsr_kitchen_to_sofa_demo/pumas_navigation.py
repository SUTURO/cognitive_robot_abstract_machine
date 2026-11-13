"""
The kitchen to sofa demo on the real HSR, whose base is driven by the Pumas navigation
of the lab, which plans its path on the laser map.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from coraplex.plans.executors import PlanExecutor, RealRobotPlanExecutor
from coraplex.pumas_navigation import PumasNavigation
from demo import KitchenToSofaDemonstration
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World

# %% the demonstration


@dataclass
class PumasKitchenToSofaDemonstration(KitchenToSofaDemonstration):
    """
    The kitchen to sofa demonstration on the real HSR, which Pumas drives to the sofa
    between two runs of Giskard.

    Giskard commands the base on every control cycle of a run, so Pumas only drives
    while no run is going on.
    """

    executor_type: type[PlanExecutor] = RealRobotPlanExecutor
    """
    The executor sending the arm, torso and head motions to Giskard.
    """

    map_T_scene: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.44, y=0.14, yaw=-0.058
        )
    )
    """
    Where the kitchen lies in the map frame of the laser map ``apartment_master1``,
    fitted to three room corners clicked on the laser map; see the README.
    """

    def perform_plan(self, world: World) -> None:
        """
        As :meth:`KitchenToSofaDemonstration.perform_plan`, with :meth:`departure_nodes`
        and :meth:`scanning_nodes` in two runs of Giskard and Pumas driving the base to
        :meth:`scanning_pose` between them.
        """
        self.execute_statechart(
            world,
            lambda context: self.statechart_of(context, self.departure_nodes(context)),
        )
        PumasNavigation(
            node=self.ros_node,
            robot=world.get_semantic_annotations_by_type(self.used_robot)[0],
        ).drive_to(self.scanning_pose(world))
        self.execute_statechart(
            world,
            lambda context: self.statechart_of(context, self.scanning_nodes(context)),
        )
