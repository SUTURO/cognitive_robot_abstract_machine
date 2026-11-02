"""
The HSR drives from the kitchen into the living room in MuJoCo. Run it from anywhere::

    python demo_in_kitchen.py
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World

# %% poses on the floor


@dataclass
class FloorPose:
    """
    A place on the floor of the world and the direction the robot faces there.
    """

    x: float
    """
    The x coordinate in the world's root frame, in meters.
    """

    y: float
    """
    The y coordinate in the world's root frame, in meters.
    """

    yaw: float = 0.0
    """
    The direction the robot faces, counterclockwise from the root frame's x-axis, in
    radians.
    """

    def in_world(self, world: World) -> Pose:
        """
        :param world: The world whose root frame this place is given in.
        :return: This place as a pose in ``world``.
        """
        return Pose.from_xyz_rpy(
            x=self.x, y=self.y, yaw=self.yaw, reference_frame=world.root
        )


# %% the demonstration


@dataclass
class KitchenToLivingRoomDemonstration(RobotDemonstration):
    """
    The HSR parks its arm and drives from the kitchen into the living room.
    """

    used_robot: type[AbstractRobot] = HSRB

    ros_node_name: str = "hsr_kitchen_to_living_room_demo"

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(x=0, y=0)
    )
    """
    Where the HSR starts, in the doorway of the kitchen.
    """

    kitchen_centre: FloorPose = field(
        default_factory=lambda: FloorPose(x=1.5, y=0.0, yaw=np.pi / 2)
    )
    """
    The free floor in the middle of the kitchen, facing the living room, driven through
    so that the robot passes clear of the desk next to the doorway.
    """

    living_room: FloorPose = field(
        default_factory=lambda: FloorPose(x=1.5, y=2.5, yaw=np.pi / 2)
    )
    """
    The free floor in the living room, beside the sofa.
    """

    def scene_specification(self) -> WorldSpecification:
        """
        .. note:: Never read: the kitchen is a predefined world rather than a
            specification, so :meth:`build_scene` builds it directly.
        """
        return WorldSpecification()

    def build_scene(self) -> World:
        """
        :return: The kitchen environment, without the robot.
        """
        return KitchenEnvironment().get_world()

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot, odom_T_robot_start=self.robot_start)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name("living_room_floor")

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arm, then drive through the kitchen into the living room.
        """
        world = context.world
        hsrb = context.require_extension(RobotAccess).robot
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=hsrb.all_arms),
                    NavigateAction(target_location=self.kitchen_centre.in_world(world)),
                    NavigateAction(target_location=self.living_room.in_world(world)),
                ]
            )
        )
        return statechart


def main(headless: bool = False) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :return: The world the robot drove in.
    """
    return KitchenToLivingRoomDemonstration(headless=headless).run()


if __name__ == "__main__":
    main()
