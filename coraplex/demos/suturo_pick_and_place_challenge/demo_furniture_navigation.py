"""
The HSR navigates from one piece of furniture to another in the kitchen environment,
in MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
from typing_extensions import TYPE_CHECKING

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
    Pose2D,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World

# %% the scene


class SceneBody(StrEnum):
    """
    The names of the scene's bodies the demonstration relies on.
    """

    KITCHEN_SURFACE = "sideboard"
    """
    The worktop of the kitchen island.
    """


# %% the demonstration


@dataclass(kw_only=True)
class NavigationDemonstration(RobotDemonstration):
    """
    The HSR lowers its torso, parks its arm and drives to a piece of furniture.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that drives.
    """

    ros_node_name: str = "hsr_furniture_navigation_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor
    """
    The executor running the plan, which performs it in MuJoCo.
    """

    collision_avoidance: bool = True
    """
    Whether collision avoidance is added to every motion state chart of this run.
    """

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1.1, y=2.5
        )
    )
    """
    Where the HSR stands before the plan begins.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            odometry_T_robot_start=self.robot_start,
        )

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(
            SceneBody.KITCHEN_SURFACE
        )

    def acquire_world(self) -> World:
        """
        Obtain the world as :meth:`RobotDemonstration.acquire_world` does, in which the
        whole HSR may touch the kitchen's floors.
        """
        world = super().acquire_world()
        [robot] = world.get_semantic_annotations_by_type(self.used_robot)
        with world.modify_world():
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=robot.bodies_with_collision,
                    body_group_b=[
                        floor.root
                        for floor in world.get_semantic_annotations_by_type(Floor)
                    ],
                )
            )
        return world

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [
            RobotAccess(
                robot=world.get_semantic_annotations_by_type(self.used_robot)[0]
            )
        ]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(extension_type=RobotAccess).robot
        taking_pose = self._standing_pose_facing(
            target=Pose.from_xyz_rpy(x=1, y=3, reference_frame=context.world.root),
            approach_from=robot.root.global_pose,
        )
        statechart = Statechart(context=context)
        statechart.add_node(
            node=Sequence(
                nodes=[
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                    ParkArmsAction(arms=robot.all_arms),
                    NavigateAction(planar_goal_pose=taking_pose),
                ]
            )
        )
        return statechart

    def _standing_pose_facing(
        self, *, target: Pose, approach_from: Pose | Pose2D
    ) -> Pose2D:
        """
        :param target: What the robot goes to reach, in the world frame.
        :param approach_from: Where the robot stands before it goes to `target`, in the
            world frame.
        :return: Where the robot's base stands on the floor, one meter off `target`
            towards `approach_from` and facing `target`.
        """
        target_position = target.position.to_np()[:2]
        offset = target_position - approach_from.position.to_np()[:2]
        standing_position = target_position - offset / np.linalg.norm(offset)
        return Pose2D.from_xy_yaw(
            x=float(standing_position[0]),
            y=float(standing_position[1]),
            yaw=float(np.arctan2(offset[1], offset[0])),
            reference_frame=target.reference_frame,
        )


# %% running the demonstration


def main(
    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor,
    collision_avoidance: bool = True,
    headless: bool = False,
) -> World:
    """
    Run the demonstration.

    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :param collision_avoidance: Whether the statechart avoids collisions.
    :param headless: Whether the physics runs without a viewer.
    :return: The world the HSR drove in.
    """
    return NavigationDemonstration(
        executor_type=executor_type,
        collision_avoidance=collision_avoidance,
        headless=headless,
    ).run()


if __name__ == "__main__":
    main()
