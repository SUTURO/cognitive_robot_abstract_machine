"""
The HSR drives from its start to a goal pose in the kitchen environment, in MuJoCo.

Run it from this folder::

    python demo_in_kitchen.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

from typing_extensions import TYPE_CHECKING

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    PhysicallySimulatedPlanExecutor,
    ForwardDrivingPlanExecutor,
)
from coraplex.plans.plan_transformation import ConfigurationTransformation
from coraplex.robot_plans.actions.core.navigation import (
    DrivesBase,
    PathPlanningNavigateAction,
)
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Parallel, Sequence
from cramph.statechart import Statechart
from giskardpy.motion_statechart.tasks.drive_tasks import StraightLineBaseGoal
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB, HSRBJoint
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World


@dataclass
class KitchenNavigationDemonstration(RobotDemonstration):
    """
    The HSR parks its arm, turns its head to the front of its base and drives from
    :attr:`robot_start` to the goal, facing the way it drives.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that drives.
    """

    ros_node_name: str = "hsr_kitchen_navigation_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = ForwardDrivingPlanExecutor
    """
    The executor running the plan, which performs it in MuJoCo.
    """

    collision_avoidance: bool = True
    """
    Whether collision avoidance is added to every motion state chart of this run.
    """

    simulation_time_per_action: timedelta | None = timedelta(seconds=120)
    """
    The simulated time each action may take, enough for a drive that crosses the
    apartment around the furniture.
    """

    event_segmentation: bool = False
    """
    Whether the events of this run are segmented; there is nothing to segment here.
    """

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(x=0, y=0)
    )
    """
    Where the HSR stands before the plan begins.
    """

    goal_x: float = 3.75189
    """
    Where the HSR drives to along the world's x axis, in meters.
    """

    goal_y: float = 4.66774
    """
    Where the HSR drives to along the world's y axis, in meters.
    """

    goal_yaw: float = 0.0
    """
    Which way the HSR faces at the goal, in radians around the world's z axis.
    """

    second_goal_x: float = 0.973906
    """
    Where the HSR drives to after the goal, in the bedroom, along the world's x axis, in
    meters.
    """

    second_goal_y: float = 4.48839
    """
    Where the HSR drives to after the goal, in the bedroom, along the world's y axis, in
    meters.
    """

    obstacle_clearance: float = 0.1
    """
    How much further than its base radius the HSR keeps away from furniture, in meters,
    so that its parked hand passes the furniture too.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            odometry_T_robot_start=self.robot_start,
        )

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name("sofa")

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
        goal = Pose.from_xyz_rpy(
            x=self.goal_x,
            y=self.goal_y,
            yaw=self.goal_yaw,
            reference_frame=context.world.root,
        )
        head_forward = JointState(
            connections=[context.world.get_connection_by_name(HSRBJoint.HEAD_PAN)],
            target_values=[0.0],
        )
        statechart = Statechart(context=context)
        statechart.add_node(
            node=Sequence(
                nodes=[
                    Parallel(
                        nodes=[
                            ParkArmsAction(arms=robot.all_arms),
                            JointPositionList(goal_state=head_forward),
                        ]
                    ),
                    PathPlanningNavigateAction(
                        target=goal, obstacle_clearance=self.obstacle_clearance
                    ),
                    PathPlanningNavigateAction(
                        start=goal,
                        obstacle_clearance=self.obstacle_clearance,
                        target=Pose.from_xyz_rpy(
                            x=self.second_goal_x,
                            y=self.second_goal_y,
                            yaw=self.goal_yaw,
                            reference_frame=context.world.root,
                        ),
                    ),
                ]
            )
        )
        return statechart


def main(headless: bool = False) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :return: The world the HSR drove in.
    """
    return KitchenNavigationDemonstration(headless=headless).run()


if __name__ == "__main__":
    main()
