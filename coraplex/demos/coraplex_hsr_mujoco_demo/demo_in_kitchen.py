"""
The HSR drives from the kitchen into the office in MuJoCo, around the furniture in
its way. Run it from anywhere::

    python demo_in_kitchen.py

The viewer stays open after the robot arrives; the program ends when it is closed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from datetime import timedelta

import numpy as np

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.plans.plan_transformation import (
    ConfigurationTransformation,
    PlanRewriting,
)
from coraplex.robot_plans.actions.core.navigation import (
    DrivesBase,
    NavigateAction,
    PathPlanningNavigateAction,
)
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from giskardpy.motion_statechart.goals.base_pose_goals import StraightLineBaseGoal
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.reasoning.queries import find_room, find_free_space_in_room, find_free_space_next_to
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
)

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

    def facing(self, target: FloorPose) -> FloorPose:
        """
        :param target: The place to face.
        :return: This place, facing ``target``.
        """
        yaw = np.arctan2(float(target.y - self.y), float(target.x - self.x))
        return replace(self, yaw=float(yaw))

    @classmethod
    def of_robot(cls, robot: AbstractRobot) -> FloorPose:
        """
        :param robot: The robot standing on the floor.
        :return: Where ``robot`` stands, without regard to where it faces.
        """
        pose = robot.root.global_pose
        return cls(x=float(pose.x), y=float(pose.y))


# %% driving facing forward


@dataclass
class DriveFacingForward(ConfigurationTransformation[DrivesBase]):
    """
    Drives the base to each pose with a :class:`StraightLineBaseGoal`, so that it faces
    the way it moves instead of sliding sideways while turning.
    """

    def is_applicable(self, plan_node: DrivesBase) -> bool:
        return True

    def apply(self, plan_node: DrivesBase) -> None:
        plan_node.base_goal_type = StraightLineBaseGoal


# %% the demonstration


@dataclass
class KitchenToLivingRoomDemonstration(RobotDemonstration):
    """
    The HSR parks its arm and drives from the kitchen into the living room, facing the
    way it moves and keeping clear of the furniture.
    """

    used_robot: type[AbstractRobot] = HSRB

    ros_node_name: str = "hsr_kitchen_to_living_room_demo"

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor

    collision_avoidance: bool = True

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0, y=0
        )
    )
    """
    Where the HSR starts, in the doorway of the kitchen.
    """

    viewer_frame_duration: timedelta = timedelta(seconds=1 / 60)
    """
    How much simulated time passes between two frames of the viewer that stays open
    after the plan, matched by the same span of real time.
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
        return KitchenEnvironment().create_world()

    def acquire_world(self) -> World:
        """
        Differs from :meth:`~coraplex.demonstrations.RobotDemonstration.acquire_world`
        in letting the robot touch the floors, which collision avoidance would otherwise
        keep it from standing on.
        """
        world = super().acquire_world()
        robot = world.get_semantic_annotations_by_type(self.used_robot)[0]
        floor_bodies = [
            body
            for floor in world.get_semantic_annotations_by_type(Floor)
            for body in floor.bodies
        ]
        with world.modify_world():
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=robot.bodies_with_collision, body_group_b=floor_bodies
                )
            )
        return world

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot, odometry_T_robot_start=self.robot_start)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name("living_room_floor")

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [
            RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0]),
            PlanRewriting(transformations=[DriveFacingForward()]),
        ]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arm, then drive through the kitchen into the living room.
        """
        world = context.world
        hsrb = context.require_extension(RobotAccess).robot
        statechart = Statechart(context=context)
        counter_top = world.get_semantic_annotation_by_name("sideboard")
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=hsrb.all_arms),
                    PathPlanningNavigateAction(
                        target=find_free_space_next_to(
                            counter_top, preferred_point=hsrb.root.global_pose
                        )
                    ),
                ]
            )
        )
        return statechart

    def tear_down(self) -> None:
        """
        Keep the physics running in real time while the viewer is open, then
        :meth:`~coraplex.demonstrations.RobotDemonstration.tear_down`.
        """
        if self.simulation is not None and not self.headless:
            renderer = self.simulation.simulator.renderer
            frame_seconds = self.viewer_frame_duration.total_seconds()
            while renderer.is_running():
                frame_start = time.monotonic()
                self.simulation.step_simulation(self.viewer_frame_duration)
                time.sleep(max(0.0, frame_seconds - (time.monotonic() - frame_start)))
        super().tear_down()

    def _drive(
        self, world: World, start: FloorPose, target: FloorPose
    ) -> list[NavigateAction]:
        """
        :param world: The world the robot drives in.
        :param start: Where the robot stands before it drives.
        :param target: Where the robot drives to.
        :return: The moves that turn the robot on the spot towards ``target`` and then
            drive it straight there, so that it faces the way it is going.
        """
        heading = start.facing(target)
        return [
            NavigateAction(target_location=heading.in_world(world)),
            NavigateAction(
                target_location=replace(target, yaw=heading.yaw).in_world(world)
            ),
        ]


def main(headless: bool = False) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :return: The world the robot drove in.
    """
    return KitchenToLivingRoomDemonstration(headless=headless).run()


if __name__ == "__main__":
    main()