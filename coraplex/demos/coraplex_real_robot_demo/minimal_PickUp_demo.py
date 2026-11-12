"""
A milk carton appears at a fixed place in the map, facing the way the real HSR faces,
and the HSR picks it up and puts it down at a fixed place in the map.

Run it from this folder while the HSR's controller is running::

    python minimal_PickUp_demo.py
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

from coraplex.demonstrations import RobotDemonstration
from coraplex.hsrb_hand import RealHSRBPlanExecutor
from coraplex.plans.context_extensions import RobotAccess
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.statechart import Statechart
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.world_entity import Body

# %% the scene


class SceneFile(PathEnumeration):
    """
    The files the scene is built from, under the coraplex resources.
    """

    MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"
    """
    The mesh of the milk carton.
    """


class SceneBody(StrEnum):
    """
    The names of the scene's bodies the demonstration refers to.
    """

    COUNTER = "counter_top"
    """
    The worktop along the wall, which marks whether the kitchen was already merged in.
    """

    MILK = "milk"
    """
    The milk carton spawned in front of the robot.
    """

    TRASH_CAN = "TRASH_CAN"
    """
    The Trashcan ... COMMENTHERE
    """


class HSRBBody(StrEnum):
    """
    The HSR's own bodies the demonstration refers to.
    """

    WRIST = "wrist_roll_link"
    """
    The wrist the hand is mounted on, which is not part of the end effector.
    """


# %% the demonstration


@dataclass
class MilkSpawnDemonstration(RobotDemonstration):
    """
    Spawns a milk carton at :attr:`milk_x`, :attr:`milk_y` in the map, facing the way
    the real HSR's base faces, and has the HSR pick it up without driving, park its arm and put it down at
    :attr:`place_x`, :attr:`place_y`.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that picks up the milk.
    """

    ros_node_name: str = "hsr_minimal_pick_up_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = RealHSRBPlanExecutor
    """
    The executor running the plan, which drives the real HSR, its hand included.
    """

    milk_x: float = 3.19
    """
    The x coordinate in the map at which the milk is spawned, in meters.
    """

    milk_y: float = 3.15
    """
    The y coordinate in the map at which the milk is spawned, in meters.
    """
    # 3.19, 3.23
    milk_bottom_height: float = 0
    """
    How high above the floor of the map the milk's lowest point is spawned, in meters.
    """

    fraction_of_height: float = 2 / 6
    """
    How far up the milk's bounding box the hand takes hold of it, from 0 at its bottom
    to 1 at its top.
    """

    lift_height: float = 0.25
    """
    How far the hand rises with the milk once it holds it, in meters.
    """

    place_x: float = 2.86
    """
    The x coordinate in the map at which the milk is put down, in meters.
    """

    place_y: float = 2.8
    """
    The y coordinate in the map at which the milk is put down, in meters.
    """

    place_bottom_height: float = 0.5
    """
    How high above the floor of the map the milk's lowest point is put down, in meters.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(semantic_annotation_type=self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneBody.COUNTER)

    def acquire_world(self) -> World:
        """
        Obtain the world as :meth:`RobotDemonstration.acquire_world` does, and put the
        milk at :meth:`compute_milk_pose` in it, spawning it unless an earlier run left it
        there. The HSR's hand and wrist may touch the milk, which they have to grasp.
        """
        world = super().acquire_world()
        if world.is_kinematic_structure_entity_in_world_by_name(SceneBody.MILK):
            milk = world.get_body_by_name(SceneBody.MILK)
        else:
            milk = (
                Milk.get_annotation_specification(
                    name=SceneBody.MILK.value,
                    root_specification=BodySpecification.mesh(
                        SceneBody.MILK.value,
                        str(SceneFile.MILK),
                        connection_specification=Connection6DoFSpecification(),
                    ),
                )
                .spawn(world)
                .root
            )
        [robot] = world.get_semantic_annotations_by_type(self.used_robot)
        milk.parent_connection.origin = self.compute_milk_pose(
            robot, milk
        ).homogeneous_matrix
        [arm] = robot.all_arms
        with world.modify_world():
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=[
                        world.get_body_by_name(HSRBBody.WRIST),
                        *arm.end_effector.bodies_with_collision,
                    ],
                    body_group_b=[milk],
                )
            )
        return world

    def compute_milk_pose(self, robot: AbstractRobot, milk: Body) -> Pose:
        """
        :param robot: The robot that picks up the milk.
        :param milk: The body of the milk.
        :return: The pose of ``milk`` in the world root at :attr:`milk_x`,
            :attr:`milk_y`, upright, facing the way the base of ``robot`` faces, with
            its lowest point at :attr:`milk_bottom_height`.
        """
        return self.compute_upright_milk_pose(
            milk,
            x=self.milk_x,
            y=self.milk_y,
            bottom_height=self.milk_bottom_height,
            yaw=robot.root.global_pose.yaw,
        )

    def compute_place_pose(self, robot: AbstractRobot, milk: Body) -> Pose:
        """
        :param robot: The robot that puts the milk down.
        :param milk: The body of the milk.
        :return: The pose of ``milk`` in the world root at :attr:`place_x`,
            :attr:`place_y`, upright, facing the way the base of ``robot`` faces, with
            its lowest point at :attr:`place_bottom_height`.
        """
        return self.compute_upright_milk_pose(
            milk,
            x=self.place_x,
            y=self.place_y,
            bottom_height=self.place_bottom_height,
            yaw=robot.root.global_pose.yaw,
        )

    @staticmethod
    def compute_upright_milk_pose(
        milk: Body, x: float, y: float, bottom_height: float, yaw: float
    ) -> Pose:
        """
        :param milk: The body of the milk.
        :param x: The x coordinate of ``milk`` in the world root.
        :param y: The y coordinate of ``milk`` in the world root.
        :param bottom_height: The height of the lowest point of ``milk`` in the world
            root.
        :param yaw: The rotation of ``milk`` about the vertical axis of the world root.
        :return: The pose of ``milk`` in the world root, standing upright.
        """
        milk_box = milk.collision.as_bounding_box_collection_in_frame(
            milk
        ).bounding_box()
        return Pose.from_xyz_rpy(
            x=x,
            y=y,
            z=bottom_height - milk_box.min_z,
            yaw=yaw,
            reference_frame=milk._world.root,
        )

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(RobotAccess).robot
        [milk] = context.world.get_semantic_annotations_by_type(Milk)
        [arm] = robot.all_arms
        place_pose = self.compute_place_pose(robot, milk.root)
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                    PickUpAction(
                        grasp=GraspCandidate(milk, self.compute_grasp_pose(milk.root)),
                        arm=arm,
                        retreat_distance=self.lift_height,
                    ),
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                    PlaceAction(object_designator=milk, target_location=place_pose),
                    ParkArmsAction(arms=robot.all_arms),
                ]
            )
        )
        return statechart

    def compute_grasp_pose(self, milk: Body) -> Pose:
        """
        :param milk: The body of the milk, standing upright in its own frame.
        :return: The grasp frame on the z axis of ``milk``, at :attr:`fraction_of_height`
            of its bounding box, in its frame.
        """
        box = milk.collision.as_bounding_box_collection_in_frame(milk).bounding_box()
        return Pose.from_xyz_rpy(
            z=box.min_z + self.fraction_of_height * (box.max_z - box.min_z),
            reference_frame=milk,
        )


# %% running the demonstration


def main(collision_avoidance: bool = False, debug: bool = False) -> World:
    """
    Run the demonstration on the real HSR.

    :param collision_avoidance: Whether the statechart avoids collisions.
    :param debug: Whether to run in debug mode, publishing every copy of the world a
        candidate is tried in.
    :return: The world the milk was picked up in.
    """
    return MilkSpawnDemonstration(
        collision_avoidance=collision_avoidance,
        event_segmentation=False,
        debug=debug,
    ).run()


if __name__ == "__main__":
    main()
