"""
The HSR lays a place setting in the SuTuRo lab of
:class:`~semantic_digital_twin.predefined_maps.kitchen_environment.KitchenEnvironment`:
it carries the milk, a bowl and a spoon from the kitchen counter top onto the desk.

Runs in simulation, so nothing on the network is needed. The scaffolding in
:mod:`coraplex.demonstrations` owns the ROS session and publishes the world to Rviz, so
the run can be watched while it happens.

Every transport leaves the grasp and the standing pose open: each pair is tried out
before it is executed, and the first that succeeds is taken.

While the plan runs, SegMind segments it into the grasps, pick-ups, placings and
containments of the place setting, and reports them once the plan is done.
"""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass, field
from enum import Enum, StrEnum
from pathlib import Path

import numpy as np
from typing_extensions import Type

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess, StatementGrounding
from coraplex.plans.executors import KinematicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.composite.transporting import TransportAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from krrood.entity_query_language.factories import an, entity, variable
from krrood.patterns.path_enumeration import PathEnumeration
from segmind.detectors.agent_event_detector_nodes import GraspDetector
from segmind.detectors.coarse_event_detector_nodes import (
    PickUpDetector,
    PlacingDetector,
)
from segmind.detectors.spatial_relation_detector_nodes import ContainmentDetector
from segmind.event_segmentation import Segmind
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootKinematicStructureEntity,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bowl,
    Floor,
    Milk,
    Spoon,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection

# %% what the scene is built from


class SceneFile(PathEnumeration):
    """
    The meshes of the place setting, under the coraplex resources.
    """

    MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"
    BOWL = Path(__file__).resolve().parents[2] / "resources" / "objects" / "bowl.stl"
    SPOON = Path(__file__).resolve().parents[2] / "resources" / "objects" / "spoon.stl"


class LabBody(StrEnum):
    """
    The lab's own bodies the demonstration refers to.
    """

    COUNTER_TOP = "counter_top"
    """
    The kitchen counter top the place setting starts on, which also marks whether the
    scene was already populated.
    """


class HSRBBody(StrEnum):
    """
    The HSR's own bodies the demonstration refers to.
    """

    WRIST = "wrist_roll_link"
    """
    The wrist the hand is mounted on, which is not part of the end effector.
    """


class LabSurfaceHeight(float, Enum):
    """
    How high above the floor the lab's surfaces the place setting stands on lie.
    """

    COUNTER_TOP = 0.85
    DESK = 0.765


@dataclass
class PlaceSettingObject:
    """
    One object of the place setting: what it is, where it starts, and where it is laid
    on the desk, in the room frame.
    """

    semantic_annotation_type: Type[HasRootKinematicStructureEntity]
    """
    What the object is.

    Its body is named after it, so the plan can ask for it by type.
    """

    mesh: SceneFile
    """
    The mesh the object is shaped by.
    """

    start_surface: LabSurfaceHeight
    """
    The surface the object starts on.
    """

    start_x: float
    """
    Where along the room's x axis the object starts.
    """

    start_y: float
    """
    Where along the room's y axis the object starts.
    """

    along_desk: float
    """
    Where along the desk the object is laid, which is the room's y axis.
    """

    origin_height: float
    """
    How high the object's origin stands above the surface it rests on.
    """

    start_yaw: float = -np.pi / 2
    """
    Which way the object faces in the room frame where it starts.
    """

    yaw_against_robot: float = 0.0
    """
    How the object is turned on the desk against the way the robot faces the desk.
    """

    across_desk: float = 0.2
    """
    Where across the desk the object is laid, 15 cm behind its front edge.
    """

    def spawn(self, world: World, map_T_room: HomogeneousTransformationMatrix) -> None:
        """
        Put the object where it starts and name it among the world's annotations.

        :param world: The world to spawn into.
        :param map_T_room: Where the frame the lab is laid out in lies in the map.
        """
        name = self.semantic_annotation_type.__name__
        SemanticAnnotationWithRootSpecification(
            name=name,
            semantic_annotation_type=self.semantic_annotation_type,
            root_specification=BodySpecification.mesh(name, str(self.mesh)),
        ).spawn(
            world,
            parent_T_self=map_T_room
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                self.start_x,
                self.start_y,
                self.start_surface + self.origin_height,
                yaw=self.start_yaw,
            ),
        )

    def annotation_in(self, world: World) -> HasRootKinematicStructureEntity:
        """
        :param world: The world the object was spawned into.
        :return: The annotation naming the object, as the plan refers to it.
        """
        return next(
            an(
                entity(
                    variable(
                        self.semantic_annotation_type,
                        domain=world.semantic_annotations,
                    )
                )
            ).evaluate()
        )

    def target_location(
        self,
        world: World,
        map_T_room: HomogeneousTransformationMatrix,
        robot_yaw_at_desk: float,
    ) -> Pose:
        """
        :param world: The world the desk stands in.
        :param map_T_room: Where the frame the lab is laid out in lies in the map.
        :param robot_yaw_at_desk: Which way the robot faces in the room frame while it
            stands at the desk.
        :return: Where on the desk the object is laid.
        """
        map_T_target = (
            map_T_room
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                self.across_desk,
                self.along_desk,
                LabSurfaceHeight.DESK + self.origin_height,
                yaw=robot_yaw_at_desk + self.yaw_against_robot,
            )
        ).to_np()
        return Pose.from_xyz_rpy(
            *map_T_target[:3, 3],
            yaw=float(np.arctan2(map_T_target[1, 0], map_T_target[0, 0])),
            reference_frame=world.root,
        )


# %% the demonstration


@dataclass
class SuturoNewLabDemonstration(RobotDemonstration):
    """
    The HSR transports the milk, a bowl and a spoon from the counter top onto the desk.
    """

    used_robot: Type[HSRB] = HSRB

    ros_node_name: str = "suturo_new_lab_demo_node"

    lab: KitchenEnvironment = field(default_factory=KitchenEnvironment)
    """
    The lab the demonstration runs in.
    """

    map_T_room: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.33, y=0.28, yaw=0.10707963267
        )
    )
    """
    Where the frame :attr:`lab` is laid out in lies in the map the robot localizes in.
    """

    robot_yaw_at_desk: float = np.pi
    """
    Which way the robot faces in the room frame while it stands at the desk, whose front
    edge looks along the room's x axis.
    """

    robot_start_in_room: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            1.9, -0.9, yaw=-np.pi / 2
        )
    )
    """
    Where the HSR stands before the plan begins, in the room frame: in the kitchen,
    facing the counter top.
    """

    milk: PlaceSettingObject = field(
        default_factory=lambda: PlaceSettingObject(
            Milk,
            SceneFile.MILK,
            LabSurfaceHeight.COUNTER_TOP,
            start_x=2.3,
            start_y=-1.97,
            along_desk=1.08,
            origin_height=0.09,
        )
    )
    """
    The milk, which starts on the counter top.
    """

    bowl: PlaceSettingObject = field(
        default_factory=lambda: PlaceSettingObject(
            Bowl,
            SceneFile.BOWL,
            LabSurfaceHeight.COUNTER_TOP,
            start_x=2.55,
            start_y=-1.97,
            along_desk=1.28,
            origin_height=0.035,
        )
    )
    """
    The bowl, which starts on the counter top.
    """

    spoon: PlaceSettingObject = field(
        default_factory=lambda: PlaceSettingObject(
            Spoon,
            SceneFile.SPOON,
            LabSurfaceHeight.COUNTER_TOP,
            start_x=2.8,
            start_y=-1.97,
            along_desk=1.48,
            origin_height=0.01,
            start_yaw=np.pi / 2,
            yaw_against_robot=np.pi,
        )
    )
    """
    The spoon, which starts on the counter top turned towards the robot, the way its
    only grasp takes it.
    """

    @property
    def place_setting(self) -> tuple[PlaceSettingObject, ...]:
        """
        The objects carried onto the desk, in the order they are carried.

        They are laid 20 cm apart, so placing one keeps the gripper clear of the last.
        """
        return self.milk, self.bowl

    def scene_specification(self) -> WorldSpecification:
        """
        :return: An empty world: the lab is not described by a model file, so
            :meth:`build_scene` builds it with :class:`KitchenEnvironment` instead.
        """
        return WorldSpecification()

    def build_scene(self) -> World:
        """
        :return: The map, holding :attr:`lab` at :attr:`map_T_room`, populated by
            :meth:`populate_scene`.
        """
        scene = World.create_with_root_body()
        lab = self.lab.get_world()
        scene.merge_world(
            lab,
            root_connection=FixedConnection(
                parent=scene.root,
                child=lab.root,
                parent_T_connection_expression=self.map_T_room,
            ),
        )
        self.populate_scene(scene)
        return scene

    def robot_specification(self) -> RobotSpecification:
        """
        :return: The HSR, placed at :attr:`robot_start_in_room` through its odom.
        """
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            world_T_odom=self.map_T_room @ self.robot_start_in_room,
        )

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(LabBody.COUNTER_TOP)

    def populate_scene(self, world: World) -> None:
        """
        Put the place setting where it starts.
        """
        for placed_object in self.place_setting:
            placed_object.spawn(world, self.map_T_room)

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        """
        Resolve the robot and let it touch the place setting and the floors, see
        :meth:`relax_collision_rules`.
        """
        robot = world.get_semantic_annotations_by_type(self.used_robot)[0]
        self.relax_collision_rules(world, robot)
        return [RobotAccess(robot), StatementGrounding()]

    def relax_collision_rules(self, world: World, robot: HSRB) -> None:
        """
        Let the HSR's wrist come as close to the place setting as a grasp needs: the
        HSR's end effector begins at its palm, so the wrist behind it would otherwise
        stall short of the grasp.

        Let the HSR touch the floors of the lab, which its wheels stand on.

        :param world: The world holding the robot, the lab and the place setting.
        :param robot: The HSR.
        """
        with world.modify_world():
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=[world.get_body_by_name(HSRBBody.WRIST)],
                    body_group_b=[
                        placed_object.annotation_in(world).root
                        for placed_object in self.place_setting
                    ],
                )
            )
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=robot.bodies_with_collision,
                    body_group_b=[
                        floor.root
                        for floor in world.get_semantic_annotations_by_type(Floor)
                    ],
                )
            )

    def segment_events(self, world: World) -> AbstractContextManager:
        """
        Detect the place setting being grasped, picked up, placed and contained.
        """
        return Segmind.create_for_semantic_annotation_types(
            world,
            tuple(
                placed_object.semantic_annotation_type
                for placed_object in self.place_setting
            ),
            detector_types=(
                PickUpDetector,
                PlacingDetector,
                ContainmentDetector,
                GraspDetector,
            ),
        )


    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Carry each object to its place on the desk.
        """
        world = context.world
        robot = context.require_extension(RobotAccess).robot
        arm = robot.all_arms[0]
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    *(
                        TransportAction.from_graspable_by_closest_grasps(
                            placed_object.annotation_in(world),
                            placed_object.target_location(
                                world, self.map_T_room, self.robot_yaw_at_desk
                            ),
                            arm,
                            context,
                        )
                        for placed_object in self.place_setting
                    ),
                ]
            )
        )
        return statechart


def main(
    executor_type: type[PlanExecutor] = KinematicallySimulatedPlanExecutor,
    collision_avoidance: bool = True,
    event_segmentation: bool = True,
    debug: bool = False,
) -> None:
    """
    Run the demonstration.

    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :param collision_avoidance: Whether the statechart avoids collisions.
    :param event_segmentation: Whether SegMind segments the run into events.
    :param debug: Whether to run in debug mode, publishing every copy of the world a
        candidate is tried in.
    """
    SuturoNewLabDemonstration(
        executor_type=executor_type,
        collision_avoidance=collision_avoidance,
        event_segmentation=event_segmentation,
        debug=debug,
    ).run()


if __name__ == "__main__":
    main()
