"""
The HSR throws the milk away in the SuTuRo lab of
:class:`~semantic_digital_twin.predefined_maps.kitchen_environment.KitchenEnvironment`:
it takes the milk from the dining table, carries it to the trash can and lets go
of it above the trash can's opening.

Runs in simulation, so nothing on the network is needed. The scaffolding in
:mod:`coraplex.demonstrations` owns the ROS session and publishes the world to Rviz, so
the run can be watched while it happens.

The pick-up leaves the grasp and the standing pose open: each pair is tried out before
it is executed, and the first that succeeds is taken. At the trash can the robot stands
on the side it comes from, clear of the trash can and within reach of its opening,
facing it, and holds the milk turned the way it took it.

.. note:: A kinematically simulated world has no gravity, so there the milk stays where
    it is let go, above the trash can.

While the plan runs, SegMind segments it into the grasps, pick-ups, placings and
containments of the milk, and reports them once the plan is done.
"""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass, field, replace
from enum import Enum, StrEnum
from pathlib import Path

import mujoco
import numpy as np
from typing_extensions import TYPE_CHECKING

from coraplex.demonstrations import RobotDemonstration
from coraplex.locations.locations import ReachabilityLocation
from coraplex.plans.context_extensions import RobotAccess, StatementGrounding
from coraplex.plans.executors import (
    PlanExecutor,
    PhysicallySimulatedPlanExecutor,
)
from coraplex.robot_plans.actions.core.pick_up import PickUpAction

from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.composite.transporting import (
    MoveAndPickUpAction,
    MoveAndPlaceAction,
    TransportAction,
)
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

from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootKinematicStructureEntity,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Milk,
    TrashCan,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Point3, Pose
from semantic_digital_twin.specifications.connections import Connection6DoFSpecification
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    FixedConnection,
)
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.geometry import Scale

if TYPE_CHECKING:
    from semantic_digital_twin.robots.robot_parts import Arm

# %% what the scene is built from


class SceneFile(PathEnumeration):
    """
    The meshes of the garbage, under the coraplex resources.
    """

    MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"


class LabBody(StrEnum):
    """
    The lab's own bodies the demonstration refers to.
    """

    COUNTER_TOP = "counter_top"
    """
    The kitchen counter top, which marks whether the scene was already populated.
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
    How high above the floor the lab's surfaces the garbage stands on lie.
    """

    COUNTER_TOP = 0.85
    """
    The kitchen counter top.
    """

    DINING_TABLE = 0.78
    """
    The dining table.
    """


@dataclass
class Garbage:
    """
    An object the robot throws away: what it is, where it starts in the room frame, and
    how it is let go of above the trash can.
    """

    semantic_annotation_type: type[HasRootKinematicStructureEntity]
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

    origin_height: float
    """
    How high the object's origin stands above the bottom of the object.
    """

    start_yaw: float = -np.pi / 2
    """
    Which way the object faces in the room frame where it starts.
    """

    yaw_against_robot: float = 0.0
    """
    How the object is turned against the way the robot faces it while the robot picks
    it up, and so while the robot holds it.
    """

    drop_clearance: float = 0.05
    """
    How far above the trash can's rim the bottom of the object is when it is let go.
    """

    box_scale: Scale = field(default_factory=lambda: Scale(0.05, 0.05, 0.2))
    """
    The size of the box the object is simulated as.
    """

    grasp_height: float = 0.05
    """
    How far above the grasp closest to the robot the object is taken hold of.
    """

    def spawn(self, world: World, map_T_room: HomogeneousTransformationMatrix) -> None:
        """
        Put the object where it starts and name it among the world's annotations.

        :param world: The world to spawn into.
        :param map_T_room: Where the frame the lab is laid out in lies in the map.
        """
        name = self.semantic_annotation_type.__name__
        garbage = SemanticAnnotationWithRootSpecification(
            name=name,
            semantic_annotation_type=self.semantic_annotation_type,
            root_specification=BodySpecification.mesh(
                name,
                str(self.mesh),
                connection_specification=Connection6DoFSpecification(),
            ),
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
        ContactParameters.create_for_grasped_object().apply_to([garbage.root])

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

    def drop_point(self, world: World, trash_can: TrashCan) -> Point3:
        """
        :param world: The world the trash can stands in.
        :param trash_can: The trash can the object is thrown into.
        :return: Where the object's origin is when it is let go, in the world frame:
            centred above the trash can's opening, :attr:`drop_clearance` above its rim.
        """
        rim_height = (
            trash_can.as_bounding_box_collection_in_frame(trash_can.root)
            .bounding_box()
            .max_z
        )
        return world.transform(
            Point3(
                z=rim_height + self.drop_clearance + self.origin_height,
                reference_frame=trash_can.root,
            ),
            world.root,
        )

    def drop_location(self, drop_point: Point3, robot_yaw: float) -> Pose:
        """
        :param drop_point: Where the object's origin is when it is let go.
        :param robot_yaw: Which way the robot faces in the frame of `drop_point` while
            it lets go of the object.
        :return: The object at `drop_point`, turned against the robot as it was when
            the robot picked it up.
        """
        return Pose.from_xyz_rpy(
            x=float(drop_point.x),
            y=float(drop_point.y),
            z=float(drop_point.z),
            yaw=robot_yaw + self.yaw_against_robot,
            reference_frame=drop_point.reference_frame,
        )


# %% the demonstration


@dataclass
class ThrowAwayDemonstration(RobotDemonstration):
    """
    The HSR takes the milk from the dining table and throws it into the trash can.
    """

    used_robot: type[HSRB] = HSRB

    ros_node_name: str = "suturo_throw_away_demo_node"

    lab: KitchenEnvironment = field(default_factory=KitchenEnvironment)
    """
    The lab the demonstration runs in.
    """

    lab_parts_movable: bool = False
    """
    Whether the drawers and doors of :attr:`lab` can move.

    Parts that cannot move are welded in place, so the physics simulates no joints for
    them.
    """

    map_T_room: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.33, y=0.28, yaw=0.10707963267
        )
    )
    """
    Where the frame :attr:`lab` is laid out in lies in the map the robot localizes in.
    """

    trash_can_in_room: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(3.6, 4.3)
    )
    """
    Where the middle of the trash can's base stands in the room frame: on the open floor
    between the dining table and the cupboard.
    """

    robot_start_in_room: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            3.6, 5.3, yaw=np.pi
        )
    )
    """
    Where the HSR stands before the plan begins, in the room frame: beside the cupboard,
    facing the dining table.
    """

    friction_cone: mujoco.mjtCone = mujoco.mjtCone.mjCONE_ELLIPTIC
    """
    The friction cone the contacts use: the elliptic cone is the exact one, where the
    default pyramid only approximates it and lets a held carton creep between the
    fingers.
    """

    impedance_ratio: float = 10.0
    """
    How much harder sliding is than pressing in: MuJoCo's own guidance for grasping with
    an elliptic cone.
    """

    no_slip_iterations: int = 2
    """
    Passes spent removing the slip soft contacts leave, so a held carton stays where the
    fingers hold it.
    """

    garbage: Garbage = field(
        default_factory=lambda: Garbage(
            Milk,
            SceneFile.MILK,
            LabSurfaceHeight.DINING_TABLE,
            start_x=2.81,
            start_y=5.6,
            origin_height=0.09,
            start_yaw=np.pi,
        )
    )
    """
    The milk, which starts on the dining table at its end towards the cupboard and is
    thrown away.
    """

    def scene_specification(self) -> WorldSpecification:
        """
        :return: An empty world: the lab is not described by a model file, so
            :meth:`build_scene` builds it with :class:`KitchenEnvironment` instead.
        """
        return WorldSpecification()

    def build_scene(self) -> World:
        """
        :return: The map, holding :attr:`lab` at :attr:`map_T_room` with its trash can
            moved to :attr:`trash_can_in_room` and its movable parts welded in place
            unless :attr:`lab_parts_movable`, populated by :meth:`populate_scene`.
        """
        scene = World.create_with_root_body()
        lab = self.lab.get_world()
        room = lab.root
        scene.merge_world(
            lab,
            root_connection=FixedConnection(
                parent=scene.root,
                child=room,
                parent_T_connection_expression=self.map_T_room,
            ),
        )
        [trash_can] = scene.get_semantic_annotations_by_type(TrashCan)
        base_height = (
            trash_can.as_bounding_box_collection_in_frame(trash_can.root)
            .bounding_box()
            .min_z
        )
        with scene.modify_world():
            scene.remove_connection(trash_can.root.parent_connection)
            scene.add_connection(
                FixedConnection(
                    parent=room,
                    child=trash_can.root,
                    parent_T_connection_expression=self.trash_can_in_room
                    @ HomogeneousTransformationMatrix.from_xyz_rpy(z=-base_height),
                )
            )
        if not self.lab_parts_movable:
            self.weld_movable_parts(scene)
        self.populate_scene(scene)
        return scene

    @staticmethod
    def weld_movable_parts(scene: World) -> None:
        """
        Replace every joint of `scene` that moves along or about one axis by a fixed
        connection holding its child where it currently is.

        :param scene: The world whose joints are replaced.
        """
        joints = [
            connection
            for connection in scene.connections
            if isinstance(connection, ActiveConnection1DOF)
        ]
        welds = [
            FixedConnection(
                parent=joint.parent,
                child=joint.child,
                parent_T_connection_expression=joint.origin,
            )
            for joint in joints
        ]
        with scene.modify_world():
            for joint in joints:
                scene.remove_connection(joint)
            for weld in welds:
                scene.add_connection(weld)

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
        Put the garbage where it starts.
        """
        self.garbage.spawn(world, self.map_T_room)

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        """
        Resolve the robot and let it touch the garbage and the floors, see
        :meth:`relax_collision_rules`.
        """
        robot = world.get_semantic_annotations_by_type(self.used_robot)[0]
        self.relax_collision_rules(world, robot)
        return [RobotAccess(robot), StatementGrounding()]

    def relax_collision_rules(self, world: World, robot: HSRB) -> None:
        """
        Let the HSR's wrist come as close to the garbage as a grasp needs: the HSR's end
        effector begins at its palm, so the wrist behind it would otherwise stall short
        of the grasp.

        Let the HSR touch the floors of the lab, which its wheels stand on.

        :param world: The world holding the robot, the lab and the garbage.
        :param robot: The HSR.
        """
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

    def segment_events(self, world: World) -> AbstractContextManager:
        """
        Detect the garbage being grasped, picked up, placed and contained.
        """
        return Segmind.create_for_semantic_annotation_types(
            world,
            (self.garbage.semantic_annotation_type,),
            detector_types=(
                PickUpDetector,
                PlacingDetector,
                ContainmentDetector,
                GraspDetector,
            ),
        )

    def standing_position_at(
        self,
        context: StatechartContext,
        drop_point: Point3,
        arm: Arm,
        approach_from: Pose,
    ) -> Pose:
        """
        :param context: The context of the plan, whose robot throws the garbage away.
        :param drop_point: Where the garbage is let go, in the world frame.
        :param arm: The arm that holds the garbage over the trash can.
        :param approach_from: Where the robot stands before it goes to the trash can,
            in the world frame.
        :return: Of the poses clear of the surroundings from which `arm` reaches
            `drop_point`, the one closest to standing at the arm's reach
            distance off `drop_point` on the side of `approach_from`.
        """
        location = ReachabilityLocation(
            target_pose=Pose(
                position=drop_point, reference_frame=drop_point.reference_frame
            ),
            arm=arm,
            context=context,
        )
        drop_position = drop_point.to_np()[:2]
        approach_direction = approach_from.position.to_np()[:2] - drop_position
        ideal_position = drop_position + (
            approach_direction
            / np.linalg.norm(approach_direction)
            * float(arm.approximate_length())
            * location.reach_fraction
        )
        return min(
            location,
            key=lambda candidate: np.linalg.norm(
                candidate.position.to_np()[:2] - ideal_position
            ),
        )

    @staticmethod
    def orient_towards(pose: Pose, target: HasRootKinematicStructureEntity) -> Pose:
        """
        :param pose: The pose to turn, in the world frame.
        :param target: The object to face.
        :return: `pose` at its position, turned level about the vertical so that its
            x-axis points at the origin of `target`.
        """
        offset = (
            target.root.global_pose.position.to_np()[:2] - pose.position.to_np()[:2]
        )
        return Pose.from_xyz_rpy(
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z),
            yaw=float(np.arctan2(offset[1], offset[0])),
            reference_frame=pose.reference_frame,
        )

    def _grasp_from(
        self, world: World, standing_pose: Pose, milk: Milk
    ) -> GraspCandidate:
        """
        :param world: The world the milk stands in.
        :param standing_pose: Where the robot stands to take the milk.
        :param milk: The milk.
        :return: The milk's grasp closest to the robot standing there, raised
            :attr:`Garbage.grasp_height` along the milk's vertical.
        """
        [closest] = GraspsByCloseness(milk, standing_pose, number_of_grasps=1)()
        milk_T_raised_grasp = (
            HomogeneousTransformationMatrix.from_xyz_rpy(
                z=self.garbage.grasp_height, reference_frame=milk.root
            )
            @ world.transform(closest.grasp_pose, milk.root).homogeneous_matrix
        )
        return replace(closest, grasp_pose=milk_T_raised_grasp.pose)

    def _standing_pose_facing(
        self, milk_pose: Pose, standing_distance: float = 0.5
    ) -> Pose:
        """
        :param milk_pose: Where the milk stands, turned the way the robot faces it.
        :param standing_distance: How far before the milk the robot's base stands, in
            meters.
        :return: Where the robot's base stands on the floor to reach the milk there,
            `standing_distance` before it and facing it.
        """
        return (
            milk_pose.homogeneous_matrix
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=-standing_distance, z=-float(milk_pose.z)
            )
        ).pose

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Pick the garbage up, carry it to the trash can and let go of it there.
        """
        world = context.world
        robot = context.require_extension(RobotAccess).robot
        arm = robot.all_arms[0]

        [trash_can] = world.get_semantic_annotations_by_type(TrashCan)
        garbage = self.garbage.annotation_in(world)
        taking_pose = self._standing_pose_facing(garbage.root.global_pose)
        drop_point = self.garbage.drop_point(world, trash_can)
        throwing_pose = self.orient_towards(
            self.standing_position_at(context, drop_point, arm, taking_pose), trash_can
        )
        drop_location = self.garbage.drop_location(drop_point, float(throwing_pose.yaw))

        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    # Concrete steps: in a physics run the milk is held by contact
                    # only, so the place finds what it holds from the pick-up before
                    # it, which a statement's trial run of the place would not see.
                    PickUpAction(grasp=self._grasp_from(world, taking_pose, garbage), arm=arm),
                    # TransportAction(
                    #     pick_up=MoveAndPickUpAction.from_standing_position(
                    #         taking_pose,
                    #         self._grasp_from(world, taking_pose, garbage),
                    #         arm,
                    #     ),
                    #     # pick_up=MoveAndPickUpAction.from_graspable_by_closest_grasps(
                    #     #     garbage,
                    #     #     arm,
                    #     #     context
                    #     # ),
                    #     place=MoveAndPlaceAction.from_standing_position(
                    #         throwing_pose,
                    #         drop_location,
                    #         garbage,
                    #     ),
                    # ),
                    ParkArmsAction(arms=robot.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                ]
            )
        )
        return statechart

    def build_simulation(self, world: World) -> MujocoSim:
        return MujocoSim(
            world=world,
            headless=self.headless,
            cone=self.friction_cone,
            impratio=self.impedance_ratio,
            noslip_iterations=self.no_slip_iterations,
        )


# %% running the demonstration


def main(
    headless: bool = False,
    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor,
    collision_avoidance: bool = False,
    event_segmentation: bool = True,
    debug: bool = False,
    lab_parts_movable: bool = False,
) -> None:
    """
    Run the demonstration.

    :param headless: Whether to run the simulation without a viewer.
    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :param collision_avoidance: Whether the statechart avoids collisions.
    :param event_segmentation: Whether SegMind segments the run into events.
    :param debug: Whether to run in debug mode, publishing every copy of the world a
        candidate is tried in.
    :param lab_parts_movable: Whether the drawers and doors of the lab can move.
    """
    ThrowAwayDemonstration(
        lab_parts_movable=lab_parts_movable,
        executor_type=executor_type,
        collision_avoidance=collision_avoidance,
        event_segmentation=event_segmentation,
        debug=debug,
        headless=headless,
    ).run()


if __name__ == "__main__":
    main()
