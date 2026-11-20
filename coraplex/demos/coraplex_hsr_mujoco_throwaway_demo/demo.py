"""
The HSR takes a milk carton from a counter and throws it into a trash can in MuJoCo.

At the trash can the robot stands on the side it comes from, clear of the trash can and
within reach of its opening, facing it, and lets go of the milk above the opening.

Run it from this folder::

    python demo.py
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import StrEnum
from pathlib import Path

import mujoco
import numpy as np

from krrood.patterns.path_enumeration import PathEnumeration

from coraplex.demonstrations import RobotDemonstration
from coraplex.locations.locations import ReachabilityLocation
from coraplex.plans.context_extensions import RobotAccess, StatementGrounding
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Milk,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Point3, Pose
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.geometry import Box, Mesh, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
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

    TRASHCAN = (
        Path(__file__).resolve().parents[2] / "resources" / "objects" / "trashcan.stl"
    )
    """
    The mesh of the trash can.
    """


class SceneBody(StrEnum):
    """
    The names of the scene's bodies.
    """

    FLOOR = "floor"
    """
    The floor the robot drives on.
    """

    COUNTER = "counter"
    """
    The counter the milk starts on.
    """

    MILK = "milk"
    """
    The milk carton.
    """

    TRASHCAN = "trashcan"
    """
    The trash can the milk is thrown into.
    """


# %% the demonstration


@dataclass
class MilkTransportDemonstration(RobotDemonstration):
    """
    The HSR takes the milk from the counter in front of it and throws it into the trash
    can behind it.
    """

    used_robot: type[AbstractRobot] = HSRB

    ros_node_name: str = "hsr_milk_transport_demo"

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor

    floor: SemanticAnnotationWithRootSpecification[Floor] = field(
        default_factory=lambda: Floor.get_annotation_specification(
            SceneBody.FLOOR,
            BodySpecification.box(
                SceneBody.FLOOR,
                Scale(4.0, 4.0, 0.02),
                parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=-0.01),
            ),
        )
    )
    """
    The floor the robot drives on, which the robot's standing poses are sampled on.
    """

    counter: BodySpecification = field(
        default_factory=lambda: BodySpecification.box(
            SceneBody.COUNTER,
            Scale(0.6, 1.0, 0.75),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, z=0.375),
        )
    )
    """
    The counter the milk starts on, in front of the robot.
    """

    trashcan: BodySpecification = field(
        default_factory=lambda: MilkTransportDemonstration._hollow_trashcan()
    )
    """
    The trash can standing on the floor behind the robot.
    """

    milk_depth: float = 0.15
    """
    How far behind the near edge of the counter the milk starts, in meters.
    """

    standing_distance: float = 0.5
    """
    How far from the milk the robot's base stands to take it, in meters.
    """

    drop_clearance: float = 0.05
    """
    How far above the trash can's rim the bottom of the milk is when it is let go, in
    meters.
    """

    grasp_height: float = 0.05
    """
    How far above the milk's origin the hand takes hold of it, in meters.
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

    no_slip_iterations: int = 10
    """
    Passes spent removing the slip soft contacts leave, so a held carton stays where the
    fingers hold it.
    """

    @classmethod
    def _hollow_trashcan(
        cls,
        width: float = 0.31,
        depth: float = 0.32,
        height: float = 0.39,
        wall_thickness: float = 0.01,
    ) -> BodySpecification:
        """
        :param width: The outer extent of the trash can along x, in meters.
        :param depth: The outer extent of the trash can along y, in meters.
        :param height: The height of the trash can, in meters.
        :param wall_thickness: The thickness of its walls and bottom, in meters.
        :return: The trash can, shown as its mesh but colliding as an open-top box of
            thin walls, since MuJoCo would collide with the mesh's closed convex hull.
        """
        half_height = height / 2
        walls = [
            Box(
                scale=Scale(width, depth, wall_thickness),
                origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=wall_thickness / 2
                ),
            ),
            *(
                Box(
                    scale=Scale(wall_thickness, depth, height),
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=side * (width - wall_thickness) / 2, z=half_height
                    ),
                )
                for side in (-1, 1)
            ),
            *(
                Box(
                    scale=Scale(width, wall_thickness, height),
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        y=side * (depth - wall_thickness) / 2, z=half_height
                    ),
                )
                for side in (-1, 1)
            ),
        ]
        return BodySpecification(
            SceneBody.TRASHCAN,
            ShapeCollection(walls),
            visual_shapes=Mesh(
                filename=str(SceneFile.TRASHCAN),
                # The mesh is in millimetres, with its base off-centre around
                # (125, 105), enlarged 2.25-fold.
                scale=Scale(0.00225, 0.00225, 0.00225),
                origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=-0.28125, y=-0.23625
                ),
            ).as_shape_collection(),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=-1.0),
        )

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(objects=[self.floor, self.counter, self.trashcan])

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneBody.MILK)

    def populate_scene(self, world: World) -> None:
        """
        Stand the milk on the counter, free to be picked up.
        """
        milk = SemanticAnnotationWithRootSpecification(
            name=SceneBody.MILK,
            semantic_annotation_type=Milk,
            root_specification=BodySpecification.mesh(
                SceneBody.MILK,
                str(SceneFile.MILK),
                connection_specification=Connection6DoFSpecification(),
            ),
        ).spawn(world)
        ContactParameters.create_for_grasped_object().apply_to([milk.root])
        milk.root.parent_connection.origin = self._milk_pose_on(
            world, world.get_body_by_name(SceneBody.COUNTER), milk
        ).homogeneous_matrix

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [
            RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0]),
            StatementGrounding(),
        ]



    def _milk_pose_on(self, world: World, surface: Body, milk: Milk) -> Pose:
        """
        :param world: The world the surface stands in.
        :param surface: The counter.
        :param milk: The milk.
        :return: Where the milk stands on ``surface``, :attr:`milk_depth` behind the
            edge nearest the robot's start and turned the way the robot faces it.

        .. note:: The counter stands in front of the robot's start, along the world's
            x-axis.
        """
        surface_box = surface.collision.as_bounding_box_collection_in_frame(
            world.root
        ).bounding_box()
        milk_box = milk.root.collision.as_bounding_box_collection_in_frame(
            milk.root
        ).bounding_box()
        return Pose.from_xyz_rpy(
            x=surface_box.min_x + self.milk_depth,
            z=surface_box.max_z - milk_box.min_z,
            reference_frame=world.root,
        )

    def _drop_point(self, world: World, trashcan: Body, milk: Milk) -> Point3:
        """
        :param world: The world the trash can stands in.
        :param trashcan: The trash can the milk is thrown into.
        :param milk: The milk.
        :return: Where the milk's origin is when it is let go, in the world frame:
            centred above the trash can's opening, :attr:`drop_clearance` above its rim.
        """
        trashcan_box = trashcan.collision.as_bounding_box_collection_in_frame(
            world.root
        ).bounding_box()
        milk_box = milk.root.collision.as_bounding_box_collection_in_frame(
            milk.root
        ).bounding_box()
        return Point3(
            (trashcan_box.min_x + trashcan_box.max_x) / 2,
            (trashcan_box.min_y + trashcan_box.max_y) / 2,
            trashcan_box.max_z + self.drop_clearance - milk_box.min_z,
            reference_frame=world.root,
        )

    def standing_position_at(
        self,
        context: StatechartContext,
        drop_point: Point3,
        arm: Arm,
        approach_from: Pose,
    ) -> Pose:
        """
        :param context: The context of the plan, whose robot throws the milk away.
        :param drop_point: Where the milk is let go, in the world frame.
        :param arm: The arm that holds the milk over the trash can.
        :param approach_from: Where the robot stands before it goes to the trash can,
            in the world frame.
        :return: Of the poses clear of the surroundings from which `arm` reaches
            `drop_point`, the one closest to standing at the arm's reach distance off
            `drop_point` on the side of `approach_from`.
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
    def orient_towards(pose: Pose, target: Body) -> Pose:
        """
        :param pose: The pose to turn, in the world frame.
        :param target: The body to face.
        :return: `pose` at its position, turned level about the vertical so that its
            x-axis points at the origin of `target`.
        """
        offset = target.global_pose.position.to_np()[:2] - pose.position.to_np()[:2]
        return Pose.from_xyz_rpy(
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z),
            yaw=float(np.arctan2(offset[1], offset[0])),
            reference_frame=pose.reference_frame,
        )

    def _standing_pose_facing(self, milk_pose: Pose) -> Pose:
        """
        :param milk_pose: Where the milk stands.
        :return: Where the robot's base stands on the floor to reach the milk there,
            :attr:`standing_distance` before it and facing it.
        """
        return (
            milk_pose.homogeneous_matrix
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=-self.standing_distance, z=-float(milk_pose.z)
            )
        ).pose

    def _grasp_from(
        self, world: World, standing_pose: Pose, milk: Milk
    ) -> GraspCandidate:
        """
        :param world: The world the milk stands in.
        :param standing_pose: Where the robot stands to take the milk.
        :param milk: The milk.
        :return: The milk's grasp closest to the robot standing there, raised
            :attr:`grasp_height` above the milk's origin.
        """
        [closest] = GraspsByCloseness(milk, standing_pose, number_of_grasps=1)()
        milk_T_raised_grasp = (
            HomogeneousTransformationMatrix.from_xyz_rpy(
                z=self.grasp_height, reference_frame=milk.root
            )
            @ world.transform(closest.grasp_pose, milk.root).homogeneous_matrix
        )
        return replace(closest, grasp_pose=milk_T_raised_grasp.pose)

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Raise the torso, park the arm, take the milk from the counter, carry it to the
        trash can and let go of it above the opening.
        """
        world = context.world
        hsrb = context.require_extension(RobotAccess).robot
        [milk] = world.get_semantic_annotations_by_type(Milk)
        [arm] = hsrb.all_arms
        trashcan = world.get_body_by_name(SceneBody.TRASHCAN)
        taking_pose = self._standing_pose_facing(
            self._milk_pose_on(world, world.get_body_by_name(SceneBody.COUNTER), milk)
        )
        drop_point = self._drop_point(world, trashcan, milk)
        throwing_pose = self.orient_towards(
            self.standing_position_at(context, drop_point, arm, taking_pose), trashcan
        )
        target = Pose.from_xyz_rpy(
            x=float(drop_point.x),
            y=float(drop_point.y),
            z=float(drop_point.z),
            yaw=float(throwing_pose.yaw),
            reference_frame=drop_point.reference_frame,
        )
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=hsrb.all_arms),
                    NavigateAction(target_location=taking_pose),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    PickUpAction(
                        grasp=self._grasp_from(world, taking_pose, milk), arm=arm
                    ),
                    ParkArmsAction(arms=hsrb.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                    NavigateAction(target_location=throwing_pose),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    PlaceAction(object_designator=milk, target_location=target),
                    ParkArmsAction(arms=hsrb.all_arms),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
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


def main(headless: bool = False) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :return: The world the milk was thrown away in.
    """
    return MilkTransportDemonstration(headless=headless).run()


if __name__ == "__main__":
    main()
