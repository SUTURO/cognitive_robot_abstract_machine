"""
Robots handling a T-shirt on a table, simulated in MuJoCo, to show how the cloth
behaves; here a Tiago picks it up off the table by its shoulder and swings it from side
to side.

Run it as a script::
    python3 -m experiments.t_shirt_demo.demo
"""

from __future__ import annotations

import itertools
from abc import abstractmethod
from dataclasses import dataclass, field
from enum import StrEnum
from functools import partial

import mujoco
import numpy

from coraplex.datastructures.trajectory import PoseTrajectory
from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import (
    FollowToolCenterPointPathAction,
    MoveFingersToWidthAction,
    MoveManipulatorAction,
    ParkArmsAction,
    SetGripperAction,
)
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.monitors import CountSimulationTimeSeconds
from cramph.statechart import Statechart
from cramph.threaded_nodes import FunctionCall
from experiments.t_shirt_demo.exceptions import TShirtNotPinned
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
    Pose,
)
from physics_simulators.base_simulator import SimulatorCallbackResult
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.robot_part_mixins import HasTwoFingers
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm
from semantic_digital_twin.robots.tiago import TiagoMujoco
from semantic_digital_twin.semantic_annotations.semantic_annotations import TShirt
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.cloth import (
    Cotton,
    Fabric,
    GarmentFile,
)
from semantic_digital_twin.world_description.geometry import Color, Scale
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

# %% the scene


class SceneEntity(StrEnum):
    """
    The names of what the scene holds.
    """

    FLOOR = "floor"
    TABLE = "table"
    T_SHIRT = "t_shirt"


@dataclass
class TShirtOnTableDemonstration(RobotDemonstration):
    """
    A robot handles a T-shirt that starts on a table in front of it, in physics, since
    the T-shirt exists only there.
    """

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor
    """
    Performs the plan in physics, since the T-shirt exists only there.
    """

    table_distance: float = 0.6
    """
    How far in front of the robot's base the near edge of the table stands, in metres.
    """

    table_scale: Scale = field(default_factory=lambda: Scale(0.9, 1.4, 0.72))
    """
    The depth, width and height of the table.
    """

    t_shirt_mass: float = 0.255
    """
    The mass of the T-shirt, in kilograms, as RGBench states it.
    """

    t_shirt_garment: GarmentFile = GarmentFile.T_SHIRT
    """
    The mesh the T-shirt is made from.
    """

    t_shirt_fabric: Fabric = field(default_factory=Cotton)
    """
    The kind of cloth the T-shirt is made of.
    """

    t_shirt_color: Color = field(
        default_factory=lambda: Color(hue=0.6, saturation=0.7, value=0.8)
    )
    """
    The color of the T-shirt.
    """

    approach_height: float = 0.1
    """
    How far above where they close the fingertips come down from, in metres.
    """

    fingertip_clearance: float = 0.005
    """
    How far above the table top the fingertips stay at the lowest, in metres.
    """

    closing_duration: float = 1.5
    """
    How long the gripper is given to close on the T-shirt before it holds it, in seconds
    of simulated time: a motion counts as done once it is commanded, before the servos
    got there.
    """

    noslip_iterations: int = 0
    """
    How many iterations of MuJoCo's no-slip solver follow every step, which keep what a
    gripper holds by friction from creeping out of it.
    """

    friction_cone: mujoco.mjtCone = mujoco.mjtCone.mjCONE_PYRAMIDAL
    """
    The shape MuJoCo approximates the friction cone of a contact by.
    """

    frictional_to_normal_impedance_ratio: float = 1.0
    """
    How much harder MuJoCo resists a contact's sliding than its penetration, its
    ``impratio``.
    """

    @property
    @abstractmethod
    def world_T_t_shirt(self) -> HomogeneousTransformationMatrix:
        """
        Where the T-shirt starts out, as :attr:`GarmentFile.T_SHIRT` lays it out with
        its neckline at its origin and its hem along its x-axis.
        """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(
            objects=[
                BodySpecification.box(
                    SceneEntity.FLOOR,
                    Scale(4.0, 4.0, 0.02),
                    parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=-0.01),
                ),
                BodySpecification.box(
                    SceneEntity.TABLE,
                    self.table_scale,
                    parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=self.table_distance + self.table_scale.x / 2,
                        z=self.table_scale.z / 2,
                    ),
                ),
            ]
        )

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return bool(world.get_semantic_annotations_by_type(TShirt))

    def populate_scene(self, world: World) -> None:
        """
        Put the T-shirt where :attr:`world_T_t_shirt` places it.
        """
        t_shirt = TShirt.get_annotation_specification(
            SceneEntity.T_SHIRT,
            TShirt.get_default_root_kinematic_structure_entity_specification(),
            annotation_kwargs={"fabric": self.t_shirt_fabric},
        ).spawn(world, parent_T_self=self.world_T_t_shirt)
        with world.modify_world():
            t_shirt.add_cloth_sheet(
                surface=self.t_shirt_garment.create_surface(self.world_T_t_shirt),
                mass=self.t_shirt_mass,
                color=self.t_shirt_color,
            )

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_simulation(self, world: World) -> MujocoSim:
        """
        :meth:`RobotDemonstration.build_simulation`, integrated and solved the way cloth
        simulates at an affordable cost, with this demonstration's friction settings.
        """
        return MujocoSim(
            world=world,
            headless=self.headless,
            integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
            solver=mujoco.mjtSolver.mjSOL_CG,
            noslip_iterations=self.noslip_iterations,
            cone=self.friction_cone,
            impratio=self.frictional_to_normal_impedance_ratio,
        )

    @staticmethod
    def _gripper_pose_in(
        world_T_frame: HomogeneousTransformationMatrix,
        x: float,
        y: float,
        z: float,
        gripper: HasTwoFingers,
    ) -> Pose:
        """
        :param world_T_frame: The frame the gripper's position is given in.
        :param x: Where the gripper is to be along the frame's x-axis, in metres.
        :param y: Where the gripper is to be along the frame's y-axis, in metres.
        :param z: How high above the frame the gripper is to be, in metres.
        :param gripper: The gripper to place.
        :return: The gripper there in the world, approaching straight down and closing
            its fingers along the frame's x-axis.
        """
        approach = gripper.approach_axis.to_np()[:3]
        closing = gripper.closing_axis.to_np()[:3]
        tool_axes = numpy.column_stack(
            [approach, closing, numpy.cross(approach, closing)]
        )
        down, forward = numpy.array([0.0, 0.0, -1.0]), numpy.array([1.0, 0.0, 0.0])
        frame_axes = numpy.column_stack([down, forward, numpy.cross(down, forward)])
        frame_T_tool = numpy.eye(4)
        frame_T_tool[:3, :3] = frame_axes @ tool_axes.T
        frame_T_tool[:3, 3] = [x, y, z]
        return (world_T_frame @ HomogeneousTransformationMatrix(frame_T_tool)).pose

    @staticmethod
    def _move_gripper_to(target_pose: Pose, arm: Arm) -> MoveManipulatorAction:
        """
        :param target_pose: Where the gripper is to be.
        :param arm: The arm moving the gripper.
        :return: The action moving the gripper there, allowed to touch the T-shirt and
            the table.
        """
        return MoveManipulatorAction(
            target_pose=target_pose,
            end_effector=arm.end_effector,
            allow_gripper_collision=True,
        )

    @staticmethod
    def _fingertip_reach(gripper: HasTwoFingers) -> float:
        """
        :param gripper: The gripper pointing down at the T-shirt.
        :return: How far ahead of the gripper's tool frame its fingertips reach along
            the direction it approaches in.
        """
        approach = gripper.approach_axis.to_np()[:3]
        reach = -numpy.inf
        for tip in gripper.fingertips.both:
            box = tip.collision.as_bounding_box_collection_in_frame(
                gripper.tool_frame
            ).bounding_box()
            for corner in itertools.product(
                [box.min_x, box.max_x], [box.min_y, box.max_y], [box.min_z, box.max_z]
            ):
                reach = max(reach, float(numpy.dot(corner, approach)))
        return reach


# %% swinging it


@dataclass
class TShirtSwingDemonstration(TShirtOnTableDemonstration):
    """
    The Tiago takes the T-shirt lying spread out on the table in front of it by the
    shoulder, turns away from the table to pull it off, and swings it from side to side.

    The gripper holds the T-shirt by pinning the vertices between its fingertips to it.
    """

    used_robot: type[AbstractRobot] = TiagoMujoco
    """
    The Tiago as MuJoCo's own model describes it, whose hand can hold something.
    """

    ros_node_name: str = "t_shirt_swing_demo"
    """
    Name of the node a run registers.
    """

    t_shirt_P_left_shoulder: Point3 = field(
        default_factory=lambda: Point3(0.024, 0.176, 0.015)
    )
    """
    The middle of the seam of the T-shirt's left shoulder, as
    :attr:`GarmentFile.T_SHIRT` lays it out.
    """

    t_shirt_margin: float = 0.05
    """
    How far behind the near edge of the table the T-shirt's neckline lies, in metres.
    """

    t_shirt_drop_height: float = 0.01
    """
    How far above the table top the T-shirt is laid out, to settle onto it, in metres.
    """

    pinning_radius: float = 0.03
    """
    How far from the point the fingertips meet at a vertex of the T-shirt may lie to be
    held by the gripper, in metres.
    """

    turning_angle: float = numpy.pi / 2
    """
    How far the robot turns to its left, away from the table, with the T-shirt held, in
    radians.
    """

    hanging_distance: float = 0.4
    """
    How far in front of the robot's base the gripper holds the T-shirt while swinging
    it, in metres.
    """

    hanging_height: float = 1.0
    """
    How high the gripper holds the T-shirt while swinging it, in metres.
    """

    swing_amplitude: float = 0.2
    """
    How far to each side of the shoulder's side of the robot the gripper swings, in
    metres.
    """

    swing_count: int = 3
    """
    How often the gripper swings to the left and back to the right.
    """

    watching_duration: float = 3.0
    """
    How long the T-shirt hangs after the last swing, in seconds of simulated time.
    """

    @property
    def world_T_t_shirt(self) -> HomogeneousTransformationMatrix:
        """
        The T-shirt lies spread out on the table, its neckline towards the robot and its
        sleeves to the sides.
        """
        return HomogeneousTransformationMatrix.from_xyz_rpy(
            x=self.table_distance + self.t_shirt_margin,
            z=self.table_scale.z + self.t_shirt_drop_height,
        )

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arms, take the T-shirt by the shoulder on the robot's left with the arm
        closest to it, turn away from the table, swing the T-shirt and let it hang.
        """
        world = context.world
        robot = context.require_extension(RobotAccess).robot
        world_T_table_side = HomogeneousTransformationMatrix(reference_frame=world.root)
        shoulder_pose = self._shoulder_on_the_left(world)
        arm = min(
            robot.all_arms,
            key=lambda arm: float(
                arm.end_effector.tool_frame.global_pose.position.euclidean_distance(
                    shoulder_pose.position
                )
            ),
        )
        gripper = arm.end_effector
        fingertip_reach = self._fingertip_reach(gripper)
        shoulder_x, shoulder_y = float(shoulder_pose.x), float(shoulder_pose.y)
        above_shoulder = self._gripper_pose_in(
            world_T_table_side,
            shoulder_x,
            shoulder_y,
            self.table_scale.z + self.approach_height + fingertip_reach,
            gripper,
        )
        world_T_turned_base = (
            robot.root.global_pose.homogeneous_matrix
            @ HomogeneousTransformationMatrix.from_xyz_rpy(yaw=self.turning_angle)
        )
        base_y_of_shoulder = float(world.transform(shoulder_pose, robot.root).y)
        swing = [
            self._gripper_pose_in(
                world_T_turned_base,
                self.hanging_distance,
                base_y_of_shoulder + side,
                self.hanging_height,
                gripper,
            )
            for side in [
                *[self.swing_amplitude, -self.swing_amplitude] * self.swing_count,
                0.0,
            ]
        ]
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    SetGripperAction(
                        configuration=gripper.default_configuration(GripperState.OPEN)
                    ),
                    self._move_gripper_to(above_shoulder, arm),
                    self._move_gripper_to(
                        self._gripper_pose_in(
                            world_T_table_side,
                            shoulder_x,
                            shoulder_y,
                            self.table_scale.z
                            + self.fingertip_clearance
                            + fingertip_reach,
                            gripper,
                        ),
                        arm,
                    ),
                    MoveFingersToWidthAction(
                        gripper=gripper, width=self.t_shirt_fabric.thickness
                    ),
                    CountSimulationTimeSeconds(seconds=self.closing_duration),
                    FunctionCall(
                        function=partial(
                            self._pin_t_shirt_to, gripper.tool_frame, fingertip_reach
                        )
                    ),
                    self._move_gripper_to(above_shoulder, arm),
                    self._move_gripper_to(
                        self._gripper_pose_in(
                            robot.root.global_pose.homogeneous_matrix,
                            self.hanging_distance,
                            base_y_of_shoulder,
                            self.hanging_height,
                            gripper,
                        ),
                        arm,
                    ),
                    NavigateAction(target_location=world_T_turned_base.pose),
                    FollowToolCenterPointPathAction(
                        target_locations=PoseTrajectory(poses=swing), arm=arm
                    ),
                    CountSimulationTimeSeconds(seconds=self.watching_duration),
                ]
            )
        )
        return statechart

    def _shoulder_on_the_left(self, world: World) -> Pose:
        """
        :param world: The world holding the T-shirt.
        :return: Where the vertex of the T-shirt closest to the middle of its shoulder on
            the robot's left lies at the start.
        """
        world_P_shoulder = self.world_T_t_shirt @ self.t_shirt_P_left_shoulder
        [t_shirt] = world.get_semantic_annotations_by_type(TShirt)
        sheet = t_shirt.cloth_sheet
        _, [closest] = sheet.starting_surface.nearest.vertex(
            [world_P_shoulder.to_np()[:3]]
        )
        return Pose.from_xyz_rpy(
            *sheet.starting_surface.vertices[closest], reference_frame=world.root
        )

    def _pin_t_shirt_to(
        self, tool_frame: KinematicStructureEntity, fingertip_reach: float
    ) -> None:
        """
        Pin the vertices of the T-shirt within :attr:`pinning_radius` of where the
        fingertips meet, ``fingertip_reach`` ahead of ``tool_frame``, to it.

        :raises TShirtNotPinned: If no vertex lies that close.
        """
        result = self.simulation.simulator.pin_flex_vertices(
            flex_name=SceneEntity.T_SHIRT,
            body_name=tool_frame.name.name,
            body_P_centre=numpy.array([fingertip_reach, 0.0, 0.0]),
            radius=self.pinning_radius,
        )
        if (
            result.type
            is not SimulatorCallbackResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL
        ):
            raise TShirtNotPinned(
                t_shirt_name=SceneEntity.T_SHIRT,
                holding_body_name=tool_frame.name.name,
                reason=result.info,
            )


# %% running it


def main(headless: bool = False) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :return: The world the T-shirt was swung in.
    """
    return TShirtSwingDemonstration(headless=headless).run()


if __name__ == "__main__":
    main()
