"""
Demonstrations in which the HSR drives through the kitchen lab to the sofa.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import mujoco

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.pumas_navigation import PumasNavigation
from coraplex.robot_plans.actions.core.navigation import (
    PathPlanningNavigateAction,
)
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
    Pose2D,
)
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor, Sofa
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World

# %% driving to the sofa


@dataclass(kw_only=True)
class SofaDemonstration(RobotDemonstration, ABC):
    """
    The HSR drives through the kitchen lab to the sofa, around the furniture.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that drives to the sofa.
    """

    collision_avoidance: bool = True
    """
    Whether the robot keeps away from the furniture and from itself.
    """

    scene_joints_movable: bool = False
    """
    Whether the doors and drawers of the kitchen can move, which the plan does not need
    and which slows the physics down.
    """

    integrator: mujoco.mjtIntegrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    """
    How MuJoCo integrates the physics of a physically simulated run.
    """

    clearance: float = 0.05
    """
    How far, beyond its radius, the planned path keeps the base from obstacles, in
    meters.
    """

    scanning_distance: float = 1.5
    """
    How far in front of the front edge of the sofa the robot's base stands after the
    drive, in meters.
    """

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(x=0.4)
    )
    """
    Where the robot starts in a simulated run: just inside the door in the south wall of
    the kitchen lab, facing into the room.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            odometry_T_robot_start=self.robot_start,
        )

    def is_scene_populated(self, world: World) -> bool:
        return all(
            world.is_kinematic_structure_entity_in_world_by_name(seat.value)
            for seat in SofaSeat
        )

    def acquire_world(self) -> World:
        """
        As :meth:`RobotDemonstration.acquire_world`, where the whole robot may touch the
        floors, which its base is always closer to than collision avoidance allows.
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

    def build_simulation(self, world: World) -> MujocoSim:
        """
        As :meth:`RobotDemonstration.build_simulation`, stepped by :attr:`integrator`.
        """
        return MujocoSim(
            world=world, headless=self.headless, integrator=self.integrator
        )

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [
            RobotAccess(
                robot=world.get_semantic_annotations_by_type(self.used_robot)[0]
            ),
        ]

    def scanning_pose(self, *, world: World) -> Pose:
        """
        :param world: The world holding the sofa.
        :return: Where the robot's base stands after the drive:
            :attr:`scanning_distance` in front of the sofa's front edge, facing it.
        """
        [sofa] = world.get_semantic_annotations_by_type(Sofa)
        sofa_front = (
            sofa.root.collision.as_bounding_box_collection_in_frame(sofa.root)
            .bounding_box()
            .min_x
        )
        world_T_scanning_pose = (
            sofa.root.global_transform
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=sofa_front - self.scanning_distance
            )
        ).pose
        return Pose.from_xyz_rpy(
            x=world_T_scanning_pose.x,
            y=world_T_scanning_pose.y,
            yaw=world_T_scanning_pose.yaw,
            reference_frame=world.root,
        )

    def drive_to_sofa(self, *, world: World) -> PathPlanningNavigateAction:
        """
        :param world: The world holding the sofa.
        :return: The drive to :meth:`scanning_pose` around the furniture.
        """
        return PathPlanningNavigateAction(
            planar_goal_pose=Pose2D.from_pose(self.scanning_pose(world=world)),
            obstacle_clearance=self.clearance,
        )

    def departure_nodes(self, *, context: StatechartContext) -> list[StatechartNode]:
        """
        :param context: The context the nodes run in.
        :return: What the robot does before it drives: park the arm.
        """
        return [
            ParkArmsAction(
                arms=context.require_extension(
                    extension_type=RobotAccess
                ).robot.all_arms
            )
        ]

    @staticmethod
    def build_sequence(
        *, context: StatechartContext, nodes: list[StatechartNode]
    ) -> Statechart:
        """
        :param context: The context the statechart is built in.
        :param nodes: The nodes to perform one after the other.
        :return: A statechart performing ``nodes`` in sequence.
        """
        statechart = Statechart(context=context)
        statechart.add_node(node=Sequence(nodes=nodes))
        return statechart


# %% scanning the sofa


@dataclass(kw_only=True)
class SofaScanningDemonstration(SofaDemonstration, ABC):
    """
    The HSR drives to the sofa and scans its seat places for people sitting on them.

    Afterwards the world holds a
    :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Human` on
    every seat place a person was seen on, and none on the others.
    """

    drives_with_pumas: bool = False
    """
    Whether the Pumas navigation of the lab drives the real robot to the sofa, between
    two runs of the executor, instead of the plan.

    .. note:: Giskard commands the base on every control cycle of a run, so Pumas only
        drives while no run is going on.
    """

    @abstractmethod
    def scanning_nodes(self, *, context: StatechartContext) -> list[StatechartNode]:
        """
        :param context: The context the nodes run in.
        :return: What the robot does in front of the sofa.
        """

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Do :meth:`departure_nodes`, drive to :meth:`scanning_pose` and do
        :meth:`scanning_nodes`.
        """
        return self.build_sequence(
            context=context,
            nodes=[
                *self.departure_nodes(context=context),
                self.drive_to_sofa(world=context.world),
                *self.scanning_nodes(context=context),
            ],
        )

    def perform_plan(self, world: World) -> None:
        """
        As :meth:`RobotDemonstration.perform_plan`, unless :attr:`drives_with_pumas`:

        then :meth:`departure_nodes` and :meth:`scanning_nodes` are two runs, with Pumas
        driving the base to :meth:`scanning_pose` between them.
        """
        if not self.drives_with_pumas:
            super().perform_plan(world)
            return
        self.execute_statechart(
            world,
            lambda context: self.build_sequence(
                context=context, nodes=self.departure_nodes(context=context)
            ),
        )
        PumasNavigation(
            node=self.ros_node,
            robot=world.get_semantic_annotations_by_type(self.used_robot)[0],
        ).drive_to(self.scanning_pose(world=world))
        self.execute_statechart(
            world,
            lambda context: self.build_sequence(
                context=context, nodes=self.scanning_nodes(context=context)
            ),
        )
