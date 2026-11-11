"""
The HSR drives from the kitchen of the kitchen lab to the sofa, looks at it, and says
which of its seat places are free; see the README for how to run it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    PlanExecutor,
)
from coraplex.robot_plans.actions.core.misc import DetectSeatsAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from coraplex.ros_session import RobotDemonstrationRosSession
from coraplex.scene_report_command import RobotSpeech
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Sofa
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World

# %% the spoken answer


class FreeSeatSentence(StrEnum):
    """
    The sentences the robot says about the free seat places of the sofa.
    """

    NONE = "No seat on the sofa is free."
    """
    Every seat place has someone sitting on it.
    """

    ONE = "The {seats} seat of the sofa is free."
    """
    Exactly one seat place is free.
    """

    SEVERAL = "The {seats} seats of the sofa are free."
    """
    More than one seat place is free.
    """

    @classmethod
    def describe(cls, free_seats: list[SofaSeat]) -> str:
        """
        :param free_seats: The free seat places, from left to right.
        :return: The sentence naming ``free_seats``, such as *The left and right seats
            of the sofa are free.*
        """
        seat_names = [seat.name.lower() for seat in free_seats]
        match seat_names:
            case []:
                return cls.NONE
            case [seat_name]:
                return cls.ONE.format(seats=seat_name)
            case [*leading, last]:
                return cls.SEVERAL.format(seats=f"{', '.join(leading)} and {last}")


# %% the demonstration


@dataclass
class KitchenToSofaDemonstration(RobotDemonstration):
    """
    The HSR drives from the kitchen of the kitchen lab to the sofa and looks at it for
    people sitting on its seat places.

    Afterwards the world holds a
    :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Human` on
    every seat place a person was seen on, and none on the others.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that looks at the sofa.
    """

    ros_node_name: str = "hsr_kitchen_to_sofa_demo"
    """
    The name of the ROS node the demonstration runs as.
    """

    scanning_distance: float = 1.8
    """
    How far in front of the front edge of the sofa the robot's base stands to look at
    it, in meters; chosen so the camera is as far from the seat cushions as in the
    recordings of the sofa, about 2.15 m.
    """

    scanning_torso_state: TorsoState = TorsoState.LOW
    """
    The torso height the robot looks at the sofa from.
    """

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=2.3, y=-1.0, yaw=np.pi / 2
        )
    )
    """
    Where the robot starts in a simulated run: in the kitchen, between the counter and
    the kitchen island, facing the living room.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        """
        :return: The HSR at :attr:`robot_start`.
        """
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            odometry_T_robot_start=self.robot_start,
        )

    def is_scene_populated(self, world: World) -> bool:
        return all(
            world.is_kinematic_structure_entity_in_world_by_name(seat.value)
            for seat in SofaSeat
        )

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Do :meth:`departure_nodes`, drive to :meth:`scanning_pose` and do
        :meth:`scanning_nodes`.
        """
        return self.statechart_of(
            context,
            [
                *self.departure_nodes(context),
                NavigateAction(target_location=self.scanning_pose(context.world)),
                *self.scanning_nodes(context),
            ],
        )

    @staticmethod
    def statechart_of(
        context: StatechartContext, nodes: list[StatechartNode]
    ) -> Statechart:
        """
        :param context: The context the statechart is built in.
        :param nodes: The nodes to perform one after the other.
        :return: A statechart performing ``nodes`` in sequence.
        """
        statechart = Statechart(context=context)
        statechart.add_node(Sequence(nodes=nodes))
        return statechart

    def departure_nodes(self, context: StatechartContext) -> list[StatechartNode]:
        """
        :param context: The context the nodes run in.
        :return: What the robot does before it drives: park the arm.
        """
        return [
            ParkArmsAction(arms=context.require_extension(RobotAccess).robot.all_arms)
        ]

    def scanning_pose(self, world: World) -> Pose:
        """
        :param world: The world holding the sofa.
        :return: Where the robot's base stands to look at the sofa:
            :attr:`scanning_distance` in front of its front edge, facing it.
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

    def scanning_nodes(self, context: StatechartContext) -> list[StatechartNode]:
        """
        :param context: The context the nodes run in.
        :return: What the robot does in front of the sofa: put the torso at
            :attr:`scanning_torso_state`, look at the middle of the seat places for
            people sitting on them and lower the torso again.
        """
        world = context.world
        seat_middle_x, seat_middle_y, seat_middle_z = np.mean(
            [
                seat.region_in(world).global_transform.position.to_np()[:3]
                for seat in SofaSeat
            ],
            axis=0,
        )
        return [
            MoveTorsoAction(torso_state=self.scanning_torso_state),
            DetectSeatsAction(
                look_targets=[
                    Pose.from_xyz_rpy(
                        x=seat_middle_x,
                        y=seat_middle_y,
                        z=seat_middle_z,
                        reference_frame=world.root,
                    )
                ]
            ),
            MoveTorsoAction(torso_state=TorsoState.LOW),
        ]


# %% running it


@dataclass
class FreeSeatAnnouncement:
    """
    Says which seat places of the sofa are free after a run, printed and, on the real
    robot, through the HSR's voice.
    """

    speak: bool = False
    """
    Whether the sentence is also sent to the HSR's voice, which only the real robot has.
    """

    ros_node_name: str = "hsr_kitchen_to_sofa_speech"
    """
    The name of the ROS node the sentence is spoken through.
    """

    def announce(self, world: World) -> str:
        """
        :param world: The world after the run, whose sofa has a region for every seat
            place.
        :return: The sentence naming the free seat places of the sofa in ``world``.
        """
        sentence = FreeSeatSentence.describe(SofaSeat.free_in(world))
        print(sentence, flush=True)
        if self.speak:
            with RobotDemonstrationRosSession.start(self.ros_node_name) as session:
                RobotSpeech(node=session.node).say(sentence)
        return sentence


def main(
    demonstration: KitchenToSofaDemonstration | None = None,
    announcement: FreeSeatAnnouncement | None = None,
) -> World:
    """
    Run the demonstration and say which seat places of the sofa are free.

    :param demonstration: The demonstration to run, by default a kinematically simulated
        one whose perception the world model answers.
    :param announcement: How the free seat places are said, by default only printed.
    :return: The world after the run, holding a person on every seat place one was seen
        on.
    """
    demonstration = demonstration or KitchenToSofaDemonstration(
        executor_type=KinematicallySimulatedPlanExecutor
    )
    world = demonstration.run()
    (announcement or FreeSeatAnnouncement()).announce(world)
    return world
