"""
The HSR starts at the door of the kitchen lab, drives to the sofa, looks at each of its
seat places and reports which are free; see the README for how to run it.

.. warning:: The real robot needs a perception pipeline answering
    :data:`~coraplex.perception.ROBOKUDO_QUERY_ACTION_NAME`::

        python3 -m robokudo.scripts.main _ae seat_place_occupancy
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    PlanExecutor,
)
from coraplex.robot_plans.actions.core.misc import DetectSeatsAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Human,
    Sofa,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection

# %% the demonstration


@dataclass
class DoorToSofaDemonstration(RobotDemonstration):
    """
    The HSR drives from the door of the kitchen lab to the sofa and scans its seat
    places for people sitting on them.

    Afterwards the world holds a
    :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Human` on
    every seat place a person was seen on, and none on the others.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that scans the sofa.
    """

    ros_node_name: str = "hsr_door_to_sofa_demo"
    """
    The name of the ROS node the demonstration runs as.
    """

    scanning_distance: float = 1.5
    """
    How far in front of the front edge of the sofa the robot's base stands to scan it,
    in meters.
    """

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(x=0.4)
    )
    """
    Where the robot starts in a simulated run: just inside the door in the south wall of
    the kitchen lab, facing into the room.
    """

    occupied_seats: list[SofaSeat] = field(default_factory=list)
    """
    The seat places a person sits on in a simulated run.
    """

    person_size: Scale = field(default_factory=lambda: Scale(0.4, 0.4, 0.8))
    """
    The extents of the box a person sitting on the sofa is shaped as, in meters.
    """

    person_color: Color = field(
        default_factory=lambda: Color(hue=0.0, saturation=0.8, value=0.9)
    )
    """
    The color a person sitting on the sofa is drawn in.
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

    def populate_scene(self, world: World) -> None:
        """
        Sit a person on each of the :attr:`occupied_seats`.
        """
        for seat in self.occupied_seats:
            name = f"person_on_{seat.value}"
            seat.place_person_in(
                world,
                Human.get_annotation_specification(
                    name,
                    BodySpecification.box(name, self.person_size, self.person_color),
                ),
            )

    def build_simulation(self, world: World) -> MujocoSim:
        """
        As :meth:`RobotDemonstration.build_simulation`, with floors that do not collide,
        since the wheels, which sit slightly inside the floors, would catch on the edge
        between two of them.
        """
        for floor in world.get_semantic_annotations_by_type(Floor):
            floor.root.collision = ShapeCollection()
        return super().build_simulation(world)

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arm, drive :attr:`scanning_distance` in front of the sofa's front edge,
        facing it, raise the torso, look for people on each of its seat places and lower
        the torso again.
        """
        world = context.world
        hsrb = context.require_extension(RobotAccess).robot
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
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=hsrb.all_arms),
                    NavigateAction(
                        target_location=Pose.from_xyz_rpy(
                            x=world_T_scanning_pose.x,
                            y=world_T_scanning_pose.y,
                            yaw=world_T_scanning_pose.yaw,
                            reference_frame=world.root,
                        )
                    ),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    DetectSeatsAction(
                        look_targets=[
                            Pose.from_xyz_rpy(reference_frame=seat.region_in(world))
                            for seat in SofaSeat
                        ]
                    ),
                    MoveTorsoAction(torso_state=TorsoState.LOW),
                ]
            )
        )
        return statechart


def occupied_seats_from_command_line() -> list[SofaSeat]:
    """
    :return: The seat places named on the command line, such as ``LEFT RIGHT``.
    """
    return [SofaSeat[argument.upper()] for argument in sys.argv[1:]]


def main(
    executor_type: type[PlanExecutor] = KinematicallySimulatedPlanExecutor,
    occupied_seats: list[SofaSeat] | None = None,
    headless: bool = False,
) -> World:
    """
    Run the demonstration and print which seat places of the sofa are free.

    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :param occupied_seats: The seat places a person sits on in a simulated run.
    :param headless: Whether the physics of a physically simulated run steps without a
        viewer.
    :return: The world after the run, holding a person on every seat place one was seen
        on.
    """
    world = DoorToSofaDemonstration(
        executor_type=executor_type,
        occupied_seats=occupied_seats or [],
        headless=headless,
    ).run()
    free_seats = SofaSeat.free_in(world)
    print("Free seats:", ", ".join(seat.name for seat in free_seats) or "none")
    return world
