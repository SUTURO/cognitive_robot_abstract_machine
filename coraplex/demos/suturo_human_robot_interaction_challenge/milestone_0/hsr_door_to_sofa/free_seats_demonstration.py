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
from enum import StrEnum

import numpy as np

from coraplex.robot_plans.actions.core.misc import DetectSeatsAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from coraplex.sofa_demonstrations import SofaScanningDemonstration
from cramph.context import StatechartContext
from cramph.node import StatechartNode
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color, Scale

# %% the answer


class FreeSeatSentence(StrEnum):
    """
    The sentences that name the free seat places of the sofa.
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


@dataclass(kw_only=True)
class DoorToSofaDemonstration(SofaScanningDemonstration):
    """
    The HSR drives from the door of the kitchen lab to the sofa and scans its seat
    places for people sitting on them.
    """

    ros_node_name: str = "hsr_door_to_sofa_demo"
    """
    The name of the ROS node the demonstration runs as.
    """

    scanning_torso_state: TorsoState = TorsoState.HIGH
    """
    The torso height the robot looks at the sofa from.
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

    def populate_scene(self, world: World) -> None:
        """
        As :meth:`SofaScanningDemonstration.populate_scene`, with a person on each of
        the :attr:`occupied_seats`.
        """
        super().populate_scene(world)
        for seat in self.occupied_seats:
            name = f"person_on_{seat.value}"
            seat.place_person_in(
                world,
                Human.get_annotation_specification(
                    name,
                    BodySpecification.box(name, self.person_size, self.person_color),
                ),
            )

    def look_targets(self, *, world: World) -> list[Pose]:
        """
        :param world: The world holding the sofa.
        :return: What the robot looks at, one after the other, to see the seat places:
            each of them.
        """
        return [
            Pose.from_xyz_rpy(reference_frame=seat.region_in(world))
            for seat in SofaSeat
        ]

    def scanning_nodes(self, *, context: StatechartContext) -> list[StatechartNode]:
        """
        Put the torso at :attr:`scanning_torso_state`, look at the :meth:`look_targets`
        for people sitting on the seat places and lower the torso again.
        """
        return [
            MoveTorsoAction(torso_state=self.scanning_torso_state),
            DetectSeatsAction(look_targets=self.look_targets(world=context.world)),
            MoveTorsoAction(torso_state=TorsoState.LOW),
        ]


@dataclass(kw_only=True)
class OneLookDoorToSofaDemonstration(DoorToSofaDemonstration):
    """
    The HSR drives from the door of the kitchen lab to the sofa and sees all of its seat
    places in one look, as the recordings of the sofa do.
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

    def look_targets(self, *, world: World) -> list[Pose]:
        """
        Differs from :meth:`DoorToSofaDemonstration.look_targets` in looking once, at
        the middle of the seat places.
        """
        seat_middle_x, seat_middle_y, seat_middle_z = np.mean(
            [
                seat.region_in(world).global_transform.position.to_np()[:3]
                for seat in SofaSeat
            ],
            axis=0,
        )
        return [
            Pose.from_xyz_rpy(
                x=seat_middle_x,
                y=seat_middle_y,
                z=seat_middle_z,
                reference_frame=world.root,
            )
        ]


# %% running it


def occupied_seats_from_command_line() -> list[SofaSeat]:
    """
    :return: The seat places named on the command line, such as ``LEFT RIGHT``.
    """
    return [SofaSeat[argument.upper()] for argument in sys.argv[1:]]


def main(demonstration: DoorToSofaDemonstration) -> World:
    """
    Run the demonstration and print which seat places of the sofa are free.

    :param demonstration: The demonstration to run.
    :return: The world after the run, holding a person on every seat place one was seen
        on.
    """
    world = demonstration.run()
    print(FreeSeatSentence.describe(SofaSeat.free_in(world)), flush=True)
    return world
