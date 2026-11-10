"""
The HSR starts at the door of the kitchen lab, drives to the sofa, looks at each of its
seat places and reports how many seat places are free, those of the chairs it saw
included.

What every run of this demonstration shares. Each way of carrying it out has a runner of
its own; run it from this folder::

    python rviz_demo.py [LEFT] [RIGHT]   # kinematic simulation, shown in RViz
    python mujoco_demo.py [LEFT] [RIGHT] # physical simulation in MuJoCo
    python real_demo.py                  # the real robot

The seat places named on the command line of a simulation have a person sitting on them.

.. warning:: The real robot does not start at the door by itself: put it there before
    running :mod:`real_demo`. It also needs the controller and the perception pipeline
    of the free seats demo running, see
    :mod:`coraplex_hsr_free_seats_demo.demo`.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    PlanExecutor,
)
from coraplex_hsr_free_seats_demo.demo import (
    FreeSeatsDemonstration,
    occupied_seats_from_command_line,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    HasSeatPlaces,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.shape_collection import ShapeCollection

__all__ = ["DoorToSofaDemonstration", "main", "occupied_seats_from_command_line"]

# %% the demonstration


@dataclass
class DoorToSofaDemonstration(FreeSeatsDemonstration):
    """
    The HSR drives from the door of the kitchen lab to the sofa and scans it for free
    seat places, see :class:`~coraplex_hsr_free_seats_demo.demo.FreeSeatsDemonstration`.
    """

    ros_node_name: str = "hsr_door_to_sofa_demo"

    robot_start: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(x=0.4)
    )
    """
    Where the robot starts in a simulated run: just inside the door in the south wall of
    the kitchen lab, facing into the room.
    """

    def build_simulation(self, world: World) -> MujocoSim:
        """
        As :meth:`FreeSeatsDemonstration.build_simulation`, with floors that do not
        collide, since the wheels, which sit slightly inside the floors, would catch on
        the edge between two of them.
        """
        for floor in world.get_semantic_annotations_by_type(Floor):
            floor.root.collision = ShapeCollection()
        return super().build_simulation(world)


def main(
    executor_type: type[PlanExecutor] = KinematicallySimulatedPlanExecutor,
    occupied_seats: list[SofaSeat] | None = None,
    headless: bool = False,
) -> int:
    """
    Run the demonstration and print how many of the seat places of the world are free.

    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :param occupied_seats: The seat places a person sits on in a simulated run.
    :param headless: Whether the physics of a physically simulated run steps without a
        viewer.
    :return: How many seat places of the world no one was seen sitting on.
    """
    world = DoorToSofaDemonstration(
        executor_type=executor_type,
        occupied_seats=occupied_seats or [],
        headless=headless,
    ).run()
    furniture = world.get_semantic_annotations_by_type(HasSeatPlaces)
    number_of_seats = sum(len(seats.seat_places) for seats in furniture)
    number_of_free_seats = sum(len(seats.free_seat_places) for seats in furniture)
    print(f"{number_of_free_seats} of {number_of_seats} seats are free")
    return number_of_free_seats
