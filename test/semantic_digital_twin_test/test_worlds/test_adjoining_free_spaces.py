"""
Paths through the free spaces of regions that border each other.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Point2
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import PlanarBoundingBox
from semantic_digital_twin.world_description.graph_of_convex_sets.adjoining import (
    AdjoiningFreeSpaces,
    PlaceInFreeSpace,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    PlanarGraphOfBoundingBoxes,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)
from semantic_digital_twin.world_description.world_entity import Body

# %% two rooms joined by a doorway


@dataclass(kw_only=True)
class TwoRooms:
    """
    Two rooms side by side, whose free spaces touch only in a doorway in the wall at
    x = 2 between them.
    """

    world: World
    """
    The world the rooms lie in.
    """

    left_room: PlanarGraphOfBoundingBoxes
    """
    The free space of the room from x = 0 to x = 2.
    """

    right_room: PlanarGraphOfBoundingBoxes
    """
    The free space of the room from x = 2 to x = 4.
    """

    doorway_y: tuple[float, float] = (1.0, 1.5)
    """
    Where along y the doorway spans, in m.
    """


def _free_space(
    *, world: World, area: PlanarBoundingBox, nodes: list[PlanarBoundingBox]
) -> PlanarGraphOfBoundingBoxes:
    """
    :return: The free space of the nodes, searched over the area.
    """
    free_space = PlanarGraphOfBoundingBoxes(
        world, BoundingBoxCollection([area], world.root)
    )
    for node in nodes:
        free_space.add_node(node)
    free_space.calculate_connectivity()
    return free_space


@pytest.fixture
def two_rooms() -> TwoRooms:
    """
    :return: Two rooms side by side, joined by a doorway in the wall at x = 2 between
        y = 1 and y = 1.5.
    """
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(Body(name=PrefixedName("map")))
    origin = HomogeneousTransformationMatrix(reference_frame=world.root)
    return TwoRooms(
        world=world,
        left_room=_free_space(
            world=world,
            area=PlanarBoundingBox(0, 0, 2, 2, origin),
            nodes=[
                PlanarBoundingBox(0, 0, 1.9, 2, origin),
                PlanarBoundingBox(1.9, 1, 2, 1.5, origin),
            ],
        ),
        right_room=_free_space(
            world=world,
            area=PlanarBoundingBox(2, 0, 4, 2, origin),
            nodes=[
                PlanarBoundingBox(2, 1, 2.1, 1.5, origin),
                PlanarBoundingBox(2.1, 0, 4, 2, origin),
            ],
        ),
    )


def test_a_path_into_the_next_room_leads_through_the_doorway(
    two_rooms: TwoRooms,
) -> None:
    """
    A path between the rooms holds one waypoint on either side of the wall, in the
    middle of the doorway and as far from the wall as two nodes may be apart to touch.
    """
    joined = AdjoiningFreeSpaces(
        world=two_rooms.world,
        free_spaces=[two_rooms.left_room, two_rooms.right_room],
    )
    start = Point2(0.5, 0.25, reference_frame=two_rooms.world.root)
    goal = Point2(3.5, 0.25, reference_frame=two_rooms.world.root)

    path = joined.path_from_to(
        start=PlaceInFreeSpace(free_space=two_rooms.left_room, position=start),
        goal=PlaceInFreeSpace(free_space=two_rooms.right_room, position=goal),
    )

    doorway_middle = sum(two_rooms.doorway_y) / 2
    half_gap = joined.gap_tolerance / 2
    border_coordinates = [
        coordinate
        for waypoint in path
        if abs(float(waypoint.x) - 2) < joined.gap_tolerance
        for coordinate in (float(waypoint.x), float(waypoint.y))
    ]
    assert (float(path[0].x), float(path[0].y)) == (float(start.x), float(start.y))
    assert (float(path[-1].x), float(path[-1].y)) == (float(goal.x), float(goal.y))
    assert border_coordinates == pytest.approx(
        [2 - half_gap, doorway_middle, 2 + half_gap, doorway_middle]
    )


def test_there_is_no_path_between_rooms_whose_free_spaces_do_not_touch(
    two_rooms: TwoRooms,
) -> None:
    """
    Rooms whose free spaces leave a gap between them have no path from one to the other.
    """
    origin = HomogeneousTransformationMatrix(reference_frame=two_rooms.world.root)
    walled_off_room = _free_space(
        world=two_rooms.world,
        area=PlanarBoundingBox(2, 0, 4, 2, origin),
        nodes=[PlanarBoundingBox(2.1, 0, 4, 2, origin)],
    )
    joined = AdjoiningFreeSpaces(
        world=two_rooms.world, free_spaces=[two_rooms.left_room, walled_off_room]
    )

    path = joined.path_from_to(
        start=PlaceInFreeSpace(
            free_space=two_rooms.left_room,
            position=Point2(0.5, 0.25, reference_frame=two_rooms.world.root),
        ),
        goal=PlaceInFreeSpace(
            free_space=walled_off_room,
            position=Point2(3.5, 0.25, reference_frame=two_rooms.world.root),
        ),
    )

    assert path is None


def test_a_path_within_one_room_is_that_rooms_own_path(two_rooms: TwoRooms) -> None:
    """
    A start and goal in the same room are joined by the path that room's free space
    plans on its own.
    """
    joined = AdjoiningFreeSpaces(
        world=two_rooms.world,
        free_spaces=[two_rooms.left_room, two_rooms.right_room],
    )
    start = Point2(0.5, 0.25, reference_frame=two_rooms.world.root)
    goal = Point2(1.95, 1.25, reference_frame=two_rooms.world.root)

    path = joined.path_from_to(
        start=PlaceInFreeSpace(free_space=two_rooms.left_room, position=start),
        goal=PlaceInFreeSpace(free_space=two_rooms.left_room, position=goal),
    )

    assert path == two_rooms.left_room.path_from_to(start, goal)


# %% the rooms of the kitchen environment


def test_a_path_from_the_kitchen_into_the_office_leads_through_the_living_room() -> (
    None
):
    """
    The office borders the kitchen only through the living room, so a path between them
    passes over the floors of the kitchen, the living room and the office, in turn.
    """
    world = KitchenEnvironment().create_world()
    kitchen, living_room, office = (
        world.get_semantic_annotation_by_name(name).floor
        for name in ("kitchen", "living_room", "office")
    )
    free_spaces = [
        room_floor.planar_free_space(bloat_obstacles=0.25)
        for room_floor in (kitchen, living_room, office)
    ]
    joined = AdjoiningFreeSpaces(world=world, free_spaces=free_spaces)
    office_position = office.root.global_pose

    path = joined.path_from_to(
        start=PlaceInFreeSpace(
            free_space=free_spaces[0],
            position=Point2(0, 0, reference_frame=world.root),
        ),
        goal=PlaceInFreeSpace(
            free_space=free_spaces[2],
            position=Point2(
                float(office_position.x),
                float(office_position.y),
                reference_frame=world.root,
            ),
        ),
    )

    floors_passed = [path[0].reference_frame]
    for waypoint in path[1:]:
        if waypoint.reference_frame is not floors_passed[-1]:
            floors_passed.append(waypoint.reference_frame)
    assert floors_passed == [
        kitchen.supporting_surface,
        living_room.supporting_surface,
        office.supporting_surface,
    ]
