import math

import pytest
import os

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.reasoning.queries import (
    semantic_annotations_on_surfaces,
    get_next_object_using_planar_distance,
    goal_surface_of_object,
    filter_annotations_by_color,
    annotation_class_by_label,
    sort_annotations_by_volume,
    find_objects_room,
    find_free_space_in_room,
    find_room,
)
from semantic_digital_twin.reasoning.predicates import OverFloorPlan
from semantic_digital_twin.reasoning.reasoner import CaseReasoner
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
from semantic_digital_twin.semantic_annotations.semantic_annotations import *
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    PrismaticConnection,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.geometry import Color


def test_load_environment_returns_world():
    """
    Tests that loading the environment returns a World object with the correct root
    name.
    """
    world = KitchenEnvironment().create_world()
    assert isinstance(world, World)
    assert world.root.name == PrefixedName("root")


def test_load_environment_names_are_stringifiable():
    """
    Every entity name must hold a plain string local part.

    The ``create_with_new_..._in_world`` factories take the name as a string and wrap it
    into a :class:`PrefixedName` themselves. Handing them a ``PrefixedName`` nests it
    inside another one, which breaks every consumer that stringifies the name.
    """
    world = KitchenEnvironment().create_world()
    for entity in world.kinematic_structure_entities:
        assert isinstance(entity.name.name, str)
        str(entity.name)


def test_world_reasoner_reason_returns_dicts():
    urdf_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "..",
        "..",
        "semantic_digital_twin",
        "resources",
        "urdf",
    )
    world = URDFParser.from_file(
        file_path=os.path.join(urdf_dir, "kitchen-small.urdf")
    ).create_world()
    reasoner = WorldReasoner(world)

    result = reasoner.reason()

    assert isinstance(result, dict)


def test_each_reasoner_applies_rules_of_its_own():
    """
    The rules keep the last case they classified in memory for as long as they are kept,
    so they belong to the reasoner that applied them rather than to the process.
    """
    world = World()

    assert CaseReasoner(world).rdr is not CaseReasoner(world).rdr


def test_semantic_annotations_on_surfaces(kitchen_environment_fixture):
    """
    Tests that giving Table annotations gives a list of the correct annotation on top.
    """
    table1 = kitchen_environment_fixture.get_semantic_annotation_by_name("fruit_table")
    table2 = kitchen_environment_fixture.get_semantic_annotation_by_name(
        "vegetable_table"
    )
    table3 = kitchen_environment_fixture.get_semantic_annotation_by_name("empty_table")
    apple = kitchen_environment_fixture.get_semantic_annotation_by_name("apple")
    carrot = kitchen_environment_fixture.get_semantic_annotation_by_name("carrot")
    orange = kitchen_environment_fixture.get_semantic_annotation_by_name("orange")
    lettuce = kitchen_environment_fixture.get_semantic_annotation_by_name("lettuce")
    banana1 = kitchen_environment_fixture.get_semantic_annotation_by_name("banana1")

    assert semantic_annotations_on_surfaces(
        [table1, table2, table3], kitchen_environment_fixture
    ) == [apple, orange, banana1, carrot, lettuce]
    assert semantic_annotations_on_surfaces([], kitchen_environment_fixture) == []


def test_get_next_object_using_planar_distance(kitchen_environment_fixture):
    """
    Tests the functionality of the `query_get_next_object_euclidean_x_y` function to
    verify that it accurately identifies the next objects based on their Euclidean
    proximity within a simulation world. The test involves setting up a virtual world,
    retrieving specific objects and annotations, and validating the results returned by
    the function against predetermined expectations.

    :raises AssertionError: If any of the function assertions fail during testing.
    """
    toya = kitchen_environment_fixture.get_body_by_name("base_link_body")
    table1 = kitchen_environment_fixture.get_semantic_annotation_by_name("fruit_table")
    table2 = kitchen_environment_fixture.get_semantic_annotation_by_name(
        "vegetable_table"
    )
    table3 = kitchen_environment_fixture.get_semantic_annotation_by_name("empty_table")
    apple = kitchen_environment_fixture.get_semantic_annotation_by_name("apple")
    carrot = kitchen_environment_fixture.get_semantic_annotation_by_name("carrot")
    orange = kitchen_environment_fixture.get_semantic_annotation_by_name("orange")
    lettuce = kitchen_environment_fixture.get_semantic_annotation_by_name("lettuce")
    banana1 = kitchen_environment_fixture.get_semantic_annotation_by_name("banana1")

    assert get_next_object_using_planar_distance(
        toya, table1, Vector3.Z()
    ).tolist() == [orange, banana1, apple]
    assert get_next_object_using_planar_distance(
        toya, table2, Vector3.Z()
    ).tolist() == [
        carrot,
        lettuce,
    ]
    assert get_next_object_using_planar_distance(
        apple.bodies[0], table1, Vector3.Z()
    ).tolist() == [apple, banana1, orange]
    assert get_next_object_using_planar_distance(
        apple.bodies[0], table1, Vector3.Y()
    ).tolist() == [apple, orange, banana1]

    assert (
        get_next_object_using_planar_distance(toya, table3, Vector3.Z()).tolist() == []
    )


def test_goal_surface_of_object(kitchen_environment_fixture):
    """
    Tests the `goal_surface_of_object` function for determining the surface of the most
    suitable semantic annotation object from a set of candidates based on similarity and
    other constraints.

    The function is evaluated under multiple scenarios to verify its logic in choosing
    the correct table, handling empty tables, and cases with no valid candidates.
    """
    table1 = kitchen_environment_fixture.get_semantic_annotation_by_name("fruit_table")
    table2 = kitchen_environment_fixture.get_semantic_annotation_by_name(
        "vegetable_table"
    )
    table3 = kitchen_environment_fixture.get_semantic_annotation_by_name("empty_table")
    table4 = kitchen_environment_fixture.get_semantic_annotation_by_name("empty_table2")

    banana = kitchen_environment_fixture.get_semantic_annotation_by_name("banana")
    apple = kitchen_environment_fixture.get_semantic_annotation_by_name("apple")
    carrot = kitchen_environment_fixture.get_semantic_annotation_by_name("carrot")
    orange = kitchen_environment_fixture.get_semantic_annotation_by_name("orange")
    lettuce = kitchen_environment_fixture.get_semantic_annotation_by_name("lettuce")

    # choosing the correct table
    assert goal_surface_of_object(banana, [table1, table2, table3]) == table1
    assert goal_surface_of_object(carrot, [table1, table2, table3]) == table2
    # choosing the empty table
    assert goal_surface_of_object(lettuce, [table1, table3]) == table3
    assert goal_surface_of_object(table1, [table1, table2, table3]) == table3
    # trying with a new threshold
    assert goal_surface_of_object(orange, [table2, table3], 2) == table2
    # returning None if there is no empty table or no tables
    assert goal_surface_of_object(apple, [table2]) == None
    assert goal_surface_of_object(orange, []) == None
    # trying with 2 empty tables
    assert goal_surface_of_object(apple, [table2, table3, table4]) == table3
    assert goal_surface_of_object(apple, [table2, table4, table3]) == table4


def test_filter_annotations_by_color(kitchen_environment_fixture):
    """
    Tests the filter_annotations_by_color function by verifying the retrieval of
    semantic annotations by their associated colors.

    The function validates that calling filter_annotations_by_color with different color
    parameters returns the expected list of annotations corresponding to that color
    within the given list of objects.
    """
    table1 = kitchen_environment_fixture.get_semantic_annotation_by_name("fruit_table")
    table2 = kitchen_environment_fixture.get_semantic_annotation_by_name(
        "vegetable_table"
    )
    apple = kitchen_environment_fixture.get_semantic_annotation_by_name("apple")
    orange = kitchen_environment_fixture.get_semantic_annotation_by_name("orange")
    carrot = kitchen_environment_fixture.get_semantic_annotation_by_name("carrot")

    assert filter_annotations_by_color(Color.RED(), [apple, orange]).tolist() == [apple]
    assert filter_annotations_by_color(Color.ORANGE(), [apple, orange]).tolist() == [
        orange
    ]
    assert (
        filter_annotations_by_color(Color.BLUE(), [apple, orange, carrot]).tolist()
        == []
    )
    assert filter_annotations_by_color(
        Color.ORANGE(),
        (
            semantic_annotations_on_surfaces(
                [table1, table2], kitchen_environment_fixture
            )
        ),
    ).tolist() == [orange, carrot]
    assert filter_annotations_by_color(Color.YELLOW(), []).tolist() == []


def test_annotation_class_by_label():
    """
    Tests the annotation_class_by_label function by verifying the retrieval of the
    semantic class.
    """
    assert annotation_class_by_label("candy_autodrop_box_cadillacs") == Candy
    assert annotation_class_by_label("milk_jumbo_pack_voll") == Milk
    assert annotation_class_by_label("bowl_collapsable_yellowgrey") == Bowl

    assert annotation_class_by_label("unknown_object") is None


def test_sort_annotations_by_volume(kitchen_environment_fixture):
    """
    Tests the sort_annotations_by_volume function by verifying the order of the returned
    annotations.
    """
    table1 = kitchen_environment_fixture.get_semantic_annotation_by_name("fruit_table")
    table2 = kitchen_environment_fixture.get_semantic_annotation_by_name(
        "vegetable_table"
    )

    apple = kitchen_environment_fixture.get_semantic_annotation_by_name("apple")
    carrot = kitchen_environment_fixture.get_semantic_annotation_by_name("carrot")
    lettuce = kitchen_environment_fixture.get_semantic_annotation_by_name("lettuce")

    assert sort_annotations_by_volume([]) == []
    assert sort_annotations_by_volume([apple, carrot]) == [apple, carrot]
    assert sort_annotations_by_volume([table1, lettuce, apple]) == [
        table1,
        lettuce,
        apple,
    ]
    assert sort_annotations_by_volume([table1, lettuce, apple], False) == [
        apple,
        lettuce,
        table1,
    ]
    assert sort_annotations_by_volume(
        semantic_annotations_on_surfaces([table2], kitchen_environment_fixture)
    ) == [lettuce, carrot]


def test_world_reasoner_leaves_urdf_doors_and_drawers_on_their_own_joints():
    """
    Doors and drawers loaded from a URDF hang straight from their cabinet on an active
    connection, e.g. ``iai_fridge_main`` -> (revolute) -> ``iai_fridge_door`` in
    ``coraplex``'s ``kitchen-small.urdf``.

    Recognising them must not change the
    kinematic structure: each one's joint is that connection.
    """
    coraplex_worlds_directory = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "..",
        "..",
        "coraplex",
        "resources",
        "worlds",
    )
    world = URDFParser.from_file(
        file_path=os.path.join(coraplex_worlds_directory, "kitchen-small.urdf")
    ).create_world()
    connections_before_reasoning = set(world.connections)
    reasoner = WorldReasoner(world)

    reasoner.infer_semantic_annotations()

    assert set(world.connections) == connections_before_reasoning
    doors = world.get_semantic_annotations_by_type(Door)
    assert doors
    for door in doors:
        assert isinstance(door.movable_joint, RevoluteConnection)
    drawers = world.get_semantic_annotations_by_type(Drawer)
    assert drawers
    for drawer in drawers:
        assert isinstance(drawer.movable_joint, PrismaticConnection)


# %% room of an annotation


@pytest.mark.parametrize(
    ("annotation_name", "room_type"),
    [("milk", Kitchen), ("sofa", LivingRoom), ("cooking_table", Bedroom)],
)
def test_find_objects_room_returns_the_room_the_annotation_stands_in(
    annotation_name: str, room_type: type[Room]
) -> None:
    """An annotation is in the room whose floor plan its root body stands
    over."""
    world = KitchenEnvironment().create_world()
    annotation = world.get_semantic_annotation_by_name(annotation_name)
    [room] = world.get_semantic_annotations_by_type(room_type)

    assert find_objects_room(annotation) is room


def test_find_objects_room_returns_none_outside_every_room() -> None:
    """An annotation that stands over no room's floor plan is in no room."""
    world = KitchenEnvironment().create_world()
    with world.modify_world():
        outside = Milk.create_with_new_body_in_world(
            name="milk_outside",
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=100, y=100, reference_frame=world.root
            ),
        )

    assert find_objects_room(outside) is None


# %% free space for the robot


def world_with_box_on_the_kitchen_point(box_width: float) -> World:
    """The kitchen environment with a box standing on the point
    :func:`find_room` gives for the kitchen.

    :param box_width: The width and depth of the box, in m.
    """
    world = KitchenEnvironment().create_world()
    kitchen_x, kitchen_y = find_room(Kitchen, world)
    with world.modify_world():
        Milk.create_with_new_body_in_world(
            name="obstacle",
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=kitchen_x, y=kitchen_y, z=0.5, reference_frame=world.root
            ),
            scale=Scale(box_width, box_width, 1.0),
        )
    return world


def test_find_free_space_in_room_returns_the_room_point_when_it_is_free() -> None:
    """A robot fits on the room's own point when nothing stands there."""
    world = KitchenEnvironment().create_world()

    free_pose = find_free_space_in_room(Kitchen, world)

    assert (float(free_pose.x), float(free_pose.y)) == pytest.approx(
        find_room(Kitchen, world)
    )


@pytest.mark.parametrize("robot_diameter", [0.5])
def test_find_free_space_in_room_moves_out_of_an_obstacle_on_the_room_point(
    robot_diameter: float,
) -> None:
    """With a box on the room's point, the robot stands just clear of it,
    inside the room."""
    box_width = 0.4
    world = world_with_box_on_the_kitchen_point(box_width)
    kitchen_x, kitchen_y = find_room(Kitchen, world)
    [kitchen] = world.get_semantic_annotations_by_type(Kitchen)

    free_pose = find_free_space_in_room(Kitchen, world, robot_diameter=robot_diameter)

    distance = math.hypot(
        float(free_pose.x) - kitchen_x, float(free_pose.y) - kitchen_y
    )
    assert distance >= (box_width + robot_diameter) / 2
    assert distance <= math.hypot(box_width, robot_diameter)
    assert OverFloorPlan(free_pose, kitchen.floor.root)()


def test_find_free_space_in_room_returns_none_when_no_place_in_reach_is_free() -> None:
    """When the search may not leave the blocked room point, no place is
    found."""
    world = world_with_box_on_the_kitchen_point(box_width=0.4)

    assert find_free_space_in_room(Kitchen, world, maximum_radius=0.0) is None
