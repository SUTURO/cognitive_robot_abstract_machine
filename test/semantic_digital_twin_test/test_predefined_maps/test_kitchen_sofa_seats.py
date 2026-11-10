from __future__ import annotations

import json

import pytest

from semantic_digital_twin.adapters.ros.messages import WorldModelSnapshot
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)

from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
    KitchenSofa,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Human,
    Sofa,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Region

# %% seat places of the sofa


@pytest.fixture
def kitchen_world() -> World:
    """
    A fresh kitchen the test may seat people in.
    """
    return KitchenEnvironment().create_world()


@pytest.fixture
def sofa(kitchen_world: World) -> Sofa:
    """
    The sofa of the kitchen.
    """
    [sofa] = kitchen_world.get_semantic_annotations_by_type(Sofa)
    return sofa


@pytest.mark.parametrize("seat", list(SofaSeat))
def test_every_seat_is_a_region_hanging_off_the_sofa(
    kitchen_world: World, sofa: Sofa, seat: SofaSeat
):
    region = seat.region_in(kitchen_world)

    assert isinstance(region, Region)
    assert region.parent_kinematic_structure_entity == sofa.root


def test_the_seat_places_of_the_sofa_are_its_seat_regions(
    kitchen_world: World, sofa: Sofa
):
    """
    The sofa lists the region of every :class:`SofaSeat` as one of its seat places.
    """
    assert sofa.seat_places == [seat.region_in(kitchen_world) for seat in SofaSeat]


def replayed(world: World) -> World:
    """
    :return: A new world built from the modifications of ``world``, as a process that
        fetches ``world`` over ROS builds its copy.
    """
    snapshot = WorldModelSnapshot(
        modifications=list(world.get_world_model_manager().model_modification_blocks),
        ids=list(world.state.keys()),
        states=list(world.state.positions),
    )
    copy = World()
    WorldModelSnapshot.apply_to_json_snapshot_to_world(
        copy,
        json.loads(json.dumps(snapshot.to_json())),
        **WorldEntityWithIDKwargsTracker.from_world(copy).create_kwargs(),
    )
    return copy


def test_the_seat_places_of_the_sofa_reach_a_copy_of_the_world(kitchen_world: World):
    """
    A process that fetches the kitchen finds the sofa with its seat places.
    """
    copy = replayed(kitchen_world)

    [sofa_in_copy] = copy.get_semantic_annotations_by_type(Sofa)
    assert sofa_in_copy.seat_places == [seat.region_in(copy) for seat in SofaSeat]


def test_the_free_seat_places_of_the_sofa_leave_out_a_taken_one(
    kitchen_world: World,
    sofa: Sofa,
    person: SemanticAnnotationWithRootSpecification[Human],
):
    """
    A seat place a person sits on is not among the free seat places of the sofa.
    """
    SofaSeat.RIGHT.place_person_in(kitchen_world, person)

    assert sofa.free_seat_places == [SofaSeat.LEFT.region_in(kitchen_world)]


def test_each_seat_rises_from_the_top_of_its_cushion(kitchen_world: World, sofa: Sofa):
    sofa_shape = KitchenSofa()
    cushion_top = max(point.height for point in sofa_shape.seat.points)

    for seat in SofaSeat:
        seat_box = (
            seat.region_in(kitchen_world)
            .area.as_bounding_box_collection_in_frame(sofa.root)
            .bounding_box()
        )
        assert seat_box.min_z == pytest.approx(cushion_top - sofa_shape.origin_height)
        assert seat_box.max_z == pytest.approx(
            cushion_top - sofa_shape.origin_height + sofa_shape.seat_place_height
        )


def test_the_seats_lie_side_by_side_between_the_armrests(
    kitchen_world: World, sofa: Sofa
):
    """
    The sofa faces along its -x axis, so the left of someone sitting on it is its -y.
    """
    sofa_shape = KitchenSofa()
    inner_width = sofa_shape.width / 2 - sofa_shape.arm_width
    left_box, right_box = (
        seat.region_in(kitchen_world)
        .area.as_bounding_box_collection_in_frame(sofa.root)
        .bounding_box()
        for seat in (SofaSeat.LEFT, SofaSeat.RIGHT)
    )

    assert left_box.min_y == pytest.approx(-inner_width)
    assert left_box.max_y < right_box.min_y
    assert right_box.max_y == pytest.approx(inner_width)


def test_the_sofa_faces_the_coffee_table(kitchen_world: World, sofa: Sofa):
    coffee_table = kitchen_world.get_kinematic_structure_entity_by_name("coffee_table")
    sofa_T_coffee_table = kitchen_world.compute_forward_kinematics(
        sofa.root, coffee_table
    ).pose

    assert sofa_T_coffee_table.x < 0


# %% free seats


def test_every_seat_of_an_empty_sofa_is_free(kitchen_world: World):
    assert SofaSeat.free_in(kitchen_world) == list(SofaSeat)


def test_a_seat_with_a_person_on_it_is_taken(kitchen_world: World):
    Human.get_annotation_specification("sitter", BodySpecification("sitter")).spawn(
        kitchen_world, parent=SofaSeat.RIGHT.region_in(kitchen_world)
    )

    assert SofaSeat.free_in(kitchen_world) == [SofaSeat.LEFT]


# %% seating people


@pytest.fixture
def person() -> SemanticAnnotationWithRootSpecification[Human]:
    """
    A person shaped like a box, small enough to sit on one seat place.
    """
    return Human.get_annotation_specification(
        "person", BodySpecification.box("person", Scale(0.4, 0.4, 0.8))
    )


def test_a_person_placed_on_a_seat_takes_only_that_seat(
    kitchen_world: World, person: SemanticAnnotationWithRootSpecification[Human]
):
    SofaSeat.LEFT.place_person_in(kitchen_world, person)

    assert SofaSeat.free_in(kitchen_world) == [SofaSeat.RIGHT]


def test_a_person_placed_on_a_seat_is_fixed_to_the_sofa(
    kitchen_world: World,
    sofa: Sofa,
    person: SemanticAnnotationWithRootSpecification[Human],
):
    seated = SofaSeat.RIGHT.place_person_in(kitchen_world, person)

    assert seated.root.parent_kinematic_structure_entity == sofa.root


def test_every_seat_of_an_empty_sofa_is_listed_free_from_left_to_right(
    kitchen_world: World,
):
    assert SofaSeat.free_in(kitchen_world) == [SofaSeat.LEFT, SofaSeat.RIGHT]


def test_a_taken_seat_is_not_listed_free(
    kitchen_world: World, person: SemanticAnnotationWithRootSpecification[Human]
):
    SofaSeat.RIGHT.place_person_in(kitchen_world, person)

    assert SofaSeat.free_in(kitchen_world) == [SofaSeat.LEFT]
