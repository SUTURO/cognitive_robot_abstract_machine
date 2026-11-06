from __future__ import annotations

import pytest

from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Human,
    Sofa,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Region

# %% seat places of the sofa


@pytest.fixture
def kitchen_world() -> World:
    """
    A fresh kitchen the test may seat people in.
    """
    return KitchenEnvironment().get_world()


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


def test_seats_cover_the_top_of_the_sofa_side_by_side(kitchen_world: World, sofa: Sofa):
    sofa_box = sofa.root.collision.as_bounding_box_collection_in_frame(
        sofa.root
    ).bounding_box()
    seat_boxes = [
        seat.region_in(kitchen_world)
        .area.as_bounding_box_collection_in_frame(sofa.root)
        .bounding_box()
        for seat in SofaSeat
    ]

    for seat_box in seat_boxes:
        assert seat_box.min_x == pytest.approx(sofa_box.min_x)
        assert seat_box.max_x == pytest.approx(sofa_box.max_x)
        assert seat_box.min_z == pytest.approx(sofa_box.max_z)
    assert seat_boxes[0].max_y == pytest.approx(sofa_box.max_y)
    assert seat_boxes[-1].min_y == pytest.approx(sofa_box.min_y)
    for left_seat_box, right_seat_box in zip(seat_boxes, seat_boxes[1:]):
        assert right_seat_box.max_y == pytest.approx(left_seat_box.min_y)


def test_the_sofa_faces_the_coffee_table(kitchen_world: World, sofa: Sofa):
    coffee_table = kitchen_world.get_kinematic_structure_entity_by_name("coffee_table")
    sofa_T_coffee_table = kitchen_world.compute_forward_kinematics(
        sofa.root, coffee_table
    ).pose

    assert sofa_T_coffee_table.x > 0


# %% free seats


def test_every_seat_of_an_empty_sofa_is_free(kitchen_world: World):
    assert all(seat.is_free_in(kitchen_world) for seat in SofaSeat)


def test_a_seat_with_a_person_on_it_is_taken(kitchen_world: World):
    Human.get_annotation_specification("sitter", BodySpecification("sitter")).spawn(
        kitchen_world, parent=SofaSeat.RIGHT.region_in(kitchen_world)
    )

    assert [seat for seat in SofaSeat if seat.is_free_in(kitchen_world)] == [
        SofaSeat.LEFT,
        SofaSeat.MIDDLE,
    ]


# %% populating a map


def test_populating_a_map_spawns_the_sofa_seats():
    world = WorldSpecification(world_parser=None).to_domain_object()

    KitchenEnvironment().populate(world)

    assert all(
        world.is_kinematic_structure_entity_in_world_by_name(seat.value)
        for seat in SofaSeat
    )
