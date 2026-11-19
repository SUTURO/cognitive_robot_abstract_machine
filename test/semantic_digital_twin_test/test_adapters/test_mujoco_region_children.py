"""
Tests for building worlds into MuJoCo whose regions carry bodies of their own.
"""

from __future__ import annotations

import mujoco
import pytest

from semantic_digital_twin.adapters.multi_sim import MujocoBuilder
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import MujocoEntityNotFoundError
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
    RegionSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Body

# %% a body sitting in a region


@pytest.fixture
def world_with_a_body_in_a_region() -> World:
    """
    A sofa on the floor with a seat region on it, and a person hanging off that region.
    """
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(Body(name=PrefixedName("floor")))
    BodySpecification.box(
        "sofa",
        Scale(1.0, 1.0, 0.4),
        child_specifications=[
            RegionSpecification.box(
                name="seat",
                scale=Scale(0.5, 0.5, 0.5),
                child_specifications=[
                    BodySpecification.box("person", Scale(0.2, 0.2, 0.4))
                ],
            )
        ],
    ).spawn(world)
    return world


def test_a_body_hanging_off_a_region_is_built_into_the_body_of_that_region(
    world_with_a_body_in_a_region: World, tmp_path
):
    builder = MujocoBuilder()
    builder.build_world(
        world=world_with_a_body_in_a_region, file_path=str(tmp_path / "scene.xml")
    )
    element_names = builder.element_names
    seat = world_with_a_body_in_a_region.get_kinematic_structure_entity_by_name("seat")
    person = world_with_a_body_in_a_region.get_kinematic_structure_entity_by_name(
        "person"
    )

    assert builder.spec.body(element_names.of(person)).parent.name == (
        element_names.of(seat)
    )


# %% looking up what was never built


def test_looking_up_an_entity_that_was_never_built_names_it(tmp_path):
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(Body(name=PrefixedName("floor")))
    builder = MujocoBuilder()
    builder.build_world(world=world, file_path=str(tmp_path / "scene.xml"))
    never_built = Body(name=PrefixedName("never_built"))

    with pytest.raises(MujocoEntityNotFoundError) as error:
        builder.body_spec_of(never_built)

    assert error.value.entity_type == mujoco.mjtObj.mjOBJ_BODY
    assert error.value.entity_name == builder.element_names.of(never_built)


# %% how a region is drawn


def test_the_shapes_of_a_region_are_drawn_translucent(
    world_with_a_body_in_a_region: World, tmp_path
):
    builder = MujocoBuilder()
    builder.build_world(
        world=world_with_a_body_in_a_region, file_path=str(tmp_path / "scene.xml")
    )
    seat = world_with_a_body_in_a_region.get_kinematic_structure_entity_by_name("seat")

    [geom] = builder.spec.body(builder.element_names.of(seat)).geoms
    assert geom.rgba[3] == pytest.approx(builder.region_opacity)


def test_the_shapes_of_a_body_keep_their_own_opacity(
    world_with_a_body_in_a_region: World, tmp_path
):
    builder = MujocoBuilder()
    builder.build_world(
        world=world_with_a_body_in_a_region, file_path=str(tmp_path / "scene.xml")
    )
    person = world_with_a_body_in_a_region.get_kinematic_structure_entity_by_name(
        "person"
    )

    [geom] = builder.spec.body(builder.element_names.of(person)).geoms
    assert geom.rgba[3] == pytest.approx(person.visual.shapes[0].color.A)
