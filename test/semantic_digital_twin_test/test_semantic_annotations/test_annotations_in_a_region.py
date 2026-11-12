from __future__ import annotations

from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Human,
    Milk,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
    RegionSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

# %% annotations in a region


def spawn_human(world: World, name: str, x: float) -> Human:
    """
    :return: A human whose root stands at ``x`` on the world's x axis.
    """
    return Human.get_annotation_specification(
        name,
        BodySpecification.box(
            name,
            Scale(0.2, 0.2, 0.2),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=x),
        ),
    ).spawn(world)


def test_the_annotations_in_a_region_are_those_whose_root_lies_inside(
    mini_world: World,
):
    region = RegionSpecification.box("region", Scale(1.0, 1.0, 1.0)).spawn(mini_world)
    inside = spawn_human(mini_world, "inside", x=0.2)
    spawn_human(mini_world, "outside", x=2.0)

    assert Human.get_instances_in_region(region) == [inside]


def test_annotations_of_other_types_are_left_out(mini_world: World):
    region = RegionSpecification.box("region", Scale(1.0, 1.0, 1.0)).spawn(mini_world)
    Milk.get_annotation_specification(
        "milk", BodySpecification.box("milk", Scale(0.1, 0.1, 0.2))
    ).spawn(mini_world)

    assert Human.get_instances_in_region(region) == []


def test_an_annotation_whose_body_carries_no_geometry_is_found_by_its_origin(
    mini_world: World,
):
    """
    A perception source reports where it saw a person, not how much space they fill, so
    the person it writes into the world has no geometry to measure an overlap with.
    """
    region = RegionSpecification.box("region", Scale(1.0, 1.0, 1.0)).spawn(mini_world)
    inside = Human.get_annotation_specification(
        "inside", BodySpecification("inside")
    ).spawn(mini_world, parent=region)

    assert Human.get_instances_in_region(region) == [inside]


def test_a_region_away_from_the_world_root_holds_what_stands_in_it(mini_world: World):
    region = RegionSpecification.box(
        "region",
        Scale(1.0, 1.0, 1.0),
        parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=2.0),
    ).spawn(mini_world)
    inside = spawn_human(mini_world, "inside", x=2.2)
    spawn_human(mini_world, "outside", x=0.2)

    assert Human.get_instances_in_region(region) == [inside]
