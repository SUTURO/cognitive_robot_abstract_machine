from __future__ import annotations

from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Human,
    Milk,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
    RegionSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

# %% annotations within a region


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


def test_region_holds_the_annotations_whose_root_lies_inside(mini_world: World):
    region = RegionSpecification.box("region", Scale(1.0, 1.0, 1.0)).spawn(mini_world)
    inside = spawn_human(mini_world, "inside", x=0.2)
    spawn_human(mini_world, "outside", x=2.0)

    assert region.semantic_annotations_within(Human) == [inside]


def test_region_ignores_annotations_of_other_types(mini_world: World):
    region = RegionSpecification.box("region", Scale(1.0, 1.0, 1.0)).spawn(mini_world)
    Milk.get_annotation_specification(
        "milk", BodySpecification.box("milk", Scale(0.1, 0.1, 0.2))
    ).spawn(mini_world)

    assert region.semantic_annotations_within(Human) == []


def test_region_follows_its_parent_frame(mini_world: World):
    region = RegionSpecification.box(
        "region",
        Scale(1.0, 1.0, 1.0),
        parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=2.0),
    ).spawn(mini_world)
    inside = spawn_human(mini_world, "inside", x=2.2)
    spawn_human(mini_world, "outside", x=0.2)

    assert region.semantic_annotations_within(Human) == [inside]
