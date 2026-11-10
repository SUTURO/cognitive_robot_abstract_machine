from __future__ import annotations

import pytest

from coraplex.robot_plans.actions.core.misc import DetectOccupantsAction
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Human,
    Sofa,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from ..plan_running import robot_extensions, run_plan

# %% hsr in front of the sofa


@pytest.fixture
def hsr_facing_the_sofa() -> tuple[World, HSRB]:
    """
    The kitchen with the HSR standing in front of the sofa, facing it.
    """
    world = KitchenEnvironment().create_world()
    [sofa] = world.get_semantic_annotations_by_type(Sofa)
    world_T_robot = (
        sofa.root.global_transform
        @ HomogeneousTransformationMatrix.from_xyz_rpy(x=-2.0)
    ).pose
    return world, RobotSpecification(
        semantic_annotation_type=HSRB,
        odometry_T_robot_start=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=world_T_robot.x, y=world_T_robot.y, yaw=world_T_robot.yaw
        ),
    ).spawn(world)


# %% detecting the occupants of the seats


def test_only_the_seat_a_person_is_seen_on_is_taken(hsr_facing_the_sofa):
    """
    The person on the left seat has no geometry, so the camera cannot see them: they
    stand for someone who left since the seats were last scanned.
    """
    world, robot = hsr_facing_the_sofa
    Human.get_annotation_specification(
        "sitter", BodySpecification.box("sitter", Scale(0.4, 0.4, 0.8))
    ).spawn(world, parent=SofaSeat.RIGHT.region_in(world))
    Human.get_annotation_specification(
        "earlier_sitter", BodySpecification("earlier_sitter")
    ).spawn(world, parent=SofaSeat.LEFT.region_in(world))
    plan = DetectOccupantsAction(regions=[seat.region_in(world) for seat in SofaSeat])

    run_plan(plan, robot_extensions(robot))

    assert [seat for seat in SofaSeat if seat.is_free_in(world)] == [SofaSeat.LEFT]
