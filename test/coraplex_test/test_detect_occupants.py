from __future__ import annotations

import numpy as np
import pytest

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import kinematically_simulated_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.misc import DetectOccupantsAction
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Human,
    Sofa,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

# %% hsr in front of the sofa


@pytest.fixture
def hsr_facing_the_sofa() -> tuple[World, HSRB]:
    """
    The kitchen with the HSR standing in front of the sofa, facing it.
    """
    world = WorldSpecification(world_parser=None).to_domain_object()
    KitchenEnvironment().populate(world)
    [sofa] = world.get_semantic_annotations_by_type(Sofa)
    world_T_robot = (
        sofa.root.global_transform
        @ HomogeneousTransformationMatrix.from_xyz_rpy(x=2.0, yaw=np.pi)
    ).pose
    RobotSpecification(
        semantic_annotation_type=HSRB,
        odom_T_robot_start=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=world_T_robot.x, y=world_T_robot.y, yaw=world_T_robot.yaw
        ),
    ).spawn(world)
    [robot] = world.get_semantic_annotations_by_type(HSRB)
    return world, robot


# %% detecting the occupants of the seats


def test_only_the_seat_a_person_is_seen_on_is_taken(hsr_facing_the_sofa, rclpy_node):
    """
    The person on the left seat has no geometry, so the camera cannot see them: they
    stand for someone who left since the seats were last scanned.
    """
    world, robot = hsr_facing_the_sofa
    Human.get_annotation_specification(
        "sitter", BodySpecification.box("sitter", Scale(0.4, 0.4, 0.8))
    ).spawn(world, parent=SofaSeat.MIDDLE.region_in(world))
    Human.get_annotation_specification(
        "earlier_sitter", BodySpecification("earlier_sitter")
    ).spawn(world, parent=SofaSeat.LEFT.region_in(world))
    context = Context(
        world=world, robot=robot, ros_node=rclpy_node, evaluate_conditions=False
    )
    plan = sequential(
        [DetectOccupantsAction([seat.region_in(world) for seat in SofaSeat])],
        context=context,
    )

    with kinematically_simulated_robot:
        plan.perform()

    assert [seat for seat in SofaSeat if seat.is_free_in(world)] == [
        SofaSeat.LEFT,
        SofaSeat.RIGHT,
    ]
