"""
Tests for planning a drive that leaves the floor it starts on.
"""

from __future__ import annotations

import numpy as np
import pytest

from coraplex.robot_plans.actions.core.navigation import (
    PathPlanningNavigateAcrossFloorsAction,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose2D
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from ...plan_running import expand, robot_extensions

pytestmark = pytest.mark.skipif(
    not HSRB.is_description_installed(),
    reason="The description of the HSR is not installed",
)

# %% the drive


@pytest.fixture
def kitchen_lab() -> World:
    """
    The kitchen lab with the HSR in its kitchen.
    """
    world = WorldSpecification(world_source=KitchenEnvironment()).create_world()
    RobotSpecification(
        semantic_annotation_type=HSRB,
        odometry_T_robot_start=HomogeneousTransformationMatrix.from_xyz_rpy(x=0.4),
    ).spawn(world)
    return world


def planned_positions(world: World) -> np.ndarray:
    """
    :param world: The kitchen lab with the HSR.
    :return: The x and y of the start and of the poses a drive from the kitchen into the
        living room goes through.
    """
    [robot] = world.get_semantic_annotations_by_type(HSRB)
    action = PathPlanningNavigateAcrossFloorsAction(
        planar_goal_pose=Pose2D.from_xy_yaw(x=3.7, y=3.2, reference_frame=world.root),
        floors=world.get_semantic_annotations_by_type(Floor),
        clearance=0.05,
    )
    expand(action, robot_extensions(robot))
    start = robot.root.global_pose.position.to_np()[:2]
    positions = [pose.position.to_np()[:2] for pose in action._path()]
    return np.array([start, *positions], dtype=float)


# %% planning


def test_path_leads_from_one_floor_to_another(kitchen_lab):
    positions = planned_positions(kitchen_lab)

    assert positions[-1] == pytest.approx([3.7, 3.2], abs=0.01)


def test_path_is_planned_without_collision_shapes_of_the_floors(kitchen_lab):
    """
    Floors that do not collide can still be driven on.
    """
    for floor in kitchen_lab.get_semantic_annotations_by_type(Floor):
        floor.root.collision = ShapeCollection()

    positions = planned_positions(kitchen_lab)

    assert positions[-1] == pytest.approx([3.7, 3.2], abs=0.01)


def test_path_leads_around_an_obstacle_on_the_straight_line(kitchen_lab):
    """
    The base's radius and the clearance are kept from a box standing on the straight
    line to the goal.
    """
    obstacle_position = np.array([2.0, 1.55])
    side_length = 0.3
    BodySpecification.box(
        "obstacle",
        Scale(side_length, side_length, 1.0),
        Color(hue=0.0, saturation=1.0, value=1.0),
    ).spawn(
        kitchen_lab,
        parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
            *obstacle_position, 0.5
        ),
    )
    [robot] = kitchen_lab.get_semantic_annotations_by_type(HSRB)

    positions = planned_positions(kitchen_lab)

    samples = np.concatenate(
        [np.linspace(start, end, 200) for start, end in zip(positions, positions[1:])]
    )
    outside = np.maximum(np.abs(samples - obstacle_position) - side_length / 2, 0)
    assert (
        np.linalg.norm(outside, axis=1).min()
        >= robot.mobile_base.base_radius + 0.05 - 1e-3
    )
