"""
Geometric regression tests for the predefined kitchen environment.
"""

from __future__ import annotations

import numpy as np
import pytest
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point2,
    Point3,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Cooktop,
    CounterTop,
    Floor,
    KitchenIsland,
    Office,
    Sofa,
)
from semantic_digital_twin.world import World

# %% mesh bounds


def visual_world_bounds(world: World, body_name: str) -> np.ndarray:
    """
    Return a body's visual mesh bounds in the world root frame.
    """
    body = world.get_body_by_name(body_name)
    mesh = body.visual.combined_mesh.copy()
    mesh.apply_transform(world.compute_forward_kinematics_np(world.root, body))
    return mesh.bounds


# %% handle placement


def test_module_1_has_fixed_face_plate_above_door() -> None:
    """
    Module 1 should have a 14.3 cm face plate separated from its door.
    """
    world = KitchenEnvironment().create_world()
    face_plate_bounds = visual_world_bounds(world, "module_1_face_plate")
    door_bounds = visual_world_bounds(world, "module_1_door")

    assert np.isclose(np.ptp(face_plate_bounds[:, 2]), 0.143)
    assert np.isclose(face_plate_bounds[0, 2] - door_bounds[1, 2], 0.005)


@pytest.mark.parametrize("body_name", ["module_1_face_plate", "module_1_door"])
def test_module_1_front_piece_is_59_5_centimeters_wide(body_name: str) -> None:
    """
    Each Module 1 front piece should be narrower than the cabinet carcass.
    """
    world = KitchenEnvironment().create_world()
    body = world.get_body_by_name(body_name)

    assert np.isclose(body.visual.combined_mesh.extents[1], 0.595)


def test_module_1_handle_top_matches_scan_inset() -> None:
    """
    The Module 1 handle should be inset from the shortened door's top edge.
    """
    world = KitchenEnvironment().create_world()
    handle_bounds = visual_world_bounds(world, "module_1_handle")
    door_bounds = visual_world_bounds(world, "module_1_door")

    assert np.isclose(door_bounds[1, 2] - handle_bounds[1, 2], 0.044)


@pytest.mark.parametrize(
    ("handle_name", "door_name"),
    [
        ("dishwasher_handle", "dishwasher_door"),
        ("oven_handle", "oven_door"),
        ("module_1_handle", "module_1_door"),
        ("oven_cabinet_handle", "oven_cabinet_door"),
    ],
)
def test_horizontal_door_handle_fits_within_door_width(
    handle_name: str, door_name: str
) -> None:
    """
    A full-width horizontal handle should not extend beyond either door edge.
    """
    world = KitchenEnvironment().create_world()
    handle_bounds = visual_world_bounds(world, handle_name)
    door_bounds = visual_world_bounds(world, door_name)

    assert handle_bounds[0, 0] >= door_bounds[0, 0]
    assert handle_bounds[1, 0] <= door_bounds[1, 0]


def test_fridge_door_handle_is_near_free_edge() -> None:
    """
    The vertical fridge handle should be inset from the edge opposite its hinge.
    """
    world = KitchenEnvironment().create_world()
    handle_bounds = visual_world_bounds(world, "fridge_door_handle")
    door_bounds = visual_world_bounds(world, "fridge_door")

    handle_center_x = handle_bounds[:, 0].mean()
    assert np.isclose(door_bounds[1, 0] - handle_center_x, 0.041)


# %% the kitchen island


def test_the_sideboard_is_a_kitchen_island_built_from_its_corpus_under_its_worktop() -> (
    None
):
    """
    The sideboard is the kitchen island: its corpus is the cabinet built into it and its
    worktop is the island's counter top, as in the other modelled kitchens.
    """
    world = KitchenEnvironment().create_world()
    [island] = world.get_semantic_annotations_by_type(KitchenIsland)

    assert island.cabinets == [
        world.get_semantic_annotation_by_name("sideboard_cabinet")
    ]
    assert island.counter_top is world.get_semantic_annotation_by_name("sideboard")
    assert isinstance(island.counter_top, CounterTop)


def test_the_island_worktop_still_carries_its_cooktop() -> None:
    """
    Naming the worktop a counter top must not lose what stands on it.
    """
    world = KitchenEnvironment().create_world()
    worktop = world.get_semantic_annotation_by_name("sideboard")

    assert worktop.get_objects_of_type(Cooktop) == [
        world.get_semantic_annotation_by_name("sideboard_cooktop")
    ]


# %% driving between the rooms


@pytest.mark.parametrize("robot_radius", [0.25])
def test_a_path_on_the_apartment_floor_leads_from_the_kitchen_into_the_office_around_the_sofa(
    robot_radius: float,
) -> None:
    """
    The apartment floor spans every room, so a path laid out on it reaches the office
    from the kitchen without a robot of the given radius touching the sofa.
    """
    world = KitchenEnvironment().create_world()
    apartment_floor = world.get_semantic_annotation_by_name("apartment_floor")
    [office] = world.get_semantic_annotations_by_type(Office)
    [sofa] = world.get_semantic_annotations_by_type(Sofa)
    office_position = office.bodies[0].global_pose
    free_space = apartment_floor.planar_free_space(bloat_obstacles=robot_radius)

    path = free_space.path_from_to(
        Point2(0, 0, reference_frame=world.root),
        Point2(
            float(office_position.x),
            float(office_position.y),
            reference_frame=world.root,
        ),
    )

    world_P_path = np.array(
        [
            world.transform(
                Point3(
                    waypoint.x, waypoint.y, 0, reference_frame=waypoint.reference_frame
                ),
                world.root,
            ).to_np()[:2]
            for waypoint in path
        ]
    )
    sofa_box = sofa.as_bounding_box_collection_at_origin(
        HomogeneousTransformationMatrix(reference_frame=world.root)
    ).bounding_box()
    world_P_driven = np.concatenate(
        [
            np.linspace(start, end, num=100)
            for start, end in zip(world_P_path[:-1], world_P_path[1:])
        ]
    )
    touches_sofa = (
        (world_P_driven[:, 0] > sofa_box.min_x - robot_radius)
        & (world_P_driven[:, 0] < sofa_box.max_x + robot_radius)
        & (world_P_driven[:, 1] > sofa_box.min_y - robot_radius)
        & (world_P_driven[:, 1] < sofa_box.max_y + robot_radius)
    )
    assert isinstance(apartment_floor, Floor)
    assert world_P_path[-1] == pytest.approx(
        [float(office_position.x), float(office_position.y)]
    )
    assert not touches_sofa.any()


def test_the_apartment_floor_has_its_top_where_a_robot_stands() -> None:
    """
    A robot placed on the ground of the world stands on the apartment floor rather than
    sunk into it.
    """
    world = KitchenEnvironment().create_world()
    apartment_floor = world.get_semantic_annotation_by_name("apartment_floor")

    floor_box = apartment_floor.as_bounding_box_collection_at_origin(
        HomogeneousTransformationMatrix(reference_frame=world.root)
    ).bounding_box()

    # Mesh files store positions as 32-bit floats.
    assert floor_box.max_z == pytest.approx(float(world.root.global_pose.z), abs=1e-6)