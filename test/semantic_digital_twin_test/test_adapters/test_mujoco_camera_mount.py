"""
Tests for :meth:`semantic_digital_twin.adapters.multi_sim.MujocoCamera.mounted_on`.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from unittest.mock import Mock

import numpy
import pytest
from typing_extensions import Iterator

from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim
from semantic_digital_twin.datastructures.camera_model import (
    FieldOfViewCameraModel,
    PinholeCameraModel,
)
from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from semantic_digital_twin.datastructures.field_of_view import FieldOfView
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.robot_parts import Camera, RobotCamera
from semantic_digital_twin.robots.hsrb import HSRBHeadRGBDCamera
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

from ...pytest_environment import runs_in_continuous_integration

# %% a camera looking at a box


@dataclass
class CameraFacingABox:
    """
    A camera one meter above the world origin, looking along world x at a cube that
    floats above and to the left of its optical axis.
    """

    camera: Camera
    """
    The camera, rooted at its optical frame.
    """

    box_center: numpy.ndarray
    """
    The center of the cube in the world.
    """

    box_edge_length: float
    """
    The edge length of the cube.
    """


@pytest.fixture
def camera_facing_a_box(camera_resolution: CameraResolution) -> CameraFacingABox:
    """
    A camera facing a cube offset from its optical axis.
    """
    world = World.create_with_root_body()
    box_center = numpy.array([2.0, 0.5, 1.3])
    box_edge_length = 0.4
    BodySpecification.box(
        "box",
        Scale(box_edge_length, box_edge_length, box_edge_length),
        parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(*box_center),
    ).spawn(world)
    optical_frame = RobotCamera.create_optical_body(
        world.root,
        HomogeneousTransformationMatrix.from_xyz_rpy(
            z=1.0, roll=-numpy.pi / 2, yaw=-numpy.pi / 2
        ),
    )
    camera = Camera(
        name=PrefixedName("camera"),
        root=optical_frame,
        camera_model=FieldOfViewCameraModel(
            angular_extent=FieldOfView(
                horizontal_angle=0.99483, vertical_angle=0.75049
            ),
            image_resolution=camera_resolution,
        ),
    )
    return CameraFacingABox(
        camera=camera, box_center=box_center, box_edge_length=box_edge_length
    )


@pytest.fixture
def camera_simulation(
    camera_facing_a_box: CameraFacingABox, monkeypatch: pytest.MonkeyPatch
) -> Iterator[MujocoSim]:
    """
    A running simulation of the camera facing the cube.
    """
    monkeypatch.setenv("MUJOCO_GL", "egl")
    camera = camera_facing_a_box.camera
    camera.root.add_simulator_property(MujocoCamera.mounted_on(camera))
    simulation = MujocoSim(world=camera.root._world, headless=True)
    simulation.start_stepped_simulation()
    simulation.step_simulation(timedelta(milliseconds=10))
    yield simulation
    simulation.stop_simulation()


# %% what the mounted camera renders


@pytest.mark.parametrize("camera_resolution", [CameraResolution(width=160, height=120)])
def test_mount_hsr_head_camera(camera_facing_a_box: CameraFacingABox) -> None:
    """
    The HSR head camera is selected and attached to its optical frame.
    """
    world = Mock(spec=World)
    camera = camera_facing_a_box.camera
    world.get_semantic_annotations_by_type.return_value = [camera]

    mounted_camera = MujocoCamera.mount_hsr_head(world)

    world.get_semantic_annotations_by_type.assert_called_once_with(HSRBHeadRGBDCamera)
    assert mounted_camera is camera
    assert camera.root.simulator_additional_properties == [
        MujocoCamera.mounted_on(camera)
    ]


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
@pytest.mark.parametrize(
    "camera_resolution",
    [
        CameraResolution(width=160, height=120),
        CameraResolution(width=1024, height=768),
    ],
    ids=["within the default offscreen framebuffer", "beyond it"],
)
def test_mounted_camera_renders_what_the_pinhole_camera_sees(
    camera_facing_a_box: CameraFacingABox, camera_simulation: MujocoSim
) -> None:
    """
    The face of the cube that points at the camera covers the pixels the camera's
    pinhole model projects it to, so the image is neither turned nor scaled.
    """
    camera = camera_facing_a_box.camera
    resolution = camera.resolution
    depth = camera_simulation.simulator.capture_depth(
        camera_name=camera.root.name.name,
        height=resolution.height,
        width=resolution.width,
    ).result
    half_edge = camera_facing_a_box.box_edge_length / 2
    box_left, box_right = (
        camera_facing_a_box.box_center[1] + half_edge,
        camera_facing_a_box.box_center[1] - half_edge,
    )
    box_top, box_bottom = (
        camera_facing_a_box.box_center[2] + half_edge,
        camera_facing_a_box.box_center[2] - half_edge,
    )
    face_distance = camera_facing_a_box.box_center[0] - half_edge
    camera_height = camera.root.global_transform.to_np()[2, 3]
    pinhole = PinholeCameraModel.from_field_of_view(
        resolution=resolution, field_of_view=camera.field_of_view
    )
    expected_columns = [
        float(pinhole.principal_point.x)
        - pinhole.focal_length_x * world_y / face_distance
        for world_y in (box_left, box_right)
    ]
    expected_rows = [
        float(pinhole.principal_point.y)
        - pinhole.focal_length_y * (world_z - camera_height) / face_distance
        for world_z in (box_top, box_bottom)
    ]

    rows, columns = numpy.nonzero(numpy.isclose(depth, face_distance, atol=1e-3))
    numpy.testing.assert_allclose(
        [columns.min(), columns.max() + 1], expected_columns, atol=1.0
    )
    numpy.testing.assert_allclose([rows.min(), rows.max() + 1], expected_rows, atol=1.0)
