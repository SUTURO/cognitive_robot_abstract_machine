"""
Tests for :class:`robokudo.io.simulated_camera.SimulatedCameraPublisher`: what it
publishes has to read back through RoboKudo's own camera interface.
"""

from __future__ import annotations

import cv2
import numpy
import pytest
from typing_extensions import TYPE_CHECKING
from scipy.spatial.transform import Rotation

from robokudo.descriptors.camera_configs.config_hsr_ros2 import HsrRos2CameraConfig
from robokudo.io.camera_interface import depth_convert_workaround
from robokudo.io.camera_model_adapters import RosCameraModelAdapter
from robokudo.io.simulated_camera import SimulatedCameraPublisher
from semantic_digital_twin.datastructures.camera_model import PinholeCameraModel
from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from semantic_digital_twin.spatial_computations.raytracer import RayTracer

from ..pytest_environment import runs_in_continuous_integration
from ..semantic_digital_twin_test.test_adapters.test_mujoco_camera_mount import (
    camera_facing_a_box,
    camera_simulation,
)

if TYPE_CHECKING:
    from rclpy.node import Node
    from robokudo.io.simulated_camera import CameraMessages
    from semantic_digital_twin.adapters.multi_sim import MujocoSim
    from ..semantic_digital_twin_test.test_adapters.test_mujoco_camera_mount import (
        CameraFacingABox,
    )

pytestmark = pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)

# %% one published frame


@pytest.fixture
def camera_resolution() -> CameraResolution:
    """
    The resolution at which the simulated camera captures frames.
    """
    return CameraResolution(width=160, height=120)


@pytest.fixture
def camera_configuration() -> HsrRos2CameraConfig:
    """
    The topic and frame names expected by the HSR camera interface.
    """
    return HsrRos2CameraConfig()


@pytest.fixture
def published_frame(
    camera_facing_a_box: CameraFacingABox,
    camera_configuration: HsrRos2CameraConfig,
    camera_simulation: MujocoSim,
    node: Node,
) -> CameraMessages:
    """
    The messages the publisher captures of a camera looking at a cube.
    """
    camera = camera_facing_a_box.camera
    publisher = SimulatedCameraPublisher(
        camera=camera,
        simulation=camera_simulation,
        node=node,
        topics=camera_configuration,
        frames=camera_configuration,
    )
    return publisher.capture()


def pinhole_of(camera_facing_a_box: CameraFacingABox) -> PinholeCameraModel:
    """
    :return: The calibration of the camera that looks at the cube.
    """
    camera = camera_facing_a_box.camera
    return PinholeCameraModel.from_field_of_view(
        resolution=camera.resolution, field_of_view=camera.field_of_view
    )


# %% reading the frame back


def test_depth_reads_back_in_millimeters(
    published_frame: CameraMessages, camera_facing_a_box: CameraFacingABox
) -> None:
    """
    RoboKudo reads the depth of the cube's front face where the camera sees it.
    """
    depth = depth_convert_workaround(published_frame.depth_image)
    pinhole = pinhole_of(camera_facing_a_box)
    box_center = camera_facing_a_box.box_center
    face_distance = box_center[0] - camera_facing_a_box.box_edge_length / 2
    camera_height = camera_facing_a_box.camera.root.global_transform.to_np()[2, 3]
    column = float(pinhole.principal_point.x) - pinhole.focal_length_x * (
        box_center[1] / face_distance
    )
    row = float(pinhole.principal_point.y) - pinhole.focal_length_y * (
        (box_center[2] - camera_height) / face_distance
    )

    assert depth[int(row), int(column)] == round(
        face_distance * RayTracer.millimeters_per_meter
    )


def test_empty_background_reads_as_missing_depth(
    published_frame: CameraMessages, camera_facing_a_box: CameraFacingABox
) -> None:
    """
    Nothing lies along the optical axis, so the camera measures no depth there.
    """
    depth = depth_convert_workaround(published_frame.depth_image)
    pinhole = pinhole_of(camera_facing_a_box)

    row, column = float(pinhole.principal_point.y), float(pinhole.principal_point.x)

    assert depth[int(row), int(column)] == 0


def test_color_reads_back_at_the_camera_resolution(
    published_frame: CameraMessages, camera_resolution: CameraResolution
) -> None:
    """
    The encoded color image preserves the camera resolution.
    """
    color = cv2.imdecode(
        numpy.frombuffer(published_frame.color_image.data, numpy.uint8),
        cv2.IMREAD_COLOR,
    )

    assert color.shape == (camera_resolution.height, camera_resolution.width, 3)


def test_calibration_reads_back_as_the_camera_model(
    published_frame: CameraMessages, camera_facing_a_box: CameraFacingABox
) -> None:
    """
    RoboKudo reads the original camera calibration from the message.
    """
    calibration = RosCameraModelAdapter.from_camera_info(published_frame.camera_info)
    pinhole = pinhole_of(camera_facing_a_box)

    assert calibration.resolution == pinhole.resolution
    numpy.testing.assert_allclose(
        calibration.intrinsic_matrix, pinhole.intrinsic_matrix
    )


def test_camera_pose_is_published_in_the_configured_frames(
    published_frame: CameraMessages,
    camera_facing_a_box: CameraFacingABox,
    camera_configuration: HsrRos2CameraConfig,
) -> None:
    """
    RoboKudo looks the camera pose up from the optical frame into the world frame its
    configuration names.
    """
    transform = published_frame.camera_transform
    translation = transform.transform.translation

    assert transform.header.frame_id == camera_configuration.tf_to
    assert transform.child_frame_id == camera_configuration.tf_from
    numpy.testing.assert_allclose(
        [translation.x, translation.y, translation.z],
        camera_facing_a_box.camera.root.global_transform.to_np()[:3, 3],
    )


def test_all_messages_of_a_frame_share_one_stamp_and_frame(
    published_frame: CameraMessages, camera_configuration: HsrRos2CameraConfig
) -> None:
    """
    RoboKudo synchronizes the images and the calibration by their stamps, and rejects
    images whose frame is not the configured optical frame.
    """
    headers = [
        published_frame.color_image.header,
        published_frame.depth_image.header,
        published_frame.camera_info.header,
    ]

    assert {header.frame_id for header in headers} == {camera_configuration.tf_from}
    assert {
        (header.stamp.sec, header.stamp.nanosec)
        for header in [*headers, published_frame.camera_transform.header]
    } == {(headers[0].stamp.sec, headers[0].stamp.nanosec)}


def test_camera_rotation_matches_the_optical_frame(
    published_frame: CameraMessages, camera_facing_a_box: CameraFacingABox
) -> None:
    """
    The published transform uses the optical frame's orientation.
    """
    rotation = published_frame.camera_transform.transform.rotation
    actual = Rotation.from_quat(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    ).as_matrix()
    expected = camera_facing_a_box.camera.root.global_transform.to_np()[:3, :3]
    numpy.testing.assert_allclose(actual, expected, atol=1e-6)
