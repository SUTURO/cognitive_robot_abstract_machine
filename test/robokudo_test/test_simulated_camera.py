"""
Tests for :mod:`robokudo.io.simulated_camera`: what
:class:`~robokudo.io.simulated_camera.SimulatedCameraPublisher` publishes has to read
back through RoboKudo's own camera interface, and
:class:`~robokudo.io.simulated_camera.RoboKudoSimulatedPerception` publishes its camera.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy
import pytest
from typing_extensions import TYPE_CHECKING
from scipy.spatial.transform import Rotation

from robokudo.analysis_engine_process import (
    AnalysisEngineModule,
    AnalysisEngineProcess,
)
from robokudo.descriptors.camera_configs.config_hsr_ros2 import HsrRos2CameraConfig
from robokudo.io.camera_interface import depth_convert_workaround
from robokudo.io.camera_model_adapters import RosCameraModelAdapter
from robokudo.io.exceptions import CameraAnnotationAmbiguous, CameraTypeNotInWorld
from robokudo.io.simulated_camera import (
    RoboKudoSimulatedPerception,
    SimulatedCameraPublisher,
)
from semantic_digital_twin.adapters.multi_sim import MujocoCamera
from semantic_digital_twin.datastructures.camera_model import PinholeCameraModel
from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.robot_parts import Camera
from semantic_digital_twin.spatial_computations.raytracer import RayTracer

from ..pytest_environment import runs_in_continuous_integration
from ..semantic_digital_twin_test.test_adapters.test_mujoco_camera_mount import (
    camera_facing_a_box as camera_facing_a_box,
    camera_simulation as camera_simulation,
)

if TYPE_CHECKING:
    from rclpy.node import Node
    from robokudo.io.simulated_camera import CameraMessages
    from semantic_digital_twin.adapters.multi_sim import MujocoSim
    from semantic_digital_twin.world import World
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


# %% perceiving through the camera of a type


@dataclass(kw_only=True)
class RecordingAnalysisEngine(AnalysisEngineProcess):
    """
    An analysis engine that records being started and stopped instead of running.
    """

    started: bool = False
    """
    Whether it was started.
    """

    stopped: bool = False
    """
    Whether it was stopped.
    """

    def start(self) -> None:
        """
        Differs from :meth:`AnalysisEngineProcess.start` in only recording the start.
        """
        self.started = True

    def stop(self) -> None:
        """
        Differs from :meth:`AnalysisEngineProcess.stop` in only recording the stop.
        """
        self.stopped = True


@pytest.fixture
def world_with_a_camera(camera_facing_a_box: CameraFacingABox) -> World:
    """
    The world of the camera facing the cube, which holds the camera as an annotation.
    """
    camera = camera_facing_a_box.camera
    world = camera.root._world
    with world.modify_world():
        world.add_semantic_annotation(camera)
    return world


@pytest.fixture
def started_simulated_perception(
    world_with_a_camera: World, camera_simulation: MujocoSim, node: Node
) -> RoboKudoSimulatedPerception:
    """
    A simulated perception through the camera facing the cube, started in the running
    simulation of its world.
    """
    simulated_perception = RoboKudoSimulatedPerception(
        camera_type=Camera,
        analysis_engine=RecordingAnalysisEngine(
            analysis_engine=AnalysisEngineModule.HUMAN_DETECTION
        ),
    )
    simulated_perception.start_perceiving(simulation=camera_simulation, ros_node=node)
    return simulated_perception


def test_by_default_the_camera_is_published_like_on_the_real_hsr() -> None:
    """
    Without other topics, the camera reaches the topics and frames the analysis engines
    of RoboKudo read on the real HSR.
    """
    simulated_perception = RoboKudoSimulatedPerception()

    assert simulated_perception.camera_topics == HsrRos2CameraConfig()
    assert simulated_perception.camera_frames == HsrRos2CameraConfig()


def test_by_default_humans_are_detected() -> None:
    """
    Without another analysis engine, the one that detects humans runs.
    """
    simulated_perception = RoboKudoSimulatedPerception()

    assert (
        simulated_perception.analysis_engine.analysis_engine
        == AnalysisEngineModule.HUMAN_DETECTION
    )


def test_mounting_attaches_a_mujoco_camera_to_the_camera_of_the_type(
    world_with_a_camera: World, camera_facing_a_box: CameraFacingABox
) -> None:
    """
    The camera of the type is the one the simulation renders.
    """
    camera = camera_facing_a_box.camera

    RoboKudoSimulatedPerception(camera_type=Camera).mount_cameras(
        world=world_with_a_camera
    )

    [mujoco_camera] = camera.root.simulator_additional_properties
    assert isinstance(mujoco_camera, MujocoCamera)


def test_a_world_without_a_camera_of_the_type_cannot_be_perceived(
    world_with_a_camera: World,
) -> None:
    """
    The world of the cube holds no head camera of an HSR to perceive through.
    """
    with pytest.raises(CameraTypeNotInWorld):
        RoboKudoSimulatedPerception().mount_cameras(world=world_with_a_camera)


def test_two_cameras_of_the_type_leave_open_which_one_is_perceived_through(
    world_with_a_camera: World, camera_facing_a_box: CameraFacingABox
) -> None:
    """
    Of two cameras of the type, none is chosen behind the caller's back.
    """
    camera = camera_facing_a_box.camera
    with world_with_a_camera.modify_world():
        world_with_a_camera.add_semantic_annotation(
            Camera(
                name=PrefixedName("second_camera"),
                root=camera.root,
                camera_model=camera.camera_model,
            )
        )

    with pytest.raises(CameraAnnotationAmbiguous):
        RoboKudoSimulatedPerception(camera_type=Camera).mount_cameras(
            world=world_with_a_camera
        )


def test_started_perception_publishes_the_camera_of_its_type(
    started_simulated_perception: RoboKudoSimulatedPerception,
    camera_facing_a_box: CameraFacingABox,
) -> None:
    """
    The analysis engine sees what the camera of the type sees.
    """
    camera_publisher = started_simulated_perception.camera_publisher

    assert camera_publisher.camera is camera_facing_a_box.camera


def test_started_perception_publishes_on_its_topics_and_frames(
    started_simulated_perception: RoboKudoSimulatedPerception,
) -> None:
    """
    The camera reaches the analysis engine where the analysis engine reads it.
    """
    camera_publisher = started_simulated_perception.camera_publisher

    assert camera_publisher.topics is started_simulated_perception.camera_topics
    assert camera_publisher.frames is started_simulated_perception.camera_frames


def test_started_perception_runs_its_analysis_engine(
    started_simulated_perception: RoboKudoSimulatedPerception,
) -> None:
    """
    Queries are answered from the moment the perception started.
    """
    assert started_simulated_perception.analysis_engine.started


def test_capturing_a_frame_publishes_what_the_camera_sees(
    started_simulated_perception: RoboKudoSimulatedPerception,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    The analysis engine gets one frame of the camera to answer the next query about.
    """
    publishing_publishers: list[SimulatedCameraPublisher] = []
    monkeypatch.setattr(
        SimulatedCameraPublisher,
        "publish",
        lambda camera_publisher: publishing_publishers.append(camera_publisher),
    )

    started_simulated_perception.capture_frame()

    assert publishing_publishers == [started_simulated_perception.camera_publisher]


def test_stopping_a_started_perception_stops_its_analysis_engine(
    started_simulated_perception: RoboKudoSimulatedPerception,
) -> None:
    """
    No analysis engine outlives the run it perceived for.
    """
    started_simulated_perception.stop_perceiving()

    assert started_simulated_perception.analysis_engine.stopped


def test_stopping_a_perception_that_never_started_stops_nothing() -> None:
    """
    A run that failed before the perception started has no analysis engine to stop.
    """
    simulated_perception = RoboKudoSimulatedPerception(
        analysis_engine=RecordingAnalysisEngine(
            analysis_engine=AnalysisEngineModule.HUMAN_DETECTION
        )
    )

    simulated_perception.stop_perceiving()

    assert not simulated_perception.analysis_engine.stopped
