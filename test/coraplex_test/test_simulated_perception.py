"""
Tests for :class:`coraplex.simulated_perception.SimulatedHsrHeadCameraPerception`: the
analysis engine runs exactly while the head camera is published.
"""

from __future__ import annotations

# %% Imports
from dataclasses import dataclass, field
from datetime import timedelta

import pytest
from typing_extensions import TYPE_CHECKING, Iterator

from coraplex.exceptions import (
    NoSingleHsrHeadCamera,
    SimulatedPerceptionAlreadyStarted,
    SimulatedPerceptionNotStarted,
)
from coraplex.perception import CameraFrameCapture, PerceptionWorldFrame
from coraplex.simulated_perception import SimulatedHsrHeadCameraPerception
from robokudo.analysis_engine_process import AnalysisEngineProcess
from robokudo.exceptions import AnalysisEngineDidNotStart
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB, HSRBHeadRGBDCamera
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from ..pytest_environment import runs_in_continuous_integration

if TYPE_CHECKING:
    from rclpy.node import Node

pytestmark = [
    pytest.mark.skipif(
        not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
    ),
    pytest.mark.skipif(
        not HSRB.is_description_installed(),
        reason="The description of the HSR is not installed",
    ),
]


# %% A stand-in for the analysis engine process


@dataclass
class RecordedAnalysisEngine(AnalysisEngineProcess):
    """
    An analysis engine that records whether it runs instead of starting a RoboKudo
    process.
    """

    fails_to_start: bool = False
    """
    Whether starting the engine fails as if it never announced that it runs.
    """

    runs: bool = field(init=False, default=False)
    """
    Whether the engine was started and not stopped since.
    """

    start_count: int = field(init=False, default=0)
    """
    How often the engine was asked to start.
    """

    stop_count: int = field(init=False, default=0)
    """
    How often the engine was asked to stop.
    """

    def start(self, timeout: timedelta = timedelta(minutes=2)) -> None:
        """
        Record the start instead of starting a process as
        :meth:`AnalysisEngineProcess.start` does, or fail if :attr:`fails_to_start`.
        """
        self.start_count += 1
        if self.fails_to_start:
            raise AnalysisEngineDidNotStart(
                analysis_engine=self.analysis_engine,
                timeout_seconds=timeout.total_seconds(),
            )
        self.runs = True

    def stop(self) -> None:
        """
        Record the stop instead of stopping a process as
        :meth:`AnalysisEngineProcess.stop` does.
        """
        self.stop_count += 1
        self.runs = False


# %% Fixtures


@pytest.fixture
def hsr_world() -> World:
    """
    A world holding nothing but a floor and the HSR, spawned with its head camera.
    """
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(Body(name=PrefixedName("floor")))
    RobotSpecification(HSRB).spawn(world)
    return world


@pytest.fixture
def hsr_head_camera_simulation(hsr_world: World) -> Iterator[MujocoSim]:
    """
    A running simulation of the HSR with a camera mounted at its head.
    """
    MujocoCamera.mount_hsr_head(hsr_world)
    simulation = MujocoSim(world=hsr_world, headless=True)
    simulation.start_stepped_simulation()
    yield simulation
    simulation.stop_simulation()


@pytest.fixture
def perception() -> SimulatedHsrHeadCameraPerception:
    """
    Perception through the head camera, by an engine that starts no process.
    """
    return SimulatedHsrHeadCameraPerception(
        analysis_engine=RecordedAnalysisEngine(analysis_engine="human_detection")
    )


# %% Starting and stopping


def test_start_publishes_the_head_camera_and_runs_the_engine(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    Starting publishes the head camera of the HSR and runs the engine.
    """
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    [camera] = hsr_world.get_semantic_annotations_by_type(HSRBHeadRGBDCamera)
    assert perception.camera_publisher.camera is camera
    assert perception.analysis_engine.runs


def test_context_captures_the_frame_through_the_publisher(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    The plan captures each frame through the head camera's publisher.
    """
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    [capture] = [
        extension
        for extension in perception.build_context_extensions()
        if isinstance(extension, CameraFrameCapture)
    ]
    assert capture == CameraFrameCapture(perception.camera_publisher.publish)


def test_context_places_the_published_world_frame_at_the_world_root(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    The world frame the camera is published in stands for the simulated world's root.
    """
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    [world_frame] = [
        extension
        for extension in perception.build_context_extensions()
        if isinstance(extension, PerceptionWorldFrame)
    ]
    assert world_frame == PerceptionWorldFrame(
        reported_name=perception.camera_configuration.tf_to, entity=hsr_world.root
    )


def test_stop_stops_the_engine_and_the_publishing(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    Stopping stops the engine and the publishing of the head camera.
    """
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    perception.stop()
    assert not perception.analysis_engine.runs
    assert perception.camera_publisher is None


# %% Before starting


def test_context_before_start_is_refused(
    perception: SimulatedHsrHeadCameraPerception,
) -> None:
    """
    A perception that never started gives a plan nothing to capture frames with.
    """
    with pytest.raises(SimulatedPerceptionNotStarted):
        perception.build_context_extensions()


def test_stop_before_start_leaves_the_engine_alone(
    perception: SimulatedHsrHeadCameraPerception,
) -> None:
    """
    Stopping a perception that never started does not ask the engine to stop.
    """
    perception.stop()
    assert perception.analysis_engine.stop_count == 0


# %% Refusing to start


def test_starting_twice_is_refused(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    A running perception refuses to start again instead of starting a second engine.
    """
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    with pytest.raises(SimulatedPerceptionAlreadyStarted):
        perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    assert perception.analysis_engine.start_count == 1


def test_world_without_an_hsr_head_camera_is_refused(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    A world without the head camera of an HSR leaves nothing to perceive through, so the
    engine is not started.
    """
    with pytest.raises(NoSingleHsrHeadCamera) as raised:
        perception.start(World(), hsr_head_camera_simulation, rclpy_node)
    assert raised.value.number_of_cameras == 0
    assert perception.analysis_engine.start_count == 0


def test_failed_engine_start_leaves_the_perception_stopped(
    hsr_world: World, hsr_head_camera_simulation: MujocoSim, rclpy_node: Node
) -> None:
    """
    An engine that fails to start leaves the perception unable to serve a plan.
    """
    perception = SimulatedHsrHeadCameraPerception(
        analysis_engine=RecordedAnalysisEngine(
            analysis_engine="human_detection", fails_to_start=True
        )
    )
    with pytest.raises(AnalysisEngineDidNotStart):
        perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    with pytest.raises(SimulatedPerceptionNotStarted):
        perception.build_context_extensions()


# %% Releasing the camera topics


def test_stop_releases_the_head_camera_topics(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    After stopping, the node publishes on none of the head camera's topics any more.
    """
    configuration = perception.camera_configuration
    camera_topics = {
        configuration.topic_color,
        configuration.topic_depth,
        configuration.topic_camera_info,
    }
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    perception.stop()
    published_topics = {publisher.topic_name for publisher in rclpy_node.publishers}
    assert published_topics.isdisjoint(camera_topics)


def test_context_after_stop_is_refused(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node: Node,
) -> None:
    """
    A stopped perception gives a plan nothing to capture frames with.
    """
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    perception.stop()
    with pytest.raises(SimulatedPerceptionNotStarted):
        perception.build_context_extensions()
