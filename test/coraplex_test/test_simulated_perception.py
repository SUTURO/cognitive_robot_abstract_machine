"""
Tests for :class:`coraplex.simulated_perception.SimulatedHsrHeadCameraPerception`: the
analysis engine runs exactly while the head camera is published.
"""

from __future__ import annotations

# %% Imports
from dataclasses import dataclass, field
from datetime import timedelta

import pytest
from typing_extensions import Iterator

from coraplex.exceptions import SimulatedPerceptionNotStarted
from coraplex.perception import CameraFrameCapture, PerceptionWorldFrame
from coraplex.simulated_perception import SimulatedHsrHeadCameraPerception
from robokudo.analysis_engine_process import AnalysisEngineProcess
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB, HSRBHeadRGBDCamera
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from ..pytest_environment import runs_in_continuous_integration

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

    runs: bool = field(init=False, default=False)
    """
    Whether the engine was started and not stopped since.
    """

    def start(self, timeout: timedelta = timedelta(minutes=2)) -> None:
        self.runs = True

    def stop(self) -> None:
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
    rclpy_node,
) -> None:
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    [camera] = hsr_world.get_semantic_annotations_by_type(HSRBHeadRGBDCamera)
    assert perception.camera_publisher.camera is camera
    assert perception.analysis_engine.runs


def test_context_captures_the_frame_through_the_publisher(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node,
) -> None:
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    [capture] = [
        extension
        for extension in perception.build_context_extensions(hsr_world)
        if isinstance(extension, CameraFrameCapture)
    ]
    assert capture == CameraFrameCapture(perception.camera_publisher.publish)


def test_context_places_the_published_world_frame_at_the_world_root(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node,
) -> None:
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    [world_frame] = [
        extension
        for extension in perception.build_context_extensions(hsr_world)
        if isinstance(extension, PerceptionWorldFrame)
    ]
    assert world_frame == PerceptionWorldFrame(
        reported_name=perception.camera_configuration.tf_to, entity=hsr_world.root
    )


def test_stop_stops_the_engine_and_the_publishing(
    perception: SimulatedHsrHeadCameraPerception,
    hsr_world: World,
    hsr_head_camera_simulation: MujocoSim,
    rclpy_node,
) -> None:
    perception.start(hsr_world, hsr_head_camera_simulation, rclpy_node)
    perception.stop()
    assert not perception.analysis_engine.runs
    assert perception.camera_publisher is None


# %% Before starting


def test_context_before_start_is_refused(
    perception: SimulatedHsrHeadCameraPerception,
) -> None:
    with pytest.raises(SimulatedPerceptionNotStarted):
        perception.build_context_extensions(World())


def test_stop_before_start_leaves_the_engine_alone(
    perception: SimulatedHsrHeadCameraPerception,
) -> None:
    perception.stop()
    assert not perception.analysis_engine.runs
