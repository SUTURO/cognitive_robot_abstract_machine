"""
RoboKudo perceiving through the simulated head camera of the HSR.
"""

from __future__ import annotations

# %% Imports
from dataclasses import dataclass, field

from typing_extensions import TYPE_CHECKING

from coraplex.exceptions import (
    NoSingleHsrHeadCamera,
    SimulatedPerceptionAlreadyStarted,
    SimulatedPerceptionNotStarted,
)
from coraplex.perception import CameraFrameCapture, PerceptionWorldFrame
from robokudo.analysis_engine_process import AnalysisEngineProcess
from robokudo.descriptors.camera_configs.config_hsr_ros2 import HsrRos2CameraConfig
from robokudo.io.simulated_camera import SimulatedCameraPublisher
from semantic_digital_twin.robots.hsrb import HSRBHeadRGBDCamera

if TYPE_CHECKING:
    from cramph.context import ContextExtension
    from rclpy.node import Node
    from semantic_digital_twin.adapters.multi_sim import MujocoSim
    from semantic_digital_twin.world import World


# %% Perception through the simulated head camera


@dataclass
class SimulatedHsrHeadCameraPerception:
    """
    Perception of an HSR in a MuJoCo simulation: its head camera, published the way the
    real HSR publishes it, and a RoboKudo analysis engine asked about what it sees.

    .. note:: The head camera has to be mounted in the simulation with
        :meth:`~semantic_digital_twin.adapters.multi_sim.MujocoCamera.mount_hsr_head`
        before the simulation is built.
    """

    analysis_engine: AnalysisEngineProcess
    """The RoboKudo pipeline that is asked what the head camera sees."""

    camera_configuration: HsrRos2CameraConfig = field(
        default_factory=HsrRos2CameraConfig
    )
    """
    The topics and frames the head camera is published on, those of the real HSR.
    """

    camera_publisher: SimulatedCameraPublisher | None = field(init=False, default=None)
    """
    Publishes the head camera between :meth:`start` and :meth:`stop`, ``None``
    otherwise.
    """

    def start(self, world: World, simulation: MujocoSim, node: Node) -> None:
        """
        Start :attr:`analysis_engine`, waiting until it runs, and then prepare
        publishing the head camera of the one HSR in ``world``.

        :param world: The world holding the HSR.
        :param simulation: The running simulation the head camera is mounted in.
        :param node: The node to publish the head camera through.
        :raises SimulatedPerceptionAlreadyStarted: If this perception already runs.
        :raises NoSingleHsrHeadCamera: If ``world`` does not hold exactly one head
            camera of an HSR.
        """
        if self.camera_publisher is not None:
            raise SimulatedPerceptionAlreadyStarted()
        cameras = world.get_semantic_annotations_by_type(HSRBHeadRGBDCamera)
        if len(cameras) != 1:
            raise NoSingleHsrHeadCamera(number_of_cameras=len(cameras))
        [camera] = cameras
        self.analysis_engine.start()
        self.camera_publisher = SimulatedCameraPublisher(
            camera=camera,
            simulation=simulation,
            node=node,
            topics=self.camera_configuration,
            frames=self.camera_configuration,
        )

    def build_context_extensions(self) -> list[ContextExtension]:
        """
        :return: What a plan needs to capture the frame :attr:`analysis_engine` is
            asked about, and to place what it reports in the simulated world.
        :raises SimulatedPerceptionNotStarted: If called outside :meth:`start` and
            :meth:`stop`.
        """
        if self.camera_publisher is None:
            raise SimulatedPerceptionNotStarted()
        return [
            CameraFrameCapture(self.camera_publisher.publish),
            PerceptionWorldFrame(
                reported_name=self.camera_configuration.tf_to,
                entity=self.camera_publisher.simulation.world.root,
            ),
        ]

    def stop(self) -> None:
        """
        Stop :attr:`analysis_engine` and the publishing of the head camera, if
        :meth:`start` started them.
        """
        if self.camera_publisher is None:
            return
        self.analysis_engine.stop()
        self.camera_publisher.destroy_publishers()
        self.camera_publisher = None
