"""
The kitchen to sofa demo with the sofa perceived by RoboKudo in a recording of the HSR's
head camera, while the robot itself is simulated.
"""

from __future__ import annotations

import subprocess
import threading
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from tf2_ros import StaticTransformBroadcaster

from coraplex.plans.executors import KinematicallySimulatedPlanExecutor, PlanExecutor
from coraplex.plans.plan_transformation import PlanRewriting
from coraplex.robot_plans.plan_transformations import DetectWithRoboKudo
from cramph.context import ContextExtension
from demo import KitchenToSofaDemonstration
from exceptions import RecordingNotPlaying
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.world_fetcher import FetchWorldServer
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection

# %% what the recordings hold


class RecordedTopic(StrEnum):
    """
    The topics of the HSR's head camera a recording holds and RoboKudo's HSR camera
    configuration reads.
    """

    COLOR = "/head_rgbd_sensor/rgb/image_rect_color/compressed"
    """
    The color images.
    """

    DEPTH = "/head_rgbd_sensor/depth_registered/image/compressedDepth"
    """
    The depth images, registered to the color images.
    """

    CAMERA_INFO = "/head_rgbd_sensor/rgb/camera_info"
    """
    The intrinsics of the color camera.
    """


class RecordedFrame(StrEnum):
    """
    The tf frames RoboKudo's HSR camera configuration looks the camera pose up between.

    The recordings place the camera at a made-up pose in ``map``, so their own tf is
    not replayed.
    """

    MAP = "map"
    """
    The frame the camera pose is looked up in and RoboKudo reports poses in, the root of
    the world on the real robot.
    """

    CAMERA = "head_rgbd_sensor_rgb_optical_frame"
    """
    The optical frame of the color camera, which the images are stamped with.
    """


class BagPlayArgument(StrEnum):
    """
    The parts of the ``ros2 bag play`` command a recording is replayed with.
    """

    COMMAND = "ros2"
    """
    The ROS 2 command line tool.
    """

    VERB = "bag"
    """
    The verb of the tool for recordings.
    """

    PLAY = "play"
    """
    Replay a recording.
    """

    LOOP = "--loop"
    """
    Start the recording again when it ends, so images keep coming however long the
    robot takes to reach the sofa.
    """

    TOPICS = "--topics"
    """
    Replay only the topics that follow.
    """


# %% replaying a recording


@dataclass
class CameraRecordingReplay:
    """
    Replays a recording of the HSR's head camera as if a simulated robot's head camera
    took it, and serves the simulated world to RoboKudo.

    While it runs, RoboKudo sees the recorded images from wherever the simulated head
    camera is.
    """

    recording: Path
    """
    The folder of the ``ros2 bag`` recording.
    """

    node: Node
    """
    The spun node the world and the tf tree are published through.
    """

    world: World
    """
    The simulated world holding the robot and the sofa.
    """

    robot: AbstractRobot
    """
    The simulated robot whose default camera stands in for the recorded camera.
    """

    world_server: FetchWorldServer | None = field(init=False, default=None)
    """
    The service RoboKudo fetches :attr:`world` from, while this replay runs.
    """

    tf_publisher: TFPublisher | None = field(init=False, default=None)
    """
    Publishes the tf tree of :attr:`world`, while this replay runs.
    """

    static_transforms: StaticTransformBroadcaster | None = field(
        init=False, default=None
    )
    """
    Puts the recorded camera frame at the default camera of :attr:`robot`, while this
    replay runs.
    """

    images_before_start: int = 30
    """
    How many replayed color images arrive before :meth:`start` returns, so RoboKudo
    holds an image of :attr:`recording` and not one of an earlier replay.
    """

    image_timeout: timedelta = timedelta(seconds=30)
    """
    How long :meth:`start` waits for those images.
    """

    player: subprocess.Popen | None = field(init=False, default=None)
    """
    The process playing :attr:`recording`, while this replay runs.
    """

    received_image_count: int = field(init=False, default=0)
    """
    How many replayed color images arrived since the replay started.
    """

    enough_images_received: threading.Event = field(
        init=False, default_factory=threading.Event
    )
    """
    Set once :attr:`images_before_start` replayed color images arrived.
    """

    def start(self) -> None:
        """
        Serve :attr:`world`, publish its tf tree with the recorded camera frame fixed to
        the default camera of :attr:`robot` and play :attr:`recording` until
        :attr:`images_before_start` of its color images have arrived.

        :raises RecordingNotPlaying: If they do not arrive within :attr:`image_timeout`.
        """
        self.world_server = FetchWorldServer(node=self.node, world=self.world)
        self.tf_publisher = TFPublisher(node=self.node, _world=self.world)
        self.fix_recorded_camera_to_robot()
        self.play_until_images_arrive()

    def fix_recorded_camera_to_robot(self) -> None:
        """
        Publish the recorded camera frame as coinciding with the default camera of
        :attr:`robot`, whose frame is an optical frame too.
        """
        camera_to_recorded_camera = TransformStamped()
        camera_to_recorded_camera.header.stamp = self.node.get_clock().now().to_msg()
        camera_to_recorded_camera.header.frame_id = (
            self.tf_publisher.frame_names.assign(self.robot.get_default_camera().root)
        )
        camera_to_recorded_camera.child_frame_id = RecordedFrame.CAMERA
        camera_to_recorded_camera.transform.rotation.w = 1.0
        self.static_transforms = StaticTransformBroadcaster(self.node)
        self.static_transforms.sendTransform(camera_to_recorded_camera)

    def play_until_images_arrive(self) -> None:
        """
        Start playing :attr:`recording` and wait for :attr:`images_before_start` of its
        color images.

        :raises RecordingNotPlaying: If they do not arrive within :attr:`image_timeout`.
        """
        self.received_image_count = 0
        self.enough_images_received.clear()
        subscription = self.node.create_subscription(
            CompressedImage,
            RecordedTopic.COLOR,
            self.count_image,
            self.images_before_start,
        )
        self.player = subprocess.Popen(
            [
                BagPlayArgument.COMMAND,
                BagPlayArgument.VERB,
                BagPlayArgument.PLAY,
                str(self.recording),
                BagPlayArgument.LOOP,
                BagPlayArgument.TOPICS,
                *RecordedTopic,
            ]
        )
        delivered = self.enough_images_received.wait(self.image_timeout.total_seconds())
        self.node.destroy_subscription(subscription)
        if not delivered:
            raise RecordingNotPlaying(
                recording=self.recording, timeout=self.image_timeout
            )

    def count_image(self, image: CompressedImage) -> None:
        """
        Count a replayed color image, and signal once :attr:`images_before_start`
        arrived.

        :param image: The replayed color image.
        """
        self.received_image_count += 1
        if self.received_image_count >= self.images_before_start:
            self.enough_images_received.set()

    def stop(self) -> None:
        """
        Stop playing the recording, publishing the tf tree and serving the world.
        """
        if self.player is not None:
            self.player.terminate()
            self.player.wait()
            self.player = None
        if self.tf_publisher is not None:
            self.tf_publisher.stop()
            self.tf_publisher = None
        if self.world_server is not None:
            self.world_server.close()
            self.world_server = None
        self.static_transforms = None


# %% the demonstration


@dataclass
class RecordedKitchenToSofaDemonstration(KitchenToSofaDemonstration):
    """
    The kitchen to sofa demonstration with a kinematically simulated robot, whose
    perception RoboKudo answers from :attr:`recording` instead of the world model.

    .. warning:: Needs RoboKudo answering seat queries, see the README.
    """

    recording: Path = field(kw_only=True)
    """
    The folder of the ``ros2 bag`` recording of the HSR's head camera looking at the
    sofa.
    """

    executor_type: type[PlanExecutor] = KinematicallySimulatedPlanExecutor
    """
    The executor running the plan, which has to simulate the robot.
    """

    replay: CameraRecordingReplay | None = field(init=False, default=None)
    """
    The replay of :attr:`recording`, while the demonstration runs.
    """

    def build_scene(self) -> World:
        """
        As :meth:`KitchenToSofaDemonstration.build_scene`, fixed below a root named like
        the frame RoboKudo reports poses in, as the real robot's world is.
        """
        world = World.create_with_root_body(str(RecordedFrame.MAP))
        scene = super().build_scene()
        world.merge_world(
            scene, root_connection=FixedConnection(parent=world.root, child=scene.root)
        )
        return world

    def acquire_world(self) -> World:
        """
        As :meth:`KitchenToSofaDemonstration.acquire_world`, and start replaying
        :attr:`recording` from the head camera of the robot in the world.
        """
        world = super().acquire_world()
        self.replay = CameraRecordingReplay(
            recording=self.recording,
            node=self.ros_node,
            world=world,
            robot=world.get_semantic_annotations_by_type(self.used_robot)[0],
        )
        self.replay.start()
        return world

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        """
        As :meth:`KitchenToSofaDemonstration.build_context_extensions`, with the
        perception answered by RoboKudo.
        """
        return [
            *super().build_context_extensions(world),
            PlanRewriting(transformations=[DetectWithRoboKudo()]),
        ]

    def tear_down(self) -> None:
        """
        Stop the replay, then as :meth:`KitchenToSofaDemonstration.tear_down`.
        """
        if self.replay is not None:
            self.replay.stop()
            self.replay = None
        super().tear_down()
