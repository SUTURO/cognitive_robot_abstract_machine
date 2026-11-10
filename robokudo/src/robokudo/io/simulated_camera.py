"""
A camera driver for a simulated robot: what a camera sees in MuJoCo, published the way the
RGB-D camera driver of the real robot publishes it.
"""

from __future__ import annotations

# %% Imports

import struct
from dataclasses import dataclass, field
from enum import StrEnum
from typing_extensions import TYPE_CHECKING

import cv2
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from robokudo.io.camera_model_adapters import RosCameraModelAdapter
from semantic_digital_twin.datastructures.camera_model import PinholeCameraModel
from semantic_digital_twin.spatial_computations.raytracer import RayTracer

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from robokudo.descriptors.camera_configs.components import (
        RGBDComponent,
        TfComponent,
    )
    from semantic_digital_twin.adapters.multi_sim import MujocoSim
    from semantic_digital_twin.datastructures.camera_model import FieldOfViewCameraModel
    from semantic_digital_twin.robots.robot_parts import Camera

# %% Messages of one frame


class CompressedImageFormat(StrEnum):
    """
    The formats of the compressed images an RGB-D camera driver publishes.
    """

    COLOR = "bgr8; jpeg compressed bgr8"
    """
    A JPEG color image.
    """

    DEPTH = "16UC1; compressedDepth"
    """
    A PNG of depths in millimeters with 16 bits per pixel, after a configuration header.
    """


@dataclass
class CameraMessages:
    """
    Messages for one camera frame, all with the same timestamp.
    """

    color_image: CompressedImage
    """
    The color image.
    """

    depth_image: CompressedImage
    """
    The depth image, aligned with the color image.
    """

    camera_info: CameraInfo
    """
    The calibration both images were taken with.
    """

    camera_transform: TransformStamped
    """
    The pose of the camera's optical frame in the world frame.
    """


# %% Publishing


@dataclass
class SimulatedCameraPublisher:
    """
    Publish simulated camera frames on the configured RoboKudo topics and TF frames.

    The simulated world root is the world frame.
    """

    camera: Camera[FieldOfViewCameraModel]
    """
    The camera whose images to publish.
    """

    simulation: MujocoSim
    """
    The running simulation the camera is mounted in.
    """

    node: Node
    """
    The node to publish through.
    """

    topics: RGBDComponent
    """
    The configuration naming the topics of the images and of the calibration.
    """

    frames: TfComponent
    """
    The configuration naming the camera's optical frame and the world frame.
    """

    maximum_range: float = 10.0
    """
    The farthest depth in meters the camera measures. Farther depths read as missing.
    """

    _color_publisher: Publisher[CompressedImage] = field(init=False)
    """
    Publishes the color images.
    """

    _depth_publisher: Publisher[CompressedImage] = field(init=False)
    """
    Publishes the depth images.
    """

    _camera_info_publisher: Publisher[CameraInfo] = field(init=False)
    """
    Publishes the calibration.
    """

    _transform_broadcaster: TransformBroadcaster = field(init=False)
    """
    Publishes the pose of the camera.
    """

    def __post_init__(self) -> None:
        """Create the ROS publishers for camera frames."""
        self._color_publisher = self.node.create_publisher(
            CompressedImage, self.topics.topic_color, 1
        )
        self._depth_publisher = self.node.create_publisher(
            CompressedImage, self.topics.topic_depth, 1
        )
        self._camera_info_publisher = self.node.create_publisher(
            CameraInfo, self.topics.topic_camera_info, 1
        )
        self._transform_broadcaster = TransformBroadcaster(self.node)

    def capture(self) -> CameraMessages:
        """
        :return: What the camera sees now, as the messages its driver publishes.
        """
        simulator = self.simulation.simulator
        resolution = self.camera.resolution
        camera_name = self.camera.root.name.name
        frame = simulator.capture_rgbd(
            camera_name, resolution.height, resolution.width
        ).result
        color = frame.color
        depth = frame.depth
        depth[depth > self.maximum_range] = -1.0

        header = Header(
            stamp=self.node.get_clock().now().to_msg(), frame_id=self.frames.tf_from
        )
        color_image = CompressedImage(
            header=header,
            format=CompressedImageFormat.COLOR,
            data=cv2.imencode(".jpg", cv2.cvtColor(color, cv2.COLOR_RGB2BGR))[
                1
            ].tobytes(),
        )
        depth_image = CompressedImage(
            header=header,
            format=CompressedImageFormat.DEPTH,
            data=bytes(struct.calcsize("iff"))
            + cv2.imencode(".png", RayTracer.convert_depth_to_millimeters(depth))[
                1
            ].tobytes(),
        )
        camera_info = RosCameraModelAdapter.to_camera_info(
            PinholeCameraModel.from_field_of_view(
                resolution=resolution, field_of_view=self.camera.field_of_view
            ),
            frame_id=self.frames.tf_from,
        )
        camera_info.header = header
        camera_transform = TransformStamped()
        camera_transform.header = Header(stamp=header.stamp, frame_id=self.frames.tf_to)
        camera_transform.child_frame_id = self.frames.tf_from
        camera_transform.transform.translation.x = float(frame.camera_position[0])
        camera_transform.transform.translation.y = float(frame.camera_position[1])
        camera_transform.transform.translation.z = float(frame.camera_position[2])
        camera_transform.transform.rotation.w = float(frame.camera_quaternion[0])
        camera_transform.transform.rotation.x = float(frame.camera_quaternion[1])
        camera_transform.transform.rotation.y = float(frame.camera_quaternion[2])
        camera_transform.transform.rotation.z = float(frame.camera_quaternion[3])
        return CameraMessages(
            color_image=color_image,
            depth_image=depth_image,
            camera_info=camera_info,
            camera_transform=camera_transform,
        )

    def publish(self) -> None:
        """
        Publish what the camera sees now.
        """
        messages = self.capture()
        self._transform_broadcaster.sendTransform(messages.camera_transform)
        self._color_publisher.publish(messages.color_image)
        self._depth_publisher.publish(messages.depth_image)
        self._camera_info_publisher.publish(messages.camera_info)
