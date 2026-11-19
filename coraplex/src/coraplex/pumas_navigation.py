"""
Driving a real robot's base with the Pumas navigation of the lab, which plans its path
on the laser map.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.publisher import Publisher

from coraplex.exceptions import NavigationGoalNotHeard, NavigationTimedOut
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.adapters.ros.semdt_to_ros2_converters import (
    PoseToRos2StampedConverter,
)
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper
from semantic_digital_twin.robots.robot_parts import AbstractRobot

# %% the Pumas interface


class PumasTopic(StrEnum):
    """
    The topics the Pumas navigation of the lab listens on.
    """

    GOAL = "/move_base_simple/goal"
    """
    Where to drive to, a pose in the map frame; RViz's *2D Goal Pose* tool sends there.
    """


# %% driving with Pumas


@dataclass
class PumasNavigation:
    """
    Sends a pose to the running Pumas navigation and waits until the robot's base stands
    there, as tf reports it.

    Pumas reports no outcome, so arriving is judged by :attr:`position_threshold` and
    :attr:`orientation_threshold` alone.
    """

    node: Node
    """
    The spun node the goal is published and tf is listened to through.
    """

    robot: AbstractRobot
    """
    The robot whose base is driven.
    """

    position_threshold: float = 0.3
    """
    How far the base may stand from the goal, in meters, to count as arrived.
    """

    orientation_threshold: float = 0.1
    """
    How far the base may be turned away from the goal, in radians, to count as arrived.
    """

    timeout: timedelta = timedelta(minutes=3)
    """
    How long the base may take to arrive, and the goal topic to be listened to.
    """

    polling_period: timedelta = timedelta(seconds=0.1)
    """
    How often the base pose is looked up while waiting.
    """

    goal_publisher: Publisher = field(init=False)
    """
    Publishes the goals on :attr:`PumasTopic.GOAL`.
    """

    tf: TFWrapper = field(init=False)
    """
    Looks up where the base stands.
    """

    def __post_init__(self) -> None:
        """
        Create :attr:`goal_publisher` and :attr:`tf` on :attr:`node`.
        """
        self.goal_publisher = self.node.create_publisher(
            PoseStamped, PumasTopic.GOAL, 1
        )
        self.tf = TFWrapper(node=self.node)

    def drive_to(self, goal: Pose) -> None:
        """
        Have Pumas drive the base to ``goal`` and wait until it stands there.

        :param goal: Where the base should stand, in the root frame of the robot's
            world, which is the map frame Pumas plans in.
        :raises NavigationGoalNotHeard: If Pumas does not listen within :attr:`timeout`.
        :raises NavigationTimedOut: If the base does not arrive within :attr:`timeout`.
        """
        world = self.robot._world
        map_frame = str(world.root.name.name)
        deadline = time.monotonic() + self.timeout.total_seconds()
        while self.goal_publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise NavigationGoalNotHeard(topic=PumasTopic.GOAL)
            time.sleep(self.polling_period.total_seconds())

        map_T_goal = world.transform(goal, world.root)
        message = PoseToRos2StampedConverter.convert(map_T_goal)
        message.header.frame_id = map_frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.goal_publisher.publish(message)

        deadline = time.monotonic() + self.timeout.total_seconds()
        while True:
            map_T_base = self.tf.lookup_transform(
                map_frame, str(self.robot.root.name.name)
            ).transform
            base_pose = Pose.from_xyz_quaternion(
                map_T_base.translation.x,
                map_T_base.translation.y,
                map_T_base.translation.z,
                map_T_base.rotation.x,
                map_T_base.rotation.y,
                map_T_base.rotation.z,
                map_T_base.rotation.w,
                reference_frame=world.root,
            )
            distance = np.hypot(
                float(base_pose.x) - float(map_T_goal.x),
                float(base_pose.y) - float(map_T_goal.y),
            )
            yaw_difference = float(base_pose.yaw) - float(map_T_goal.yaw)
            yaw_error = abs(np.arctan2(np.sin(yaw_difference), np.cos(yaw_difference)))
            if (
                distance <= self.position_threshold
                and yaw_error <= self.orientation_threshold
            ):
                return
            if time.monotonic() >= deadline:
                raise NavigationTimedOut(goal=map_T_goal, timeout=self.timeout)
            time.sleep(self.polling_period.total_seconds())
