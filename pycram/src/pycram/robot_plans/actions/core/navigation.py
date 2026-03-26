from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

import numpy as np
from typing_extensions import Union, Optional, Type, Any, Iterable

from pycram.datastructures.dataclasses import Context

# from pycram_suturo_demos.pycram_advanced_hsr_demos.bring_object_from_table_to_shelf_demo import pose_to_ros
from semantic_digital_twin.robots.abstract_robot import Camera
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.motions.robot_body import LookingMotion
from pycram.robot_plans.motions.navigation import MoveMotion
from pycram.config.action_conf import ActionConfig
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import PoseStamped
from pycram.failures import NavigationGoalNotReachedError
from pycram.language import SequentialPlan
from pycram.validation.error_checkers import PoseErrorChecker
from semantic_digital_twin.spatial_types.spatial_types import Pose
import geometry_msgs.msg


@dataclass
class NavigateAction(ActionDescription):
    """
    Navigates the Robot to a position.
    """

    target_location: PoseStamped
    """
    Location to which the robot should be navigated
    """

    keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    def execute(self) -> None:
        return SequentialPlan(
            self.context, MoveMotion(self.target_location, self.keep_joint_states)
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        pose_validator = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_validator.is_error_acceptable(
            World.robot.pose, self.target_location
        ):
            raise NavigationGoalNotReachedError(World.robot.pose, self.target_location)

    @classmethod
    def description(
        cls,
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        keep_joint_states: Union[
            Iterable[bool], bool
        ] = ActionConfig.navigate_keep_joint_states,
    ) -> PartialDesignator[NavigateAction]:
        return PartialDesignator[NavigateAction](
            NavigateAction,
            target_location=target_location,
            keep_joint_states=keep_joint_states,
        )


@dataclass
class nav2NavigateAction(ActionDescription):
    """
    Navigates the Robot to a position.
    """

    target_location: PoseStamped | Pose
    """
    Location to which the robot should be navigated
    """
    simulated: bool = False
    """
    variable to indcate we are in sim
    """

    def execute(self) -> None:
        from pycram.external_interfaces import nav2_move

        if Pose:
            self.target_location = self.pose_to_ros(self.target_location)
        if self.simulated:
            SequentialPlan(
                self.context, MoveMotion(self.target_location, self.keep_joint_states)
            ).perform()
        else:
            nav2_move.start_nav_to_pose(self.target_location)

    def pose_to_ros(self, pose: Pose) -> PoseStamped:
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose.position.x = float(pose.x)
        pose_stamped.pose.position.y = float(pose.y)
        pose_stamped.pose.position.z = float(pose.z)
        pose_stamped.pose.orientation.x = float(pose.to_quaternion().x)
        pose_stamped.pose.orientation.y = float(pose.to_quaternion().y)
        pose_stamped.pose.orientation.z = float(pose.to_quaternion().z)
        pose_stamped.pose.orientation.w = float(pose.to_quaternion().w)
        pose_stamped.header.frame_id = pose.reference_frame.name.name
        return pose_stamped

    @classmethod
    def description(
        cls,
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        simulated: bool = False,
    ) -> PartialDesignator[NavigateAction]:
        return PartialDesignator[NavigateAction](
            NavigateAction,
            simlated=simulated,
            target_location=target_location,
        )


@dataclass
class LookAtAction(ActionDescription):
    """
    Lets the robot look at a position.
    """

    target: PoseStamped
    """
    Position at which the robot should look, given as 6D pose
    """

    camera: Camera = None
    """
    Camera that should be looking at the target
    """

    def execute(self) -> None:
        camera = self.camera or self.robot_view.get_default_camera()
        SequentialPlan(
            self.context, LookingMotion(target=self.target, camera=camera)
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if the robot is looking at the target location by spawning a virtual object at the target location and
        creating a ray from the camera and checking if it intersects with the object.
        """
        return

    @classmethod
    def description(
        cls,
        target: Union[Iterable[PoseStamped], PoseStamped],
        camera: Optional[Union[Iterable[Camera], Camera]] = None,
    ) -> PartialDesignator[LookAtAction]:
        return PartialDesignator[LookAtAction](
            LookAtAction, target=target, camera=camera
        )


NavigateActionDescription = NavigateAction.description
nav2NavigateActionDescription = nav2NavigateAction.description
LookAtActionDescription = LookAtAction.description
