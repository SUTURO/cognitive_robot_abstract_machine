from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from typing_extensions import Optional, Any

from pycram.config.action_conf import ActionConfig
from pycram.plans.factories import execute_single
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.motions.navigation import MoveMotion
from pycram.robot_plans.motions.robot_body import LookingMotion
from semantic_digital_twin.robots.abstract_robot import Camera
from semantic_digital_twin.spatial_types.spatial_types import Pose


@dataclass
class NavigateAction(ActionDescription):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Location to which the robot should be navigated
    """

    keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    def execute(self) -> None:
        self.add_subplan(
            execute_single(MoveMotion(self.target_location, self.keep_joint_states))
        ).perform()


# todo fix this
#
# @dataclass
# class nav2NavigateAction(ActionDescription):
#     """
#     Navigates the Robot to a position.
#     """
#
#     target_location: PoseStamped | Pose
#     """
#     Location to which the robot should be navigated
#     """
#     simulated: bool = False
#     """
#     variable to indcate we are in sim
#     """
#
#     def execute(self) -> None:
#         from pycram.external_interfaces import nav2_move
#
#         if isinstance(self.target_location, Pose):
#             self.target_location = self.pose_to_ros(self.target_location)
#         if self.simulated:
#             SequentialPlan(
#                 self.context, MoveMotion(self.target_location, True)
#             ).perform()
#         else:
#             nav2_move.start_nav_to_pose(self.target_location)
#
#     def pose_to_ros(self, pose: Pose) -> PoseStamped:
#         pose_stamped = geometry_msgs.msg.PoseStamped()
#         pose_stamped.pose.position.x = float(pose.x)
#         pose_stamped.pose.position.y = float(pose.y)
#         pose_stamped.pose.position.z = float(pose.z)
#         pose_stamped.pose.orientation.x = float(pose.to_quaternion().x)
#         pose_stamped.pose.orientation.y = float(pose.to_quaternion().y)
#         pose_stamped.pose.orientation.z = float(pose.to_quaternion().z)
#         pose_stamped.pose.orientation.w = float(pose.to_quaternion().w)
#         pose_stamped.header.frame_id = pose.reference_frame.name.name
#         return pose_stamped
#
#     @classmethod
#     def description(
#         cls,
#         target_location: Union[Iterable[PoseStamped], PoseStamped],
#         simulated: bool = False,
#     ) -> PartialDesignator[nav2NavigateAction]:
#         return PartialDesignator[nav2NavigateAction](
#             nav2NavigateAction,
#             simulated=simulated,
#             target_location=target_location,
#         )


@dataclass
class LookAtAction(ActionDescription):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose
    """

    camera: Camera = None
    """
    Camera that should be looking at the target
    """

    def execute(self) -> None:
        camera = self.camera or self.robot.get_default_camera()
        self.add_subplan(
            execute_single(LookingMotion(target=self.target, camera=camera))
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
LookAtActionDescription = LookAtAction.description
