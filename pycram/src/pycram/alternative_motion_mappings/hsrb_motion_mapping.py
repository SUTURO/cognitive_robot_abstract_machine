try:
    from nav2_msgs.action import NavigateToPose
except ModuleNotFoundError:
    NavigateToPose = None
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.ros2_nodes.ros_tasks import (
    NavigateActionServerTask,
)
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.robots.hsrb import HSRB
from ..datastructures.enums import ExecutionType
from ..robot_description import ViewManager
from ..robot_plans import MoveMotion, MoveTCPMotion, MoveGripperMotion

from ..robot_plans.motions.base import AlternativeMotion


class HSRBMoveMotion(MoveMotion, AlternativeMotion[HSRB]):
    execution_type = ExecutionType.REAL

    def perform(self):
        return

    @property
    def _motion_chart(self) -> NavigateActionServerTask:
        return NavigateActionServerTask(
            target_pose=self.target.to_spatial_type(),
            base_link=self.robot_view.root,
            action_topic="/hsrb/move_base",
            message_type=NavigateToPose,
        )


#
# class HSRBMoveGripperMotion(MoveGripperMotion, AlternativeMotion[HSRB]):
#     execution_type = ExecutionType.REAL
#
#     def perform(self):
#         return
#
#     @property
#     def _motion_chart(self) -> GripperActionServerTask:
#         gripper_state = self.motion
#         if gripper_state.OPEN:
#             gripper_effort = 0.8
#         else:
#             gripper_effort = 0.2
#         return GripperActionServerTask(
#             gripper_effort=gripper_effort,  # Effort-Wert für das Öffnen des Grippers
#             action_topic='/gripper_controller/grasp'
#         )


class HSRMoveTCPSim(MoveTCPMotion, AlternativeMotion[HSRB]):

    execution_type = ExecutionType.SIMULATED

    def perform(self):
        return

    @property
    def _motion_chart(self) -> CartesianPose:
        tip = self.robot_view._world.get_semantic_annotations_by_type(ParallelGripper)[
            0
        ]
        return CartesianPose(
            root_link=self.world.root,
            tip_link=tip,
            goal_pose=self.target.to_spatial_type(),
        )


class HSRMoveTCPReal(MoveTCPMotion, AlternativeMotion[HSRB]):

    execution_type = ExecutionType.REAL

    def perform(self):
        return

    @property
    def _motion_chart(self) -> CartesianPose:
        # tip = self.robot_view.arms[0]
        # Temp hack from Simon
        tip = self.robot_view._world.get_semantic_annotations_by_type(ParallelGripper)[
            0
        ]
        return CartesianPose(
            root_link=self.world.root,
            tip_link=tip.tool_frame,
            goal_pose=self.target.to_spatial_type(),
        )
