from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

try:
    from nav2_msgs.action import NavigateToPose
except ModuleNotFoundError:
    NavigateToPose = None
from rclpy.action import ActionClient
from std_msgs.msg import Header
from typing_extensions import Type, TypeVar, Generic

import krrood.symbolic_math.symbolic_math as sm
from giskardpy.motion_statechart.context import ExecutionContext, BuildContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.graph_node import (
    MotionStatechartNode,
    NodeArtifacts,
)
from giskardpy.motion_statechart.ros_context import RosContextExtension
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


Action = TypeVar("Action")
ActionGoal = TypeVar("ActionGoal")
ActionResult = TypeVar("ActionResult")
ActionFeedback = TypeVar("ActionFeedback")


@dataclass(eq=False, repr=False)
class ActionServerTask(
    MotionStatechartNode,
    ABC,
    Generic[Action, ActionGoal, ActionResult, ActionFeedback],
):
    """
    Abstract base class for tasks that call a ROS2 action server.
    """

    action_topic: str
    """
    Topic name for the action server.
    """

    message_type: Type[Action]
    """
    Fully specified goal message that can be send out. 
    """

    _action_client: ActionClient = field(init=False)
    """
    ROS action client, is created in `build`.
    """

    _msg: ActionGoal = field(init=False, default=None)
    """
    ROS message to send to the action server.
    """

    _result: ActionResult = field(init=False, default=None)
    """
    ROS action server result.
    """

    @abstractmethod
    def build_msg(self, context: BuildContext):
        """
        Build the action server message and returns it.
        """
        ...

    def build(self, context: BuildContext) -> NodeArtifacts:
        """
        Creates the action client.
        """
        ros_context_extension = context.require_extension(RosContextExtension)
        self._action_client = ActionClient(
            ros_context_extension.ros_node, self.message_type, self.action_topic
        )
        self.build_msg(context)
        logger.info(f"Waiting for action server {self.action_topic}")
        self._action_client.wait_for_server()
        return NodeArtifacts()

    def on_start(self, context: ExecutionContext):
        """
        Creates a goal and sends it to the action server asynchronously.
        """
        future = self._action_client.send_goal_async(self._msg)
        future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self._result = future.result()
        logger.info(f"Action server {self.action_topic} done.")


@dataclass(eq=False, repr=False)
class NavigateActionServerTask(
    ActionServerTask[
        NavigateToPose,
        NavigateToPose.Goal,
        NavigateToPose.Result,
        NavigateToPose.Feedback,
    ]
):
    """
    Node for calling a Navigation2 ROS2 action server to navigate to a given pose.1
    """

    target_pose: HomogeneousTransformationMatrix
    """
    Target pose to which the robot should navigate.
    """

    base_link: Body
    """
    Base link of the robot, used for estimating the distance to the goal
    """

    action_topic: str
    """
    Topic name for the navigation action server.
    """

    def build_msg(self, context: BuildContext):
        root_p_goal = context.world.transform(
            target_frame=context.world.root, spatial_object=self.target_pose
        )
        position = root_p_goal.to_position().to_np()
        orientation = root_p_goal.to_quaternion().to_np()
        pose_stamped = PoseStamped(
            header=Header(frame_id="map"),
            pose=Pose(
                position=Point(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(
                    x=orientation[0],
                    y=orientation[1],
                    z=orientation[2],
                    w=orientation[3],
                ),
            ),
        )
        self._msg = NavigateToPose.Goal(pose=pose_stamped)

    def build(self, context: BuildContext) -> NodeArtifacts:
        """
        Builds the motion state node this includes creating the action client and setting the observation expression.
        The observation is true if the robot is within 1cm of the target pose.
        """
        super().build_msg(context)
        artifacts = NodeArtifacts()
        root_T_goal = context.world.transform(
            target_frame=context.world.root, spatial_object=self.target_pose
        )
        root_T_current = context.world.compose_forward_kinematics_expression(
            context.world.root, self.base_link
        )

        position_error = root_T_goal.to_position().euclidean_distance(
            root_T_current.to_position()
        )
        rotation_error = root_T_goal.to_rotation_matrix().rotational_error(
            root_T_current.to_rotation_matrix()
        )

        artifacts.observation = sm.trinary_logic_and(
            position_error < 0.01, sm.abs(rotation_error) < 0.01
        )

        logger.info(f"Waiting for action server {self.action_topic}")
        self._action_client.wait_for_server()

        return artifacts

    def on_tick(self, context: ExecutionContext) -> ObservationStateValues:
        if self._result:
            return (
                ObservationStateValues.TRUE
                if self._result.error_code == NavigateToPose.Result.NONE
                else ObservationStateValues.FALSE
            )
        return ObservationStateValues.UNKNOWN


# @dataclass
# class GripperActionServerTask(ActionServerTask[
#         NavigateToPose,
#         NavigateToPose.Goal,
#         NavigateToPose.Result,
#         NavigateToPose.Feedback,
#     ]
# ):
#     """
#     Task für die Grippersteuerung über einen Action Server, um den Effort anzuwenden (öffnen/schließen des Grippers).
#     """
#
#     gripper_effort: float
#     """
#     Der Effort-Wert für den Gripper. Positive Werte öffnen den Gripper, negative schließen ihn.
#     """
#
#     action_topic: str
#     """
#     Der Topicname für den Gripper Action Server.
#     """
#
#     _action_client: ActionClient = None
#     _send_goal_future = None
#     _msg = None
#
#     def build_msg(self):
#         """
#         Baut die Gripper Action Goal Nachricht mit dem gewünschten Effort.
#         """
#         msg = GripperApplyEffort.Goal()
#         msg.effort = self.gripper_effort  # Setze den Effort (positiv für Öffnen, negativ für Schließen)
#         self._msg = msg
#
#     def build(self, node):
#         """
#         Baut den Action Client und sendet das Ziel (Goal).
#         """
#         self._action_client = ActionClient(node, GripperApplyEffort, self.action_topic)
#
#         # Warten, bis der Action Server bereit ist
#         node.get_logger().info(f"Waiting for action server {self.action_topic}")
#         self._action_client.wait_for_server(timeout_sec=5.0)
#
#         # Goal-Nachricht für den Gripper
#         goal_msg = GripperApplyEffort.Goal()
#         goal_msg.effort = self.gripper_effort
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)
#
#     def goal_response_callback(self, future):
#         """
#         Callback, wenn der Action Server auf das Ziel reagiert.
#         """
#         result = future.result()
#         if result.accepted:
#             print('Goal accepted by the action server.')
#         else:
#             print('Goal rejected by the action server.')
#
#     def feedback_callback(self, feedback):
#         """
#         Callback, um Feedback vom Action Server zu erhalten.
#         """
#         print(f"Feedback received: {feedback.feedback}")
#
#     def on_tick(self):
#         """
#         Überprüft, ob das Ziel erfolgreich abgeschlossen wurde.
#         """
#         if self._send_goal_future.done():
#             result = self._send_goal_future.result()
#             if result:
#                 if result.status == 3:  # Erfolgreiche Ausführung
#                     return True
#                 else:
#                     return False
#         return None
