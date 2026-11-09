"""ROS adapter for completed RoboKudo scene queries; requires its message packages."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from threading import Event

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from robokudo_msgs.action import Query
from robokudo_msgs.msg import ObjectDesignator
from typing_extensions import Any, TYPE_CHECKING, Callable

from coraplex.exceptions import PerceptionSourceUnavailable, ScenePerceptionFailed
from coraplex.perception import ROBOKUDO_QUERY_ACTION_NAME
from coraplex.scene_perception import ObservedObject, SceneObservation, ScenePerception
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Scale

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.task import Future
    from coraplex.scene_perception import SceneQuery

# %% action outcomes


class SceneFailure(StrEnum):
    """Reasons an action result cannot be used as a completed observation."""

    TIMEOUT = "request timed out"
    """An action future did not finish within its configured budget."""
    REJECTED = "request rejected"
    """The action server declined the goal."""
    UNSUCCESSFUL = "request aborted or canceled"
    """The action ended without success."""


# %% scene source


@dataclass
class RoboKudoScenePerception(ScenePerception):
    """Read all hypotheses from one successful query; the ROS node must spin externally."""

    ros_node: Node
    """Externally serviced ROS node providing the action client."""
    action_name: str = ROBOKUDO_QUERY_ACTION_NAME
    """Existing RoboKudo query endpoint."""
    server_timeout: float = 10.0
    """Seconds allowed for action-server discovery."""
    response_timeout: float = 30.0
    """Seconds allowed separately for goal acceptance and the completed result."""

    def observe(self, query: SceneQuery) -> SceneObservation:
        client = ActionClient(self.ros_node, Query, self.action_name)
        try:
            if not client.wait_for_server(timeout_sec=self.server_timeout):
                raise PerceptionSourceUnavailable(self.action_name)
            handle = self.wait_result(
                client.send_goal_async(Query.Goal(obj=ObjectDesignator()))
            )
            if not handle.accepted:
                raise ScenePerceptionFailed(SceneFailure.REJECTED)
            result = self.wait_result(
                handle.get_result_async(), handle.cancel_goal_async
            )
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise ScenePerceptionFailed(SceneFailure.UNSUCCESSFUL)
            return SceneObservation(
                objects=[self.parse_object(item, query) for item in result.result.res]
            )
        finally:
            client.destroy()

    def wait_result(
        self, future: Future, cancel: Callable[[], Future] | None = None
    ) -> Any:
        """Wait for an action future, requesting cancellation when its deadline expires."""
        ready = Event()
        future.add_done_callback(lambda completed: ready.set())
        if not ready.wait(self.response_timeout):
            if cancel is not None:
                cancel()
            raise ScenePerceptionFailed(SceneFailure.TIMEOUT)
        return future.result()

    @staticmethod
    def parse_object(designator: ObjectDesignator, query: SceneQuery) -> ObservedObject:
        """Preserve unknown objects even when their localization or dimensions are absent."""
        pose = None
        if designator.pose:
            stamped = designator.pose[0]
            position = stamped.pose.position
            orientation = stamped.pose.orientation
            pose = Pose.from_xyz_quaternion(
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
                reference_frame=query.world.get_kinematic_structure_entity_by_name(
                    stamped.header.frame_id
                ),
            )
        dimensions = None
        if designator.shape_size:
            size = designator.shape_size[0].dimensions
            if min(size.x, size.y, size.z) > 0:
                dimensions = Scale(size.x, size.y, size.z)
        return ObservedObject(
            category=designator.type,
            identity=designator.uid or None,
            pose=pose,
            dimensions=dimensions,
        )
