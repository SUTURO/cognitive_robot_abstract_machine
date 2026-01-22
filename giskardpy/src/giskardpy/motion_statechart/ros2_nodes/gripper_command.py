import logging
from dataclasses import dataclass, field

from giskardpy.motion_statechart.context import BuildContext, ExecutionContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.graph_node import NodeArtifacts
from giskardpy.motion_statechart.ros2_nodes.ros_tasks import ActionServerTask
from typing_extensions import Type, TypeVar
from tmc_control_msgs.action import GripperApplyEffort

logger = logging.getLogger(__name__)
Action = TypeVar("Action")


@dataclass(eq=False, repr=False)
class GripperCommandTask(
    ActionServerTask[
        GripperApplyEffort,
        GripperApplyEffort.Goal,
        GripperApplyEffort.Result,
        GripperApplyEffort.Feedback,
    ]
):

    effort: float = field(kw_only=True)
    message_type: Type[Action] = field(kw_only=True, default=GripperApplyEffort)

    def build_msg(self, context: BuildContext):
        goal_msg = GripperApplyEffort.Goal()
        goal_msg.effort = self.effort
        self._msg = goal_msg

    def build(self, context: BuildContext) -> NodeArtifacts:
        super().build_msg(context)
        artifacts = NodeArtifacts()

        self.action_type = GripperApplyEffort
        self.action_name = self.action_topic

        logger.info(f"Waiting for action server {self.action_topic}")
        self._action_client.wait_for_server()

        return artifacts

    def on_tick(self, context: ExecutionContext) -> ObservationStateValues:
        if self._result:
            return (
                ObservationStateValues.TRUE
                if self._result.error_code == GripperApplyEffort.Result.NONE
                else ObservationStateValues.FALSE
            )
        return ObservationStateValues.UNKNOWN
