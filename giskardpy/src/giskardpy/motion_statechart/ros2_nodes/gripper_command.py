import logging
from dataclasses import dataclass, field

from giskardpy.motion_statechart.context import BuildContext, ExecutionContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.ros2_nodes.ros_tasks import ActionServerTask
from typing_extensions import Type, TypeVar

try:
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

        def on_tick(self, context: ExecutionContext) -> ObservationStateValues:
            if self._result:
                return ObservationStateValues.TRUE
            return ObservationStateValues.UNKNOWN

except ModuleNotFoundError:

    class GripperCommandTask:
        pass
