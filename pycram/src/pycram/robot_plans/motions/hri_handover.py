from dataclasses import dataclass, field
from typing import Optional, List

from giskardpy.motion_statechart.graph_node import Task
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.world import World
from .base import BaseMotion


@dataclass
class HandoverMotion(BaseMotion):
    world: World = field(kw_only=True, default=None)

    def perform(self):
        return

    @property
    def _motion_chart(self) -> Task:
        prehandover_goal = JointPositionList(
            goal_state=JointState.from_str_dict(
                {
                    "arm_lift_joint": 0.30,  # arm raised a bit
                    "arm_flex_joint": -0.5,  # flex arm forward to roughly horizontal
                    "arm_roll_joint": 0.0,  # neutral roll, arm faces forward
                    "wrist_flex_joint": -1,  # tilt gripper opening slightly upward
                    "wrist_roll_joint": 0.0,  # neutral wrist roll
                },
                world=self.world,
            ),
        )

        return prehandover_goal
