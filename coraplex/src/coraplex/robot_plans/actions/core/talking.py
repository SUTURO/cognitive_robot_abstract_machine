"""
Actions that let the robot speak.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass

from std_msgs.msg import String

from coraplex.datastructures.enums import SpeechTopic
from coraplex.robot_plans.actions.base import SequentialAction
from cramph.node import StatechartNode
from giskardpy.motion_statechart.ros2_nodes.topics import PublishOnStart

# %% talking


@dataclass(eq=False, repr=False, kw_only=True)
class TalkingAction(SequentialAction):
    """
    The robot says a sentence out loud through its text-to-speech node.

    The action succeeds once the sentence is handed over, not once it has been spoken.
    """

    speaking_text: str
    """
    The sentence the robot says, e.g. ``"I will bring you the milk"``.
    """

    def create_steps(self) -> list[StatechartNode]:
        return [
            PublishOnStart(
                topic_name=SpeechTopic.TEXT_TO_SAY,
                msg=String(data=self.speaking_text),
            )
        ]
