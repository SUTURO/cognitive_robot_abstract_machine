"""
Tests for the robot saying a sentence out loud.
"""

from __future__ import annotations

from coraplex.robot_plans.actions.core.talking import TalkingAction
from cramspeech.speech_node import SpeechTopic
from giskardpy.motion_statechart.ros2_nodes.topic_monitor import PublishOnStart

# %% handing the sentence to the text-to-speech node


def test_talking_sends_the_sentence_to_the_text_to_speech_node() -> None:
    sentence = "I will bring you the milk"

    (step,) = TalkingAction(speaking_text=sentence).create_steps()

    assert isinstance(step, PublishOnStart)
    assert step.topic_name == SpeechTopic.TEXT_TO_SAY
    assert step.msg.data == sentence
