# %% imports
import pytest
import rclpy
from py_trees.common import Status

import robokudo.annotators.detected_objects_announcer
from robokudo.annotators.detected_objects_announcer import (
    DetectedObjectsAnnouncer,
    RobotVoiceInput,
)
from robokudo.annotators.exceptions import RobotVoiceMessagesUnavailable
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import Classification
from robokudo.types.scene import ObjectHypothesis


# %% helpers
def _announcer_in_pipeline() -> DetectedObjectsAnnouncer:
    pipeline = Pipeline("Sequence")
    announcer = DetectedObjectsAnnouncer()
    pipeline.add_child(announcer)
    return announcer


def _hypothesis(classname: str) -> ObjectHypothesis:
    hypothesis = ObjectHypothesis()
    hypothesis.annotations.append(Classification(classname=classname))
    return hypothesis


# %% sentence
def test_sentence_says_nothing_was_seen_without_hypotheses():
    announcer = _announcer_in_pipeline()

    assert announcer.describe_detected_objects() == "I do not see any objects."


def test_sentence_counts_objects_in_order_of_detection():
    announcer = _announcer_in_pipeline()
    announcer.get_cas().annotations = [
        _hypothesis("cup"),
        _hypothesis("bottle"),
        _hypothesis("apple"),
        _hypothesis("bottle"),
    ]

    assert (
        announcer.describe_detected_objects() == "I see a cup, 2 bottles and an apple."
    )


def test_sentence_speaks_class_names_with_spaces():
    announcer = _announcer_in_pipeline()
    announcer.get_cas().annotations = [_hypothesis("spoon_dinner_redgrip")]

    assert announcer.describe_detected_objects() == "I see a spoon dinner redgrip."


def test_sentence_ignores_annotations_outside_object_hypotheses():
    announcer = _announcer_in_pipeline()
    announcer.get_cas().annotations = [Classification(classname="cup")]

    assert announcer.describe_detected_objects() == "I do not see any objects."


# %% speaking
def test_update_says_sentence_in_english_through_robot_voice(node):
    voice_messages = pytest.importorskip("tmc_voice_msgs.msg")
    announcer = _announcer_in_pipeline()
    announcer.get_cas().annotations = [_hypothesis("cup")]
    received = []
    node.create_subscription(
        voice_messages.Voice,
        RobotVoiceInput.TOPIC,
        received.append,
        10,
    )
    announcer.setup(node=node, timeout=1.0)

    status = announcer.update()
    for _ in range(20):
        if received:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    assert status == Status.SUCCESS
    assert [(message.sentence, message.language) for message in received] == [
        (announcer.describe_detected_objects(), voice_messages.Voice.ENGLISH)
    ]


def test_setup_raises_without_robot_voice_messages(node, monkeypatch):
    monkeypatch.setattr(
        robokudo.annotators.detected_objects_announcer,
        "find_spec",
        lambda package_name: None,
    )
    announcer = _announcer_in_pipeline()

    with pytest.raises(RobotVoiceMessagesUnavailable):
        announcer.setup(node=node, timeout=1.0)
