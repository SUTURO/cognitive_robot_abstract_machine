# %% imports
import rclpy
from py_trees.common import Status
from std_msgs.msg import String

from robokudo.annotators.detected_objects_announcer import DetectedObjectsAnnouncer
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


# %% publishing
def test_update_publishes_sentence_on_topic(node):
    announcer = _announcer_in_pipeline()
    announcer.get_cas().annotations = [_hypothesis("cup")]
    received: list[str] = []
    node.create_subscription(
        String,
        announcer.descriptor.parameters.topic_name,
        lambda message: received.append(message.data),
        10,
    )
    announcer.setup(node=node, timeout=1.0)

    status = announcer.update()
    for _ in range(20):
        if received:
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    assert status == Status.SUCCESS
    assert received == [announcer.describe_detected_objects()]
