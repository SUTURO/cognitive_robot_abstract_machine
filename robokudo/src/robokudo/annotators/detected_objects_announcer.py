"""Announces the detected objects as one English sentence for the robot's text-
to-speech."""

# %% imports
from __future__ import annotations

from collections import Counter
from typing import Any

from py_trees.common import Status
from rclpy.publisher import Publisher
from std_msgs.msg import String

from robokudo.annotators.core import BaseAnnotator
from robokudo.types.annotation import Classification
from robokudo.types.scene import ObjectHypothesis
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry


# %% announcer
class DetectedObjectsAnnouncer(BaseAnnotator):
    """Publishes a sentence such as "I see a cup and 2 bottles." naming the
    classified object hypotheses in the CAS."""

    class Descriptor(BaseAnnotator.Descriptor):
        """Configuration of the :class:`DetectedObjectsAnnouncer`."""

        class Parameters:
            """Parameters of the :class:`DetectedObjectsAnnouncer`."""

            def __init__(self) -> None:
                self.topic_name: str = "/tts_text"
                """Topic the text-to-speech node reads the sentences to say
                from."""
                self.queue_size: int = 10
                """Depth of the publisher's message queue."""
        parameters = Parameters()

    def __init__(
        self,
        name: str = "DetectedObjectsAnnouncer",
        descriptor: DetectedObjectsAnnouncer.Descriptor | None = None,
    ) -> None:
        super().__init__(name, descriptor)
        self.publisher: Publisher | None = None
        """Publisher of the sentences, created in :meth:`setup`."""

    def setup(self, **kwargs: Any) -> bool:
        """Create the publisher on the shared RoboKudo node, see
        :meth:`BaseAnnotator.setup`."""
        self.publisher = (
            ROSNodeRegistry()
            .get()
            .create_publisher(
                String,
                self.descriptor.parameters.topic_name,
                self.descriptor.parameters.queue_size,
            )
        )
        return True

    def update(self) -> Status:
        """Publish the sentence describing the detected objects.

        :return: SUCCESS after publishing.
        """
        sentence = self.describe_detected_objects()
        self.publisher.publish(String(data=sentence))
        self.feedback_message = sentence
        return Status.SUCCESS

    def describe_detected_objects(self) -> str:
        """
        :return: A sentence naming every detected class with its count, in the order
            the classes were first detected.
        """
        class_counts = Counter(
            annotation.classname.replace("_", " ")
            for hypothesis in self.get_cas().filter_annotations_by_type(
                ObjectHypothesis
            )
            for annotation in hypothesis.annotations
            if isinstance(annotation, Classification)
        )
        if not class_counts:
            return "I do not see any objects."

        phrases = [
            self._count_phrase(classname, count)
            for classname, count in class_counts.items()
        ]
        if len(phrases) == 1:
            return f"I see {phrases[0]}."
        return f"I see {', '.join(phrases[:-1])} and {phrases[-1]}."

    def _count_phrase(self, classname: str, count: int) -> str:
        """
        :return: The class name with its English article or count, such as "an apple"
            or "2 bottles".
        """
        if count > 1:
            return f"{count} {classname}s"
        article = "an" if classname[0].lower() in "aeiou" else "a"
        return f"{article} {classname}"
