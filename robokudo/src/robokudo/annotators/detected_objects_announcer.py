"""Says the detected objects as one English sentence through the robot's voice."""

# %% imports
from __future__ import annotations

from collections import Counter
from enum import StrEnum
from importlib.util import find_spec
from typing import Any

from py_trees.common import Status
from rclpy.publisher import Publisher
from rosidl_runtime_py.utilities import get_message

from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.exceptions import RobotVoiceMessagesUnavailable
from robokudo.types.annotation import Classification
from robokudo.types.scene import ObjectHypothesis
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry


# %% robot voice
class RobotVoiceInput(StrEnum):
    """The input of the HSR's running speech engine."""

    TOPIC = "/talk_request"
    """Topic the speech engine reads the sentences to say from."""
    MESSAGE = "tmc_voice_msgs/msg/Voice"
    """Message type of the sentences, provided by the robot's ROS workspace."""


# %% announcer
class DetectedObjectsAnnouncer(BaseAnnotator):
    """Has the robot say a sentence such as "I see a cup and 2 bottles." naming
    the classified object hypotheses in the CAS."""

    class Descriptor(BaseAnnotator.Descriptor):
        """Configuration of the :class:`DetectedObjectsAnnouncer`."""

        class Parameters:
            """Parameters of the :class:`DetectedObjectsAnnouncer`."""

            def __init__(self) -> None:
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
        self.voice_message_type: Any = None
        """Message class of the robot's speech engine, loaded in :meth:`setup`."""

    def setup(self, **kwargs: Any) -> bool:
        """Create the publisher to the robot's speech engine on the shared
        RoboKudo node, see :meth:`BaseAnnotator.setup`.

        :raises RobotVoiceMessagesUnavailable: If the robot's ROS workspace
            is not sourced.
        """
        if find_spec(RobotVoiceInput.MESSAGE.split("/")[0]) is None:
            raise RobotVoiceMessagesUnavailable(message_type=RobotVoiceInput.MESSAGE)
        self.voice_message_type = get_message(RobotVoiceInput.MESSAGE)
        self.publisher = (
            ROSNodeRegistry()
            .get()
            .create_publisher(
                self.voice_message_type,
                RobotVoiceInput.TOPIC,
                self.descriptor.parameters.queue_size,
            )
        )
        return True

    def update(self) -> Status:
        """Have the robot say the sentence describing the detected objects in
        English.

        :return: SUCCESS after publishing.
        """
        sentence = self.describe_detected_objects()
        message = self.voice_message_type()
        message.language = self.voice_message_type.ENGLISH
        message.sentence = sentence
        self.publisher.publish(message)
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
