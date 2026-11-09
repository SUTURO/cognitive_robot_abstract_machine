"""Run one live scene query from a terminal and optionally send it to robot speech."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import StrEnum
from importlib.util import find_spec
from time import monotonic, sleep

from rclpy.duration import Duration
from rosidl_runtime_py.utilities import get_message
from typing_extensions import TYPE_CHECKING, Any

from coraplex.demonstrations import RobotDemonstrationRosSession
from coraplex.exceptions import SpeechInputUnavailable, SpeechMessagesUnavailable
from coraplex.perception import ROBOKUDO_QUERY_ACTION_NAME
from coraplex.ros_scene_perception import RoboKudoScenePerception
from coraplex.scene_perception import SceneQuery

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.publisher import Publisher

# %% terminal interfaces


class SceneCommandOption(StrEnum):
    """Supported terminal switches."""

    SPEAK = "--speak"
    """Send the completed report to the robot as well as printing it."""
    ACTION = "--action"
    """Select the scene-perception action endpoint."""
    SPEECH_TOPIC = "--speech-topic"
    """Select the existing robot speech input."""
    LANGUAGE = "--language"
    """Select the robot voice-message language value."""
    TIMEOUT = "--timeout"
    """Set the discovery, world-fetch and response timeout in seconds."""


class RobotVoiceInput(StrEnum):
    """The HSR's documented existing speech interface."""

    TOPIC = "/talk_request"
    """Topic consumed by the running robot speech engine."""
    MESSAGE = "tmc_voice_msgs/msg/Voice"
    """Message supplied by the robot ROS workspace, loaded only for speech."""


# %% robot speech


@dataclass
class RobotSpeech:
    """Send one finished sentence to an existing voice node; does not synthesize audio."""

    node: Node
    """Externally spun node used to discover the speech subscriber."""
    topic: str = RobotVoiceInput.TOPIC
    """Existing voice-input topic."""
    language: int = 1
    """English language value documented for the HSR."""
    timeout: float = 30.0
    """Seconds allowed for discovery and transport acknowledgement separately."""
    queue_depth: int = 10
    """Maximum number of outgoing requests retained for the subscriber."""
    polling_period: float = 0.05
    """Seconds between subscriber-discovery checks while the executor spins."""
    publisher: Publisher = field(init=False)
    """Publisher retaining the loaded robot message type."""
    message_type: Any = field(init=False)
    """ROS-generated message class already installed in the robot environment."""

    def __post_init__(self) -> None:
        package = RobotVoiceInput.MESSAGE.split("/")[0]
        if find_spec(package) is None:
            raise SpeechMessagesUnavailable(RobotVoiceInput.MESSAGE)
        self.message_type = get_message(RobotVoiceInput.MESSAGE)
        self.publisher = self.node.create_publisher(
            self.message_type, self.topic, self.queue_depth
        )

    def say(self, sentence: str) -> None:
        """Publish once after discovery and await transport, without asserting playback completion."""
        deadline = monotonic() + self.timeout
        while self.publisher.get_subscription_count() == 0:
            if monotonic() >= deadline:
                raise SpeechInputUnavailable(self.topic)
            sleep(self.polling_period)
        message = self.message_type()
        message.language = self.language
        message.sentence = sentence
        self.publisher.publish(message)
        if not self.publisher.wait_for_all_acked(Duration(seconds=self.timeout)):
            raise SpeechInputUnavailable(self.topic)


# %% one-shot command


@dataclass
class SceneReportCommand:
    """Fetch the live world, request one completed scan, and print or speak its report."""

    speak: bool = False
    """Whether to send the report directly to the existing robot voice node."""
    action_name: str = ROBOKUDO_QUERY_ACTION_NAME
    """Action served by the robot's detection pipeline."""
    speech_topic: str = RobotVoiceInput.TOPIC
    """Robot speech input, relevant only when speech is requested."""
    language: int = 1
    """Voice-message language; the documented English value is the default."""
    timeout: float = 30.0
    """Seconds allowed for each external discovery or response step."""
    node_name: str = "scene_reporter"
    """Name of the temporary ROS node, stopped when this command finishes."""

    @classmethod
    def from_arguments(cls, arguments: list[str] | None = None) -> SceneReportCommand:
        """Parse terminal options without connecting to ROS or importing voice messages."""
        defaults = cls()
        parser = argparse.ArgumentParser(
            description="Fetch the robot world and describe one completed RoboKudo scan. Requires a running fetch_world service and detection pipeline. Add --speak for the existing HSR voice node."
        )
        parser.add_argument(
            SceneCommandOption.SPEAK,
            action="store_true",
            help="also send the report directly to the HSR voice node",
        )
        parser.add_argument(SceneCommandOption.ACTION, default=defaults.action_name)
        parser.add_argument(
            SceneCommandOption.SPEECH_TOPIC, default=defaults.speech_topic
        )
        parser.add_argument(
            SceneCommandOption.LANGUAGE, type=int, default=defaults.language
        )
        parser.add_argument(
            SceneCommandOption.TIMEOUT, type=float, default=defaults.timeout
        )
        options = parser.parse_args(arguments)
        if options.timeout <= 0:
            parser.error("--timeout must be greater than zero")
        return cls(
            speak=options.speak,
            action_name=options.action,
            speech_topic=options.speech_topic,
            language=options.language,
            timeout=options.timeout,
        )

    def execute(self) -> None:
        """Perform one stationary observation and release all owned ROS resources."""
        with RobotDemonstrationRosSession.start(self.node_name) as session:
            speech = (
                RobotSpeech(
                    node=session.node,
                    topic=self.speech_topic,
                    language=self.language,
                    timeout=self.timeout,
                )
                if self.speak
                else None
            )
            session.node.get_logger().info(
                "Fetching the live world from a running fetch_world service."
            )
            world = session.fetch_world(timeout_seconds=self.timeout)
            source = RoboKudoScenePerception(
                ros_node=session.node,
                action_name=self.action_name,
                server_timeout=self.timeout,
                response_timeout=self.timeout,
            )
            batch = source.observe(SceneQuery(world=world))
            sentence = batch.describe(SceneQuery(world=world))
            print(sentence, flush=True)
            if speech is not None:
                speech.say(sentence)

    @classmethod
    def run(cls) -> None:
        """Run the terminal entry point using the invoking process's arguments."""
        command = cls.from_arguments()
        command.execute()


if __name__ == "__main__":
    SceneReportCommand.run()
