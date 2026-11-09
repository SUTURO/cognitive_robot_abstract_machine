"""The terminal command fetches the live world and reports one completed scan."""

from __future__ import annotations

from dataclasses import dataclass
from unittest.mock import Mock
import pytest

from coraplex.scene_report_command import SceneReportCommand, RobotSpeech
from coraplex.exceptions import SpeechMessagesUnavailable, SpeechInputUnavailable
from coraplex.scene_perception import SceneObservation, ObservedObject
from semantic_digital_twin.world import World

# %% robot speech messages


@dataclass
class SpeechRequest:
    """Message stand-in for a robot's existing speech input."""

    language: int = 0
    """Language selected by the caller."""
    sentence: str = ""
    """Completed report sent for playback."""


# %% terminal execution


@pytest.mark.parametrize("speak", [False, True])
def test_command_reports_one_scan_and_optionally_speaks(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], speak: bool
) -> None:
    """Printing and robot speech use the same completed report."""
    session = Mock()
    session.fetch_world.return_value = World.create_with_root_body("root")
    context = Mock()
    context.__enter__ = Mock(return_value=session)
    context.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(
        "coraplex.scene_report_command.RobotDemonstrationRosSession.start",
        Mock(return_value=context),
    )
    source = Mock()
    source.observe.return_value = SceneObservation(
        objects=[ObservedObject(category="cup")]
    )
    monkeypatch.setattr(
        "coraplex.scene_report_command.RoboKudoScenePerception",
        Mock(return_value=source),
    )
    monkeypatch.setattr(
        "coraplex.scene_report_command.get_message", Mock(return_value=SpeechRequest)
    )
    monkeypatch.setattr(
        "coraplex.scene_report_command.find_spec", Mock(return_value=object())
    )
    publisher = session.node.create_publisher.return_value
    publisher.get_subscription_count.return_value = 1
    command = SceneReportCommand.from_arguments(["--speak"] if speak else [])
    command.execute()
    assert capsys.readouterr().out.strip() == "I detected a cup."
    assert source.observe.call_count == 1
    if speak:
        publisher.publish.assert_called_once_with(
            SpeechRequest(language=command.language, sentence="I detected a cup.")
        )
    else:
        session.node.create_publisher.assert_not_called()
    context.__exit__.assert_called_once()


def test_help_is_available_without_connecting_to_the_robot(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """Discovering command usage does not start ROS or request perception."""
    start = Mock()
    monkeypatch.setattr(
        "coraplex.scene_report_command.RobotDemonstrationRosSession.start", start
    )
    with pytest.raises(SystemExit) as exit_status:
        SceneReportCommand.from_arguments(["--help"])
    assert exit_status.value.code == 0
    assert "--speak" in capsys.readouterr().out
    start.assert_not_called()


# %% speech readiness


def test_missing_robot_voice_messages_fail_before_perception(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A missing optional robot interface does not start a scan or pretend to speak."""
    monkeypatch.setattr(
        "coraplex.scene_report_command.find_spec", Mock(return_value=None)
    )
    node = Mock()
    with pytest.raises(SpeechMessagesUnavailable):
        RobotSpeech(node=node)
    node.create_publisher.assert_not_called()


def test_absent_speech_subscriber_does_not_receive_a_request(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Speech discovery has a bounded wait rather than silently losing a request."""
    monkeypatch.setattr(
        "coraplex.scene_report_command.find_spec", Mock(return_value=object())
    )
    monkeypatch.setattr(
        "coraplex.scene_report_command.get_message", Mock(return_value=SpeechRequest)
    )
    node = Mock()
    node.create_publisher.return_value.get_subscription_count.return_value = 0
    speech = RobotSpeech(node=node, timeout=0)
    with pytest.raises(SpeechInputUnavailable):
        speech.say("A completed report.")
    node.create_publisher.return_value.publish.assert_not_called()
