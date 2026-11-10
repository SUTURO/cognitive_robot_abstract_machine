"""
Publication modes of a simulated camera.
"""

from __future__ import annotations

# %% imports
from unittest.mock import Mock, call

import pytest
from typing_extensions import TYPE_CHECKING

from robokudo.io import simulated_camera
from robokudo.io.simulated_camera import SimulatedCameraPublisher

if TYPE_CHECKING:
    from rclpy.node import Node


# %% publication modes
@pytest.mark.parametrize("continuous", [True, False])
def test_camera_timer_depends_on_publication_mode(
    node: Node, monkeypatch: pytest.MonkeyPatch, continuous: bool
) -> None:
    """
    Only continuous publication schedules frames on a timer.
    """
    camera_node = Mock()
    monkeypatch.setattr(simulated_camera, "TransformBroadcaster", Mock())
    publisher = SimulatedCameraPublisher(
        camera=Mock(),
        simulation=Mock(),
        node=camera_node,
        topics=Mock(),
        frames=Mock(),
        continuous=continuous,
    )
    assert camera_node.create_timer.call_args_list == (
        [call(1 / publisher.frames_per_second, publisher.publish)] if continuous else []
    )
    publisher.stop()


def test_single_frame_is_published_only_when_requested(
    node: Node, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Single-frame mode captures once when publication is requested.
    """
    camera_node = Mock()
    monkeypatch.setattr(simulated_camera, "TransformBroadcaster", Mock())
    frame = Mock()
    capture = Mock(return_value=frame)
    monkeypatch.setattr(SimulatedCameraPublisher, "capture", capture)
    publisher = SimulatedCameraPublisher(
        camera=Mock(),
        simulation=Mock(),
        node=camera_node,
        topics=Mock(),
        frames=Mock(),
        continuous=False,
    )
    assert capture.call_count == 0
    publisher.publish()
    assert capture.call_count == 1
    publisher.stop()


def test_continuous_camera_publishes_each_timer_tick(
    node: Node, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Each scheduled tick publishes one frame at the configured rate.
    """
    camera_node = Mock()
    monkeypatch.setattr(simulated_camera, "TransformBroadcaster", Mock())
    capture = Mock(return_value=Mock())
    monkeypatch.setattr(SimulatedCameraPublisher, "capture", capture)
    publisher = SimulatedCameraPublisher(
        camera=Mock(),
        simulation=Mock(),
        node=camera_node,
        topics=Mock(),
        frames=Mock(),
        continuous=True,
    )
    period, publish_frame = camera_node.create_timer.call_args.args
    publish_frame()
    publish_frame()
    assert period == 1 / publisher.frames_per_second
    assert capture.call_count == 2
    publisher.stop()
    assert camera_node.destroy_timer.call_args_list == [
        call(camera_node.create_timer.return_value)
    ]
