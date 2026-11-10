"""
Publication of a simulated camera frame.
"""

from __future__ import annotations

# %% imports
from unittest.mock import Mock

import pytest
from typing_extensions import TYPE_CHECKING

from robokudo.io import simulated_camera
from robokudo.io.simulated_camera import SimulatedCameraPublisher

if TYPE_CHECKING:
    from rclpy.node import Node


# %% publication
def test_single_frame_is_published_only_when_requested(
    node: Node, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Capture one frame when publication is requested.
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
    )
    assert capture.call_count == 0
    publisher.publish()
    assert capture.call_count == 1
