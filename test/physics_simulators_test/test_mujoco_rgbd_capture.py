"""
Consistent RGB and depth capture in MuJoCo.
"""

from __future__ import annotations

# %% imports
from threading import RLock
from unittest.mock import MagicMock, call

import numpy
import pytest

from physics_simulators import mujoco_simulator
from physics_simulators.mujoco_simulator import MujocoSimulator


# %% capture
def test_color_and_depth_use_one_scene(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    Render both images from the same scene without refreshing it between them.
    """
    simulator = object.__new__(MujocoSimulator)
    simulator._model_lock = RLock()
    simulator._mj_model = MagicMock()
    simulator._mj_data = MagicMock()
    simulator._mj_data.camera.return_value.xpos = numpy.array([1.0, 2.0, 3.0])
    simulator._mj_data.camera.return_value.xmat = numpy.eye(3).reshape(-1)
    renderer = MagicMock()
    renderer.__enter__.return_value = renderer
    color = numpy.zeros((2, 3, 3), dtype=numpy.uint8)
    depth = numpy.ones((2, 3), dtype=numpy.float32)
    renderer.render.side_effect = [color, depth]
    monkeypatch.setattr(
        mujoco_simulator.mujoco, "Renderer", MagicMock(return_value=renderer)
    )

    frame = simulator.capture_rgbd("camera", 2, 3).result

    assert frame.color is color
    assert frame.depth is depth
    numpy.testing.assert_array_equal(
        frame.camera_position, simulator._mj_data.camera.return_value.xpos
    )
    numpy.testing.assert_array_equal(frame.camera_quaternion, [0.0, 1.0, 0.0, 0.0])
    assert renderer.update_scene.call_args_list == [call(simulator._mj_data, "camera")]
