"""
The HSR turns towards the human in the demonstration.
"""

from __future__ import annotations

# %% imports
import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import numpy
import pytest

from semantic_digital_twin.robots.hsrb import HSRB, HSRBHeadRGBDCamera
from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
from ..pytest_environment import runs_in_continuous_integration

pytestmark = [
    pytest.mark.skipif(
        not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
    ),
    pytest.mark.skipif(
        not HSRB.is_description_installed(),
        reason="The description of the HSR is not installed",
    ),
]


# %% demonstration
@pytest.fixture(scope="module")
def human_demo() -> ModuleType:
    """
    Load the human detection demonstration from its directory.
    """
    folder = (
        Path(__file__).parents[2]
        / "coraplex"
        / "demos"
        / "coraplex_hsr_human_detection_demo"
    )
    with pytest.MonkeyPatch.context() as patch:
        patch.syspath_prepend(str(folder))
        specification = importlib.util.spec_from_file_location(
            "hsr_human_detection_demo", folder / "demo.py"
        )
        module = importlib.util.module_from_spec(specification)
        patch.setitem(sys.modules, specification.name, module)
        specification.loader.exec_module(module)
    return module


def test_robot_turns_towards_the_human_on_its_left(human_demo: ModuleType) -> None:
    """
    The robot faces the human and carries its head camera with it.
    """
    demonstration = human_demo.HumanDetectionDemonstration(headless=True)
    world = demonstration.run()
    [human] = world.get_semantic_annotations_by_type(Human)
    [camera] = world.get_semantic_annotations_by_type(HSRBHeadRGBDCamera)
    [robot] = world.get_semantic_annotations_by_type(HSRB)
    robot_to_human = world.compute_forward_kinematics_np(robot.root, human.root)[:3, 3]
    human_position = human.root.global_transform.to_np()[:3, 3]
    assert human_position[:2] == pytest.approx(
        [0.0, demonstration.human_distance], abs=0.05
    )
    assert robot_to_human[0] == pytest.approx(demonstration.human_distance, abs=0.1)
    assert robot_to_human[1] == pytest.approx(0.0, abs=0.1)
    world_to_camera = camera.root.global_transform.to_np()
    direction = human_position[:2] - world_to_camera[:2, 3]
    direction /= numpy.linalg.norm(direction)
    viewing_direction = world_to_camera[:2, 2]
    viewing_direction /= numpy.linalg.norm(viewing_direction)
    assert numpy.dot(viewing_direction, direction) == pytest.approx(1.0, abs=0.02)
