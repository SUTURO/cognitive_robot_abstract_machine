"""
Tests for the HSR driving through the rooms of the kitchen environment in MuJoCo.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

from giskardpy.motion_statechart.tasks.drive_tasks import DifferentialDriveBaseGoal
from semantic_digital_twin.robots.hsrb import HSRB

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

# %% the demo, loaded from its folder


@pytest.fixture(scope="module")
def kitchen_navigation_demo() -> ModuleType:
    """
    The ``demo_in_kitchen`` module, its folder put on the path for the modules it
    imports.
    """
    folder = (
        Path(__file__).parents[2] / "coraplex" / "demos" / "coraplex_hsr_mujoco_demo"
    )
    with pytest.MonkeyPatch.context() as patch:
        patch.syspath_prepend(str(folder))
        specification = importlib.util.spec_from_file_location(
            "hsr_kitchen_navigation_demo", folder / "demo_in_kitchen.py"
        )
        module = importlib.util.module_from_spec(specification)
        patch.setitem(sys.modules, specification.name, module)
        specification.loader.exec_module(module)
    return module


# %% driving from room to room


def test_the_hsr_drives_from_the_kitchen_through_the_office_into_the_bedroom(
    kitchen_navigation_demo,
):
    """
    The kitchen, office and bedroom each have a floor of their own, so the HSR only
    arrives when its path passes from floor to floor.
    """
    demonstration = kitchen_navigation_demo.KitchenNavigationDemonstration(
        headless=True
    )

    world = demonstration.run()

    [robot] = world.get_semantic_annotations_by_type(HSRB)
    base_pose = robot.root.global_pose
    assert (
        np.hypot(
            float(base_pose.x) - demonstration.second_goal_x,
            float(base_pose.y) - demonstration.second_goal_y,
        )
        <= DifferentialDriveBaseGoal.threshold
    )
