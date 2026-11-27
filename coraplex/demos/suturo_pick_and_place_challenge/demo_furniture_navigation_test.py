"""
Tests for the HSR driving to a piece of furniture in the kitchen environment in MuJoCo.

Run them from the repository's root::

    pytest coraplex/demos/suturo_pick_and_place_challenge/demo_furniture_navigation_test.py
"""

from __future__ import annotations

import numpy as np
import pytest

from giskardpy.motion_statechart.tasks.drive_tasks import DifferentialDriveBaseGoal
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    DiningTable,
)

from demo_furniture_navigation import NavigationDemonstration

pytestmark = pytest.mark.skipif(
    not HSRB.is_description_installed(),
    reason="The description of the HSR is not installed",
)

# %% driving to the dining table


def test_the_hsr_drives_to_where_its_arm_reaches_the_dining_table():
    """
    The HSR stands within its arm's reach of the dining table's centre, which also keeps
    it out of the bedroom behind the low wall beside the table.
    """
    demonstration = NavigationDemonstration(headless=True)

    world = demonstration.run()

    [robot] = world.get_semantic_annotations_by_type(HSRB)
    [arm] = robot.all_arms
    [dining_table] = world.get_semantic_annotations_by_type(DiningTable)
    base_position = robot.root.global_pose.position.to_np()[:2]
    table_position = dining_table.root.global_pose.position.to_np()[:2]
    assert np.linalg.norm(base_position - table_position) <= (
        arm.approximate_length() + DifferentialDriveBaseGoal.threshold
    )
