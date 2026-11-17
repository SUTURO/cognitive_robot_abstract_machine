"""
Tests for the Tiago picking a T-shirt up off a table and swinging it in MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy
import pytest

from experiments.t_shirt_demo.demo import SceneEntity, TShirtSwingDemonstration
from semantic_digital_twin.robots.tiago import TiagoMujoco
from semantic_digital_twin.semantic_annotations.semantic_annotations import TShirt

from ...pytest_environment import runs_in_continuous_integration

pytestmark = pytest.mark.skipif(
    not TiagoMujoco.is_description_installed(),
    reason="The MuJoCo model of the Tiago is not installed",
)

# %% the scene


def test_the_t_shirt_lies_spread_out_on_the_table() -> None:
    """
    The T-shirt starts above the table's top and within its edges.
    """
    demonstration = TShirtSwingDemonstration(headless=True)
    world = demonstration.build_simulated_world()

    [t_shirt] = world.get_semantic_annotations_by_type(TShirt)
    vertices = t_shirt.cloth_sheet.starting_surface.vertices

    table = demonstration.table_scale
    assert vertices[:, 0].min() >= demonstration.table_distance
    assert vertices[:, 0].max() <= demonstration.table_distance + table.x
    assert numpy.abs(vertices[:, 1]).max() <= table.y / 2
    assert vertices[:, 2].min() > table.z


# %% swinging it


@dataclass
class TShirtSwingKeepingWhereTheTShirtEnded(TShirtSwingDemonstration):
    """
    The demonstration, keeping where the vertices of the T-shirt are when it ends.
    """

    t_shirt_at_the_end: numpy.ndarray = field(init=False, default=None)
    """
    Where each vertex of the T-shirt is when the demonstration ends.
    """

    def tear_down(self) -> None:
        """
        :meth:`TShirtSwingDemonstration.tear_down`, after keeping where the T-shirt is.
        """
        if self.simulation is not None:
            self.t_shirt_at_the_end = (
                self.simulation.simulator.get_flex_vertex_positions(
                    flex_name=SceneEntity.T_SHIRT
                ).result
            )
        super().tear_down()


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_the_t_shirt_ends_hanging_from_the_gripper_clear_of_the_table() -> None:
    """
    After the swing, the T-shirt hangs from the gripper at the height it is held at,
    with no part of it left on the table and none of it on the floor.
    """
    demonstration = TShirtSwingKeepingWhereTheTShirtEnded(headless=True)

    demonstration.run()

    vertices = demonstration.t_shirt_at_the_end
    over_the_table = vertices[:, 0] > demonstration.table_distance
    assert not numpy.any(over_the_table)
    assert vertices[:, 2].min() > 0.0
    assert vertices[:, 2].max() == pytest.approx(demonstration.hanging_height, abs=0.1)
