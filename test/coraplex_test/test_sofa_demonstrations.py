"""
Tests for the demonstrations that drive the HSR to the sofa.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

import pytest
from typing_extensions import Callable

from coraplex import sofa_demonstrations
from coraplex.robot_plans.actions.core.navigation import (
    PathPlanningNavigateAcrossFloorsAction,
)
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from coraplex.sofa_demonstrations import SofaScanningDemonstration
from cramph.context import StatechartContext
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor
from semantic_digital_twin.world import World

pytestmark = pytest.mark.skipif(
    not HSRB.is_description_installed(),
    reason="The description of the HSR is not installed",
)

# %% a demonstration that records what it would run


class DrivenBy(StrEnum):
    """
    What drives the robot apart from the plan.
    """

    PUMAS = "pumas"
    """
    The Pumas navigation.
    """


@dataclass(kw_only=True)
class RecordingScanningDemonstration(SofaScanningDemonstration):
    """
    Records the kinds of nodes of every run instead of running them.
    """

    performed: list[list[type[StatechartNode]] | DrivenBy] = field(default_factory=list)
    """
    The kinds of nodes of each run, and the drives between them, in order.
    """

    def scanning_nodes(self, *, context: StatechartContext) -> list[StatechartNode]:
        return [MoveTorsoAction(torso_state=TorsoState.LOW)]

    def execute_statechart(
        self,
        world: World,
        build_statechart: Callable[[StatechartContext], Statechart],
    ) -> None:
        """
        Record the kinds of nodes in the sequence of the statechart.
        """
        context = StatechartContext.with_extensions(
            world=world, extensions=self.build_context_extensions(world)
        )
        [sequence] = build_statechart(context).top_level_nodes
        self.performed.append([type(node) for node in sequence.nodes])


# %% who drives


def test_plan_drives_to_the_sofa():
    demonstration = RecordingScanningDemonstration()

    demonstration.perform_plan(demonstration.build_simulated_world())

    assert demonstration.performed == [
        [ParkArmsAction, PathPlanningNavigateAcrossFloorsAction, MoveTorsoAction]
    ]


def test_pumas_drives_between_two_runs(monkeypatch):
    demonstration = RecordingScanningDemonstration(drives_with_pumas=True)

    @dataclass(kw_only=True)
    class RecordingNavigation:
        """
        Records that it drives.
        """

        node: object
        """
        The node the navigation would publish through.
        """

        robot: HSRB
        """
        The robot that would be driven.
        """

        def drive_to(self, goal: Pose) -> None:
            """
            Record the drive.

            :param goal: Where it would drive to.
            """
            demonstration.performed.append(DrivenBy.PUMAS)

    monkeypatch.setattr(sofa_demonstrations, "PumasNavigation", RecordingNavigation)

    demonstration.perform_plan(demonstration.build_simulated_world())

    assert demonstration.performed == [
        [ParkArmsAction],
        DrivenBy.PUMAS,
        [MoveTorsoAction],
    ]


# %% the scene


def test_floors_do_not_collide():
    """
    The base, which is always close to the floor, can avoid collisions with the rest.
    """
    world = RecordingScanningDemonstration().build_simulated_world()

    floors = world.get_semantic_annotations_by_type(Floor)

    assert len(floors) == len(
        KitchenEnvironment().create_world().get_semantic_annotations_by_type(Floor)
    )
    assert all(not floor.root.has_collision() for floor in floors)
