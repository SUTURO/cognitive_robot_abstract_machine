"""
Tests for the HSRB picking a crumpled T-shirt up off a table by friction, holding it and
dropping it, in MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy
import pytest

from cramph.executor import StatechartExecutor, TickingExecutorExtension
from coraplex.plans.executors import PlanExecutor
from experiments.t_shirt_demo.demo import SceneEntity
from experiments.t_shirt_demo.exceptions import NoTShirtWithinReach
from experiments.t_shirt_demo.pick_and_drop import TShirtPickAndDropDemonstration
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import TShirt
from semantic_digital_twin.world import World

from ...pytest_environment import runs_in_continuous_integration

pytestmark = pytest.mark.skipif(
    not HSRB.is_description_installed(),
    reason="The description of the HSRB is not installed",
)

# %% the scene


@pytest.fixture(scope="module")
def demonstration_world() -> tuple[TShirtPickAndDropDemonstration, World]:
    """
    The demonstration and the world it builds, before any physics ran.
    """
    demonstration = TShirtPickAndDropDemonstration(headless=True)
    return demonstration, demonstration.build_simulated_world()


def test_the_t_shirt_starts_upright_above_the_table(demonstration_world) -> None:
    """
    The T-shirt starts standing on its hem just above the table, so that it crumples as
    it falls.
    """
    demonstration, world = demonstration_world
    [t_shirt] = world.get_semantic_annotations_by_type(TShirt)
    vertices = t_shirt.cloth_sheet.starting_surface.vertices
    garment_extents = t_shirt.cloth_sheet.surface.mesh.extents

    table = demonstration.table_scale
    assert vertices[:, 2].min() == pytest.approx(
        table.z + demonstration.t_shirt_drop_height, abs=1e-6
    )
    assert numpy.ptp(vertices[:, 2]) == pytest.approx(garment_extents[0], abs=1e-6)
    assert numpy.ptp(vertices[:, 0]) == pytest.approx(garment_extents[2], abs=1e-6)
    assert vertices[:, 0].min() > demonstration.table_distance
    assert numpy.abs(vertices[:, 1]).max() <= table.y / 2


# %% choosing where to take it


def test_the_fold_taken_is_the_highest_point_within_reach(demonstration_world) -> None:
    """
    A higher point of the pile out of reach is passed over for the highest one the
    gripper can reach down to.
    """
    demonstration, world = demonstration_world
    edge = demonstration.table_distance
    top = demonstration.table_scale.z
    reachable_low = [edge + 0.05, 0.0, top + 0.02]
    reachable_high = [edge + 0.1, 0.05, top + 0.08]
    too_deep = [edge + demonstration.reach_into_table + 0.05, 0.0, top + 0.15]
    too_far_aside = [edge + 0.1, demonstration.lateral_reach + 0.05, top + 0.15]

    fold = demonstration.highest_fold_within_reach(
        numpy.array([reachable_low, too_deep, reachable_high, too_far_aside]), world
    )

    assert fold.to_np()[:3].tolist() == pytest.approx(reachable_high)


def test_a_t_shirt_out_of_reach_cannot_be_taken(demonstration_world) -> None:
    """
    A pile lying only where the gripper cannot reach down to offers no fold to take.
    """
    demonstration, world = demonstration_world
    out_of_reach = [
        demonstration.table_distance + demonstration.reach_into_table + 0.05,
        0.0,
        demonstration.table_scale.z + 0.05,
    ]

    with pytest.raises(NoTShirtWithinReach):
        demonstration.highest_fold_within_reach(numpy.array([out_of_reach]), world)


# %% picking it up, holding it and dropping it


@dataclass
class TShirtRecordedEveryTick(TickingExecutorExtension):
    """
    Records where the vertices of the T-shirt are after every tick, with the simulated
    time.
    """

    demonstration: TShirtPickAndDropDemonstration = field(kw_only=True)
    """
    The demonstration whose simulation holds the T-shirt.
    """

    times: list[float] = field(default_factory=list, init=False)
    """
    The simulated time after each tick, in seconds.
    """

    vertices: list[numpy.ndarray] = field(default_factory=list, init=False)
    """
    Where each vertex of the T-shirt is after each tick.
    """

    def after_tick(self, executor: StatechartExecutor) -> None:
        simulator = self.demonstration.simulation.simulator
        self.times.append(simulator.current_simulation_time)
        self.vertices.append(
            simulator.get_flex_vertex_positions(flex_name=SceneEntity.T_SHIRT).result
        )


@dataclass
class TShirtPickAndDropRecordingTheTShirt(TShirtPickAndDropDemonstration):
    """
    The demonstration, recording the T-shirt after every tick of its plan.
    """

    recording: TShirtRecordedEveryTick | None = field(init=False, default=None)
    """
    The recording of the T-shirt, None until the plan is performed.
    """

    floor_clearance: float = 0.05
    """
    How high above the floor every vertex of the T-shirt has to be for the T-shirt to
    count as off the floor, in metres.
    """

    def create_executor(self, world: World) -> PlanExecutor:
        """
        :meth:`TShirtPickAndDropDemonstration.create_executor`, recording the T-shirt.
        """
        executor = super().create_executor(world)
        self.recording = TShirtRecordedEveryTick(demonstration=self)
        executor.extensions.append(self.recording)
        return executor

    def longest_time_hanging_in_the_air(self) -> float:
        """
        :return: The longest stretch of simulated time, in seconds, through which no
            vertex of the T-shirt was over the table or on the floor.
        """
        longest, started = 0.0, None
        for time, vertices in zip(self.recording.times, self.recording.vertices):
            in_the_air = (
                vertices[:, 0].max() < self.table_distance
                and vertices[:, 2].min() > self.floor_clearance
            )
            if not in_the_air:
                started = None
                continue
            started = time if started is None else started
            longest = max(longest, time - started)
        return longest


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_the_t_shirt_is_held_in_the_air_then_dropped_onto_the_floor() -> None:
    """
    The gripper holds the crumpled T-shirt by friction, hanging clear of the table and
    the floor for as long as it is to be held, and once it opens, the T-shirt ends up
    lying on the floor.
    """
    demonstration = TShirtPickAndDropRecordingTheTShirt(headless=True)

    demonstration.run()

    assert (
        demonstration.longest_time_hanging_in_the_air()
        >= demonstration.holding_duration
    )
    at_the_end = demonstration.recording.vertices[-1]
    assert at_the_end[:, 0].max() < demonstration.table_distance
    assert at_the_end[:, 2].max() < demonstration.table_scale.z
