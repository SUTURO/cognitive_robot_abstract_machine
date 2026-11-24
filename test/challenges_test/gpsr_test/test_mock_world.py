"""
Tests for the simulation of the GPSR demonstration.
"""

from __future__ import annotations

# %% Imports
from datetime import timedelta

import pytest
from typing_extensions import Iterator

from challenges.GPSR.action_mapping.mock_world import (
    GeneratedCommandDemonstration,
    OncePerCallSyncedMujocoSim,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body


# %% Fixtures


@pytest.fixture
def falling_box_world() -> World:
    """
    A world whose only link is a 10 cm box hanging free one metre above the root.
    """
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_body(root)
        box = Body(name=PrefixedName("box"))
        geometry = ShapeCollection(
            [Box(scale=Scale(0.1, 0.1, 0.1))], reference_frame=box
        )
        box.collision, box.visual = geometry, geometry
        world.add_connection(
            Connection6DoF.create_with_dofs(
                world=world,
                parent=root,
                child=box,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=1.0, reference_frame=root
                ),
            )
        )
    return world


@pytest.fixture
def falling_box_simulation(
    falling_box_world: World,
) -> Iterator[OncePerCallSyncedMujocoSim]:
    """
    A started, stepped and headless simulation of a box falling from one metre.
    """
    simulation = OncePerCallSyncedMujocoSim(world=falling_box_world, headless=True)
    simulation.start_stepped_simulation()
    yield simulation
    simulation.stop_simulation()


# %% Syncing once per call


def test_stepping_keeps_the_synchronizers_rate(
    falling_box_simulation: OncePerCallSyncedMujocoSim,
) -> None:
    """
    Stepping leaves the rate the synchronizer copies the physics into the world at as it
    was.
    """
    rate_before = 30.0
    falling_box_simulation.synchronizer.sync_rate_hz = rate_before

    falling_box_simulation.step_simulation(timedelta(milliseconds=100))

    assert falling_box_simulation.synchronizer.sync_rate_hz == rate_before


def test_stepping_shorter_than_half_a_physics_step_advances_no_time(
    falling_box_simulation: OncePerCallSyncedMujocoSim,
) -> None:
    """
    A duration that rounds to no physics step advances the simulation by none, as
    :meth:`MujocoSim.step_simulation` does.
    """
    step_size = falling_box_simulation.simulator.step_size
    time_before = falling_box_simulation.simulator.current_simulation_time

    falling_box_simulation.step_simulation(timedelta(seconds=step_size / 4))

    assert falling_box_simulation.simulator.current_simulation_time == time_before


def test_the_world_holds_the_physics_after_a_call(
    falling_box_simulation: OncePerCallSyncedMujocoSim, falling_box_world: World
) -> None:
    """
    After a call, the world holds where the physics moved the box to.
    """
    falling_box_simulation.step_simulation(timedelta(milliseconds=300))

    simulated_height = falling_box_simulation.simulator.get_body_position(
        body_name="box"
    ).result[2]
    world_height = float(falling_box_world.get_body_by_name("box").global_pose.z)
    assert world_height == pytest.approx(simulated_height)


# %% Setting up the demonstration


def test_demonstration_needs_a_plan() -> None:
    """
    A demonstration cannot be set up without the plan it carries out.
    """
    with pytest.raises(TypeError):
        GeneratedCommandDemonstration()


def test_demonstration_needs_the_spoken_instruction() -> None:
    """
    A demonstration cannot be set up without the instruction its plan was written for.
    """
    with pytest.raises(TypeError):
        GeneratedCommandDemonstration(json_plan='{"plans": []}')
