"""
Tests for the simulation of the GPSR demonstration.
"""

from __future__ import annotations

# %% Imports
from datetime import timedelta

import pytest
from typing_extensions import Iterator

from challenges.GPSR.demonstration import (
    GeneratedCommandDemonstration,
    OncePerCallSyncedMujocoSim,
    SceneContactCategory,
    SceneName,
)
from coraplex.plans.context_extensions import RobotAccess
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.adapters.multi_sim import MujocoGeom
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
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


# %% The scene of the demonstration


@pytest.fixture(scope="module")
def demonstration() -> GeneratedCommandDemonstration:
    """
    The demonstration of a command without plans, run without a viewer or perception.
    """
    return GeneratedCommandDemonstration(
        json_plan='{"plans": []}', instruction="", headless=True
    )


@pytest.fixture(scope="module")
def demonstration_world(demonstration: GeneratedCommandDemonstration) -> World:
    """
    The kitchen of the demonstration with the milk, the person and the HSR in it.
    """
    return demonstration.build_simulated_world()


def test_milk_stands_at_the_front_edge_of_the_counter_top(
    demonstration_world: World,
) -> None:
    """
    The milk stands on the counter top, flush with the edge nearer to the robot.
    """
    root = demonstration_world.root
    milk_box = (
        demonstration_world.get_body_by_name(SceneName.MILK)
        .collision.as_bounding_box_collection_in_frame(root)
        .bounding_box()
    )
    counter_box = (
        demonstration_world.get_body_by_name(SceneName.COUNTER_TOP)
        .collision.as_bounding_box_collection_in_frame(root)
        .bounding_box()
    )
    front_edge_y = min(counter_box.min_y, counter_box.max_y, key=abs)
    assert milk_box.min_z == pytest.approx(counter_box.max_z)
    assert front_edge_y in (
        pytest.approx(milk_box.min_y),
        pytest.approx(milk_box.max_y),
    )


def test_person_stands_with_their_back_at_the_distance_from_the_window_sill(
    demonstration: GeneratedCommandDemonstration, demonstration_world: World
) -> None:
    """
    The person's back keeps :attr:`person_distance_to_window` from the window sill.
    """
    human_box = (
        demonstration_world.get_body_by_name(SceneName.HUMAN)
        .collision.as_bounding_box_collection_in_frame(demonstration_world.root)
        .bounding_box()
    )
    window_wall = demonstration.kitchen_environment.measurements.window_wall
    assert human_box.max_x == pytest.approx(
        window_wall.sill_front_x - demonstration.person_distance_to_window
    )


def test_the_person_is_a_human_named_as_the_command_names_them(
    demonstration_world: World,
) -> None:
    """
    The person is a human the command can name with ``"person"``.
    """
    person = demonstration_world.get_semantic_annotation_by_name(SceneName.PERSON)
    assert isinstance(person, Human)


def test_only_the_grippers_of_the_robot_touch_the_scene_in_mujoco(
    demonstration_world: World,
) -> None:
    """
    Every collision shape of the robot but those of its grippers is in the robot body's
    own contact category; the grippers' shapes keep the default one.
    """
    [robot] = demonstration_world.get_semantic_annotations_by_type(HSRB)
    gripper_bodies = {
        body
        for arm in robot.all_arms
        for body in arm.end_effector.bodies_with_collision
    }
    for body in robot.bodies_with_collision:
        for geometry in body.collision:
            contact = geometry.get_simulator_property_of_type(MujocoGeom)
            if body in gripper_bodies:
                assert contact is None
            else:
                assert contact.contact_type == SceneContactCategory.ROBOT_BODY


def test_context_without_perception_gives_the_plan_its_robot(
    demonstration: GeneratedCommandDemonstration, demonstration_world: World
) -> None:
    """
    Without perception, the plan's context only gives access to the robot.
    """
    [robot] = demonstration_world.get_semantic_annotations_by_type(HSRB)
    [extension] = demonstration.build_context_extensions(demonstration_world)
    assert isinstance(extension, RobotAccess)
    assert extension.robot is robot
