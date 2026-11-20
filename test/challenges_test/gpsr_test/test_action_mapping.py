from __future__ import annotations

# %% Imports
import json
from pathlib import Path

import numpy
import pytest
from typing_extensions import Any

from challenges.GPSR.action_mapping.exceptions import ObjectNotInRoom
from challenges.GPSR.action_mapping.action_mapping import (
    ActionType,
    GeneratedCommand,
    GeneratedPlan,
    ManipulationAtStandingPoseAction,
    PayloadKey,
    PlannedAction,
)
from coraplex.robot_plans.actions.core.navigation import PathPlanningNavigateAction
from coraplex.robot_plans.actions.core.talking import TalkingAction
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import Pose
from coraplex.robot_plans.transformation_by_room import RoomVisitPlanner
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Kitchen,
    LivingRoom,
    Room,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body


# %% Fixtures


@pytest.fixture
def payload() -> dict[str, Any]:
    """
    A command with three tasks, as the command generator answers it.
    """
    return json.loads(
        (Path(__file__).parent / "resources" / "generated_command.json").read_text()
    )


@pytest.fixture
def context(kitchen_environment_with_rooms: World) -> StatechartContext:
    """
    A context whose world has a kitchen and a living room for the steps to name.
    """
    return StatechartContext(world=kitchen_environment_with_rooms)


@pytest.fixture
def kitchen() -> Kitchen:
    """
    A kitchen outside of any world, as the location a step names.
    """
    return Kitchen(
        floor=Floor(root=Body(name=PrefixedName("kitchen_floor"))),
        name=PrefixedName("kitchen"),
    )


@pytest.fixture
def living_room() -> LivingRoom:
    """
    A living room outside of any world, as the location a step names.
    """
    return LivingRoom(
        floor=Floor(root=Body(name=PrefixedName("living_room_floor"))),
        name=PrefixedName("living_room"),
    )


@pytest.fixture
def any_room() -> set[type[Room]]:
    """
    The kinds of room a step that names no room may be performed in.
    """
    return {Kitchen, LivingRoom}


def saying(text: str, location: Room | None = None) -> PlannedAction:
    """
    :param text: What the robot says.
    :param location: The room the step names, if any.
    :return: A step that says ``text``, performed in ``location``.
    """
    return PlannedAction(
        action_type=TalkingAction(speaking_text=text), text=text, location=location
    )


# %% Parsing a generated command


def test_actions_keep_their_order_and_fields(
    payload: dict[str, Any], context: StatechartContext
) -> None:
    command = GeneratedCommand.from_json(payload, "go there", context)
    [plan] = command.plans
    say = payload[PayloadKey.PLANS][0][0]
    [kitchen_room] = context.world.get_semantic_annotations_by_type(Kitchen)
    [living_room_room] = context.world.get_semantic_annotations_by_type(LivingRoom)
    talking, *drives = [action.action_type for action in plan.actions]
    assert type(talking) is TalkingAction
    assert [drive._type_ for drive in drives] == [
        PathPlanningNavigateAction,
        PathPlanningNavigateAction,
    ]
    assert plan.actions[0].action_type.speaking_text == say[PayloadKey.TEXT]
    assert [action.location for action in plan.actions] == [
        None,
        kitchen_room,
        living_room_room,
    ]


def test_action_kind_not_mapped_yet_has_no_action(context: StatechartContext) -> None:
    action = PlannedAction.from_json({PayloadKey.ACTION: ActionType.OPEN}, context)
    assert action.action_type is None


# %% Finding the object a step names


def test_object_to_place_is_found_outside_the_target_room(
    kitchen_environment_with_rooms: World,
) -> None:
    [living_room_room] = (
        kitchen_environment_with_rooms.get_semantic_annotations_by_type(LivingRoom)
    )
    apple = kitchen_environment_with_rooms.get_semantic_annotation_by_name("apple")
    found = PlannedAction.find_object_annotation(
        "apple", ActionType.PLACE, living_room_room, kitchen_environment_with_rooms
    )
    assert found is apple


def test_object_to_pick_up_is_found_in_the_named_room(
    kitchen_environment_with_rooms: World,
) -> None:
    [kitchen_room] = kitchen_environment_with_rooms.get_semantic_annotations_by_type(
        Kitchen
    )
    apple = kitchen_environment_with_rooms.get_semantic_annotation_by_name("apple")
    found = PlannedAction.find_object_annotation(
        "apple", ActionType.PICK_UP, kitchen_room, kitchen_environment_with_rooms
    )
    assert found is apple


def test_object_to_pick_up_missing_from_the_named_room_raises(
    kitchen_environment_with_rooms: World,
) -> None:
    [living_room_room] = (
        kitchen_environment_with_rooms.get_semantic_annotations_by_type(LivingRoom)
    )
    with pytest.raises(ObjectNotInRoom):
        PlannedAction.find_object_annotation(
            "apple",
            ActionType.PICK_UP,
            living_room_room,
            kitchen_environment_with_rooms,
        )


# %% Manipulating at a standing pose


def test_approaching_a_manipulation_keeps_its_standing_pose(
    apartment_world_pr2_copy_with_context,
) -> None:
    world, robot, extensions = apartment_world_pr2_copy_with_context
    statechart = Statechart(
        context=StatechartContext.with_extensions(world, extensions)
    )
    standing_pose = Pose.from_xyz_rpy(x=1.0, y=0.5, reference_frame=world.root)
    standing_pose_before = standing_pose.to_np().copy()
    manipulation = ManipulationAtStandingPoseAction(
        standing_pose=standing_pose,
        manipulated_location=Pose.from_xyz_rpy(
            x=2.0, y=0.5, reference_frame=world.root
        ),
        manipulation=TalkingAction(speaking_text="placing"),
    )
    statechart.add_node(manipulation)
    manipulation.create_steps()
    assert numpy.allclose(standing_pose.to_np(), standing_pose_before)


# %% Rewrapping a plan into a command for the room transformation


def test_leading_step_without_room_joins_the_first_room(
    kitchen: Kitchen, any_room: set[type[Room]]
) -> None:
    plan = GeneratedPlan(
        actions=[saying("announce"), saying("go", kitchen), saying("take")]
    )
    [step] = plan.rewrap_into_command("bring it", any_room).steps
    assert step.rooms == {Kitchen}
    assert step.actions == [action.action_type for action in plan.actions]


def test_change_of_room_starts_a_new_step(
    kitchen: Kitchen, living_room: LivingRoom, any_room: set[type[Room]]
) -> None:
    plan = GeneratedPlan(
        actions=[
            saying("go", kitchen),
            saying("take"),
            saying("go on", living_room),
            saying("put"),
        ]
    )
    steps = plan.rewrap_into_command("bring it", any_room).steps
    assert [step.rooms for step in steps] == [{Kitchen}, {LivingRoom}]
    assert [step.actions for step in steps] == [
        [plan.actions[0].action_type, plan.actions[1].action_type],
        [plan.actions[2].action_type, plan.actions[3].action_type],
    ]


def test_plan_naming_no_room_may_run_in_any_room(any_room: set[type[Room]]) -> None:
    plan = GeneratedPlan(actions=[saying("hello")])
    [step] = plan.rewrap_into_command("greet", any_room).steps
    assert step.rooms == any_room


def test_rewrapped_plans_share_a_visit_to_the_same_room(
    kitchen: Kitchen, living_room: LivingRoom, any_room: set[type[Room]]
) -> None:
    transport = GeneratedPlan(
        actions=[saying("take", kitchen), saying("put", living_room)]
    )
    question = GeneratedPlan(actions=[saying("ask", kitchen)])
    commands = [
        plan.rewrap_into_command("tasks", any_room) for plan in (transport, question)
    ]
    room_visits = RoomVisitPlanner().transform_plan_by_rooms(commands)
    assert [room_visit.room for room_visit in room_visits] == [Kitchen, LivingRoom]
    assert room_visits[0].actions == [
        transport.actions[0].action_type,
        question.actions[0].action_type,
    ]
