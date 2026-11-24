from __future__ import annotations

# %% Imports
import json
from pathlib import Path

import numpy
import pytest
from typing_extensions import Any

from challenges.GPSR.action_mapping import action_mapping
from challenges.GPSR.action_mapping.exceptions import (
    LocationNotInWorld,
    LocationWithoutBody,
    MissingActionArgument,
    NoFreeTableInRoom,
    NoPersonFound,
    NothingToFind,
    ObjectNotGraspable,
    ObjectNotInRoom,
    UnmappedActionType,
)
from challenges.GPSR.action_mapping.action_mapping import (
    ActionType,
    FindPersonAction,
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
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from coraplex.robot_plans.transformation_by_room import RoomVisitPlanner
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bedroom,
    Floor,
    Human,
    Kitchen,
    LivingRoom,
    Room,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)


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


# %% Rejecting steps that cannot be mapped


def test_action_kind_not_mapped_yet_raises(context: StatechartContext) -> None:
    """
    A kind of step the robot cannot perform yet is rejected instead of becoming no
    action.
    """
    with pytest.raises(UnmappedActionType):
        PlannedAction.from_json({PayloadKey.ACTION: ActionType.OPEN}, context)


def test_unknown_action_kind_raises(context: StatechartContext) -> None:
    """
    A kind of step the command generator made up is rejected.
    """
    with pytest.raises(UnmappedActionType):
        PlannedAction.from_json({PayloadKey.ACTION: "dance"}, context)


@pytest.mark.parametrize(
    "action, missing_argument",
    [
        (ActionType.SAY, PayloadKey.TEXT),
        (ActionType.NAVIGATE, PayloadKey.LOCATION),
        (ActionType.PICK_UP, PayloadKey.OBJECT),
        (ActionType.PLACE, PayloadKey.OBJECT),
    ],
)
def test_step_without_its_argument_names_the_missing_argument(
    context: StatechartContext, action: ActionType, missing_argument: PayloadKey
) -> None:
    """
    A step that lacks the argument its kind needs names that argument.
    """
    with pytest.raises(MissingActionArgument) as raised:
        PlannedAction.from_json({PayloadKey.ACTION: action}, context)
    assert raised.value.argument == missing_argument


def test_placing_without_a_location_names_the_missing_location(
    context: StatechartContext,
) -> None:
    """
    A placing step that names the object but no location names the location as missing.
    """
    with pytest.raises(MissingActionArgument) as raised:
        PlannedAction.from_json(
            {PayloadKey.ACTION: ActionType.PLACE, PayloadKey.OBJECT: "apple"}, context
        )
    assert raised.value.argument == PayloadKey.LOCATION


def test_placing_in_a_room_needs_no_free_standing_spot(
    context: StatechartContext, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Placing in a room looks for a table there, even when the room has no free spot for
    the robot to stand on, instead of reporting the named room as missing.
    """
    monkeypatch.setattr(
        action_mapping, "find_free_space_in_room", lambda room_type, world: None
    )
    with pytest.raises(NoFreeTableInRoom):
        PlannedAction.from_json(
            {
                PayloadKey.ACTION: ActionType.PLACE,
                PayloadKey.OBJECT: "apple",
                PayloadKey.LOCATION: "living room",
            },
            context,
        )


def test_finding_without_object_or_person_raises(context: StatechartContext) -> None:
    """
    A finding step that names neither an object nor a person is rejected.
    """
    with pytest.raises(NothingToFind):
        PlannedAction.from_json({PayloadKey.ACTION: ActionType.FIND}, context)


def test_picking_up_something_without_grasps_raises(
    context: StatechartContext,
) -> None:
    """
    An object the robot has no grasps for, such as a sofa, is rejected for picking up.
    """
    with pytest.raises(ObjectNotGraspable):
        PlannedAction.from_json(
            {PayloadKey.ACTION: ActionType.PICK_UP, PayloadKey.OBJECT: "sofa"}, context
        )


def test_location_missing_from_the_world_raises(context: StatechartContext) -> None:
    """
    A step naming a kind of place the world has none of, such as a bedroom, is rejected.
    """
    with pytest.raises(LocationNotInWorld) as raised:
        PlannedAction.from_json(
            {PayloadKey.ACTION: ActionType.NAVIGATE, PayloadKey.LOCATION: "bedroom"},
            context,
        )
    assert raised.value.location_type is Bedroom


def test_standing_next_to_a_location_without_a_body_raises(
    kitchen_environment_with_rooms: World,
) -> None:
    """
    The robot cannot stand next to a location that has no body.
    """
    location = SemanticAnnotation(name=PrefixedName("somewhere"))
    with pytest.raises(LocationWithoutBody):
        PlannedAction.free_standing_pose(location, kitchen_environment_with_rooms)


# %% Finding the person a step names


def test_person_is_found_whatever_the_description_says(
    context: StatechartContext,
) -> None:
    """
    A person described in free text is the person in the world, not an annotation of
    that name.
    """
    with context.world.modify_world():
        person = Human.create_with_new_body_in_world(
            name="visitor",
            world=context.world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=1, y=1),
        )
    action = PlannedAction.from_json(
        {
            PayloadKey.ACTION: ActionType.FIND,
            PayloadKey.PERSON: "person pointing to the right",
        },
        context,
    )
    assert isinstance(action.action_type, FindPersonAction)
    assert action.action_type.person is person


def test_finding_a_person_in_a_world_without_people_raises(
    context: StatechartContext,
) -> None:
    """
    A person cannot be found in a world without any.
    """
    with pytest.raises(NoPersonFound):
        PlannedAction.from_json(
            {PayloadKey.ACTION: ActionType.FIND, PayloadKey.PERSON: "person"}, context
        )


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
