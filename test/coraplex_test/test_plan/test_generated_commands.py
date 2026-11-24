"""
Tests for mapping a command generator's plans to the actions the robot performs.
"""

from __future__ import annotations

# %% Imports
import json
from dataclasses import dataclass
from pathlib import Path

import numpy
import pytest
from typing_extensions import Any, Iterator

from krrood.patterns.path_enumeration import PathEnumeration

from coraplex.robot_plans import generated_commands
from coraplex.exceptions import (
    LocationNotInWorld,
    LocationWithoutBody,
    MissingActionArgument,
    NoFreeTableInRoom,
    NoGraspFitsGripper,
    NoStandingPoseOnFloor,
    NoPersonFound,
    NothingToFind,
    ObjectNotGraspable,
    ObjectNotInRoom,
    UnmappedActionType,
)
from coraplex.robot_plans.generated_commands import (
    ActionType,
    CollisionAllowingAction,
    DetectedObjectType,
    FindPersonAction,
    GeneratedCommand,
    GeneratedPlan,
    ManipulationAtStandingPoseAction,
    FreeSpaceFinder,
    ManipulationPlanner,
    PayloadKey,
    PlannedAction,
    WorldReferenceResolver,
)
from coraplex.perception import RoboKudoQuery, RoboKudoQueryTask
from coraplex.robot_plans.actions.composite.transporting import (
    MoveAndPickUpAction,
    MoveAndPlaceAction,
)
from coraplex.robot_plans.actions.core.navigation import (
    FaceAtAction,
    NavigateAction,
    PathPlanningNavigateAction,
)
from coraplex.robot_plans.actions.core.talking import TalkingAction
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from coraplex.robot_plans.transformation_by_room import RoomVisitPlanner
from semantic_digital_twin.collision_checking.collision_rules import AllowAllCollisions
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.reasoning.predicates import OverFloorPlan
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bedroom,
    Floor,
    Human,
    Kitchen,
    LivingRoom,
    Milk,
    Room,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)


# %% Test resources


class CommandGeneratorAnswer(PathEnumeration):
    """
    Answers of the command generator, kept as the JSON files it writes.
    """

    TWO_TASKS = Path(__file__).parents[2] / "dataset" / "generated_command.json"
    """
    Driving to the kitchen and to the living room table, and finding the apple.
    """


# %% Fixtures


@pytest.fixture
def payload() -> dict[str, Any]:
    """
    A command with two tasks, as the command generator answers it.
    """
    return json.loads(CommandGeneratorAnswer.TWO_TASKS.read_text())


@pytest.fixture
def pr2_context(
    apartment_world_pr2_copy_with_context: tuple[
        World, AbstractRobot, list[ContextExtension]
    ],
) -> StatechartContext:
    """
    The context of a plan of the PR2 in the apartment, with the milk on the table.
    """
    world, robot, extensions = apartment_world_pr2_copy_with_context
    return StatechartContext.with_extensions(world, extensions)


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
def fallback_room_types() -> set[type[Room]]:
    """
    The kinds of room a step that names no room may be performed in.
    """
    return {Kitchen, LivingRoom}


# %% Test helpers


@dataclass
class FixedStandingCandidates:
    """
    Stands in for a reachability costmap by offering fixed standing poses for a target.
    """

    target_pose: Pose
    """
    The pose the robot is to reach.
    """

    candidates: list[Pose]
    """
    The poses the robot could stand at, best first.
    """

    def __iter__(self) -> Iterator[Pose]:
        return iter(self.candidates)


def saying(text: str, location: Room | None = None) -> PlannedAction:
    """
    :param text: What the robot says.
    :param location: The room the step names, if any.
    :return: A step that says ``text``, performed in ``location``.
    """
    return PlannedAction(
        action=TalkingAction(speaking_text=text), text=text, location=location
    )


# %% Parsing a generated command


def test_actions_keep_their_order_and_fields(
    payload: dict[str, Any], context: StatechartContext
) -> None:
    """
    The actions of a plan keep the order, the kinds and the locations they were given in.
    """
    command = GeneratedCommand.from_json(payload, "go there", context)
    plan = command.plans[0]
    say = payload[PayloadKey.PLANS][0][0]
    [kitchen_annotation] = context.world.get_semantic_annotations_by_type(Kitchen)
    [living_room_annotation] = context.world.get_semantic_annotations_by_type(
        LivingRoom
    )
    talking, *drives = [action.action for action in plan.actions]
    assert type(talking) is TalkingAction
    assert [drive._type_ for drive in drives] == [
        PathPlanningNavigateAction,
        PathPlanningNavigateAction,
    ]
    assert plan.actions[0].action.speaking_text == say[PayloadKey.TEXT]
    assert [action.location for action in plan.actions] == [
        None,
        kitchen_annotation,
        living_room_annotation,
    ]


def test_command_keeps_the_spoken_instruction(
    payload: dict[str, Any], context: StatechartContext
) -> None:
    """
    The command remembers the instruction its plans were written for.
    """
    command = GeneratedCommand.from_json(payload, "go there", context)
    assert command.instruction == "go there"


def test_navigation_drives_to_a_free_standing_pose_at_each_location(
    payload: dict[str, Any], context: StatechartContext
) -> None:
    """
    Each navigation drives to the free spot for the robot at the location it names.
    """
    command = GeneratedCommand.from_json(payload, "go there", context)
    drives = command.plans[0].actions[1:]
    for drive in drives:
        expected_target = FreeSpaceFinder(context.world).standing_pose(drive.location)
        target = drive.action._kwargs_["target"]
        assert numpy.allclose(target.to_np(), expected_target.to_np())


def test_finding_an_object_drives_to_the_object(
    payload: dict[str, Any], context: StatechartContext
) -> None:
    """
    Finding an object the world already holds drives to where the object stands.
    """
    command = GeneratedCommand.from_json(payload, "go there", context)
    finding = command.plans[1].actions[1]
    apple = context.world.get_semantic_annotation_by_name("apple")
    assert isinstance(finding.action, NavigateAction)
    assert finding.object_annotation is apple
    assert numpy.allclose(
        finding.action.target_location.to_np(), apple.root.global_pose.to_np()
    )


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
        generated_commands, "find_free_space_in_room", lambda room_type, world: None
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
        FreeSpaceFinder(kitchen_environment_with_rooms).standing_pose(location)


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
    assert isinstance(action.action, FindPersonAction)
    assert action.action.person is person


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
    """
    An object to place is found by its name, wherever it stands.
    """
    [living_room_annotation] = (
        kitchen_environment_with_rooms.get_semantic_annotations_by_type(LivingRoom)
    )
    apple = kitchen_environment_with_rooms.get_semantic_annotation_by_name("apple")
    found = WorldReferenceResolver(kitchen_environment_with_rooms).resolve_object(
        "apple", ActionType.PLACE, living_room_annotation
    )
    assert found is apple


def test_object_to_pick_up_is_found_in_the_named_room(
    kitchen_environment_with_rooms: World,
) -> None:
    """
    An object to pick up is found in the room the step names.
    """
    [kitchen_annotation] = (
        kitchen_environment_with_rooms.get_semantic_annotations_by_type(Kitchen)
    )
    apple = kitchen_environment_with_rooms.get_semantic_annotation_by_name("apple")
    found = WorldReferenceResolver(kitchen_environment_with_rooms).resolve_object(
        "apple", ActionType.PICK_UP, kitchen_annotation
    )
    assert found is apple


def test_object_to_pick_up_missing_from_the_named_room_raises(
    kitchen_environment_with_rooms: World,
) -> None:
    """
    An object to pick up that the named room does not hold is not found.
    """
    [living_room_annotation] = (
        kitchen_environment_with_rooms.get_semantic_annotations_by_type(LivingRoom)
    )
    with pytest.raises(ObjectNotInRoom):
        WorldReferenceResolver(kitchen_environment_with_rooms).resolve_object(
            "apple", ActionType.PICK_UP, living_room_annotation
        )


def test_object_to_pick_up_is_the_one_in_the_named_room_not_the_one_of_its_name(
    kitchen_environment_with_rooms: World,
) -> None:
    """
    Of two bananas, the one in the named room is picked up, not the one whose name the
    step gives.
    """
    [kitchen_annotation] = (
        kitchen_environment_with_rooms.get_semantic_annotations_by_type(Kitchen)
    )
    banana_in_kitchen = kitchen_environment_with_rooms.get_semantic_annotation_by_name(
        "banana1"
    )
    found = WorldReferenceResolver(kitchen_environment_with_rooms).resolve_object(
        "banana", ActionType.PICK_UP, kitchen_annotation
    )
    assert found is banana_in_kitchen


# %% The room a step is performed in


def test_step_naming_a_room_is_performed_there(context: StatechartContext) -> None:
    """
    A step naming a room is performed in that kind of room.
    """
    [kitchen_annotation] = context.world.get_semantic_annotations_by_type(Kitchen)
    assert saying("go", kitchen_annotation).room_type is Kitchen


def test_step_naming_an_object_is_performed_in_its_room(
    context: StatechartContext,
) -> None:
    """
    A step naming an object is performed in the room the object stands in.
    """
    apple = context.world.get_semantic_annotation_by_name("apple")
    step = PlannedAction(
        action=TalkingAction(speaking_text="take"), object_annotation=apple
    )
    assert step.room_type is Kitchen


def test_step_naming_nothing_is_performed_wherever_the_robot_is() -> None:
    """
    A step naming neither a place nor an object has no room of its own.
    """
    assert saying("hello").room_type is None


# %% Allowing collisions


def test_collision_allowing_action_allows_every_collision() -> None:
    """
    While the wrapped action runs, every collision is allowed.
    """
    action = CollisionAllowingAction(
        performed_action=TalkingAction(speaking_text="placing")
    )
    rules = action.create_temporary_collision_rules()
    assert [type(rule) for rule in rules] == [AllowAllCollisions]


def test_collision_allowing_action_performs_the_wrapped_action() -> None:
    """
    The only step is the wrapped action.
    """
    performed = TalkingAction(speaking_text="placing")
    action = CollisionAllowingAction(performed_action=performed)
    assert action.create_steps() == [performed]


# %% Manipulating at a standing pose


def test_approaching_a_manipulation_keeps_its_standing_pose(
    apartment_world_pr2_copy_with_context: tuple[
        World, AbstractRobot, list[ContextExtension]
    ],
) -> None:
    """
    Approaching a manipulation leaves the standing pose it was given unchanged.
    """
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


def test_approach_stands_one_base_radius_back_from_the_standing_pose(
    apartment_world_pr2_copy_with_context: tuple[
        World, AbstractRobot, list[ContextExtension]
    ],
) -> None:
    """
    The robot first drives to one base radius further from the manipulated location
    than the standing pose, then performs the manipulation allowing collisions.
    """
    world, robot, extensions = apartment_world_pr2_copy_with_context
    statechart = Statechart(
        context=StatechartContext.with_extensions(world, extensions)
    )
    standing_pose = Pose.from_xyz_rpy(x=1.0, y=0.5, reference_frame=world.root)
    performed = TalkingAction(speaking_text="placing")
    manipulation = ManipulationAtStandingPoseAction(
        standing_pose=standing_pose,
        manipulated_location=Pose.from_xyz_rpy(
            x=2.0, y=0.5, reference_frame=world.root
        ),
        manipulation=performed,
    )
    statechart.add_node(manipulation)
    drive, allowing_collisions = manipulation.create_steps()
    approach = drive._kwargs_["target"]
    assert drive._type_ is PathPlanningNavigateAction
    assert float(approach.x) == pytest.approx(
        float(standing_pose.x) - robot.mobile_base.base_radius
    )
    assert float(approach.y) == pytest.approx(float(standing_pose.y))
    assert isinstance(allowing_collisions, CollisionAllowingAction)
    assert allowing_collisions.performed_action is performed


def test_standing_at_the_manipulated_location_approaches_the_standing_pose(
    apartment_world_pr2_copy_with_context: tuple[
        World, AbstractRobot, list[ContextExtension]
    ],
) -> None:
    """
    A standing pose right at the manipulated location leaves no direction to step back
    in, so the robot approaches the standing pose itself.
    """
    world, robot, extensions = apartment_world_pr2_copy_with_context
    statechart = Statechart(
        context=StatechartContext.with_extensions(world, extensions)
    )
    standing_pose = Pose.from_xyz_rpy(x=1.0, y=0.5, reference_frame=world.root)
    manipulation = ManipulationAtStandingPoseAction(
        standing_pose=standing_pose,
        manipulated_location=Pose.from_xyz_rpy(
            x=1.0, y=0.5, z=0.8, reference_frame=world.root
        ),
        manipulation=TalkingAction(speaking_text="placing"),
    )
    statechart.add_node(manipulation)
    drive, _ = manipulation.create_steps()
    assert numpy.allclose(drive._kwargs_["target"].to_np(), standing_pose.to_np())


# %% Picking up and placing at a reachable standing pose


def test_picking_up_stands_over_a_floor_and_reaches_for_the_object(
    pr2_context: StatechartContext,
) -> None:
    """
    Picking up stands where the robot reaches the object from a floor, and picks the
    object up there.
    """
    [milk] = pr2_context.world.get_semantic_annotations_by_type(Milk)
    pick_up = ManipulationPlanner(pr2_context).pick_up(milk)
    floors = pr2_context.world.get_semantic_annotations_by_type(Floor)
    assert any(
        OverFloorPlan(pose=pick_up.standing_pose, floor=floor.root)()
        for floor in floors
    )
    assert pick_up.manipulated_location.reference_frame is milk.root
    assert isinstance(pick_up.manipulation, MoveAndPickUpAction)


def test_picking_up_with_a_gripper_too_narrow_for_every_grasp_raises(
    pr2_context: StatechartContext,
    apartment_world_pr2_copy_with_context: tuple[
        World, AbstractRobot, list[ContextExtension]
    ],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    An object none of whose grasps fits the opened gripper cannot be picked up.
    """
    world, robot, extensions = apartment_world_pr2_copy_with_context
    monkeypatch.setattr(robot.all_arms[0].end_effector, "separation_when_open", 0.0)
    [milk] = pr2_context.world.get_semantic_annotations_by_type(Milk)
    with pytest.raises(NoGraspFitsGripper):
        ManipulationPlanner(pr2_context).pick_up(milk)


def test_placing_puts_the_held_object_down_at_the_target(
    pr2_context: StatechartContext,
) -> None:
    """
    Placing reaches for the target location and puts the held object down there.
    """
    [milk] = pr2_context.world.get_semantic_annotations_by_type(Milk)
    target_location = milk.root.global_pose
    placing = ManipulationPlanner(pr2_context).place(milk, target_location)
    assert placing.manipulated_location is target_location
    assert isinstance(placing.manipulation, MoveAndPlaceAction)


def test_standing_pose_is_the_first_candidate_over_a_floor(
    context: StatechartContext,
) -> None:
    """
    Of the standing poses offered, the first one over a floor is chosen.
    """
    root = context.world.root
    over_no_floor = Pose.from_xyz_rpy(x=-20.0, y=-20.0, reference_frame=root)
    over_kitchen_floor = Pose.from_xyz_rpy(x=2.0, y=1.0, reference_frame=root)
    candidates = FixedStandingCandidates(
        target_pose=over_kitchen_floor, candidates=[over_no_floor, over_kitchen_floor]
    )
    assert (
        ManipulationPlanner(context).standing_pose_on_floor(candidates)
        is over_kitchen_floor
    )


def test_standing_pose_without_a_candidate_over_a_floor_raises(
    context: StatechartContext,
) -> None:
    """
    A target the robot reaches only from poses over no floor cannot be reached.
    """
    root = context.world.root
    over_no_floor = Pose.from_xyz_rpy(x=-20.0, y=-20.0, reference_frame=root)
    candidates = FixedStandingCandidates(
        target_pose=over_no_floor, candidates=[over_no_floor]
    )
    with pytest.raises(NoStandingPoseOnFloor):
        ManipulationPlanner(context).standing_pose_on_floor(candidates)


# %% Finding a free place to put an object down


def test_free_pose_on_a_table_in_the_room_lies_in_the_room(
    context: StatechartContext,
) -> None:
    """
    An object put down on a table in a room is put down over that room's floor.
    """
    [kitchen_annotation] = context.world.get_semantic_annotations_by_type(Kitchen)
    apple = context.world.get_semantic_annotation_by_name("apple")
    free_pose = FreeSpaceFinder(context.world).pose_on_table_in_room(Kitchen, apple)
    assert OverFloorPlan(pose=free_pose, floor=kitchen_annotation.floor.root)()


def test_room_without_a_table_has_no_free_pose(context: StatechartContext) -> None:
    """
    A room without a table offers no place to put an object down.
    """
    apple = context.world.get_semantic_annotation_by_name("apple")
    with pytest.raises(NoFreeTableInRoom):
        FreeSpaceFinder(context.world).pose_on_table_in_room(LivingRoom, apple)


def test_no_surfaces_offer_no_free_pose(context: StatechartContext) -> None:
    """
    Without surfaces there is no place to put an object down.
    """
    apple = context.world.get_semantic_annotation_by_name("apple")
    assert FreeSpaceFinder.pose_on_first_free_surface([], apple) is None


# %% Turning a plan into a command for the room transformation


def test_leading_step_without_room_joins_the_first_room(
    kitchen: Kitchen, fallback_room_types: set[type[Room]]
) -> None:
    """
    Actions before the first one naming a room are performed in that room.
    """
    plan = GeneratedPlan(
        actions=[saying("announce"), saying("go", kitchen), saying("take")]
    )
    [step] = plan.to_command("bring it", fallback_room_types).steps
    assert step.rooms == {Kitchen}
    assert step.actions == [action.action for action in plan.actions]


def test_change_of_room_starts_a_new_step(
    kitchen: Kitchen, living_room: LivingRoom, fallback_room_types: set[type[Room]]
) -> None:
    """
    An action naming another room starts a new step.
    """
    plan = GeneratedPlan(
        actions=[
            saying("go", kitchen),
            saying("take"),
            saying("go on", living_room),
            saying("put"),
        ]
    )
    steps = plan.to_command("bring it", fallback_room_types).steps
    assert [step.rooms for step in steps] == [{Kitchen}, {LivingRoom}]
    assert [step.actions for step in steps] == [
        [plan.actions[0].action, plan.actions[1].action],
        [plan.actions[2].action, plan.actions[3].action],
    ]


def test_plan_naming_no_room_may_run_in_any_room(
    fallback_room_types: set[type[Room]],
) -> None:
    """
    A plan that names no room may be performed in any room.
    """
    plan = GeneratedPlan(actions=[saying("hello")])
    [step] = plan.to_command("greet", fallback_room_types).steps
    assert step.rooms == fallback_room_types


# %% Planning the room visits of several plans


def test_plans_turned_into_commands_share_a_visit_to_the_same_room(
    kitchen: Kitchen, living_room: LivingRoom, fallback_room_types: set[type[Room]]
) -> None:
    """
    Plans that name the same room are performed during one visit to it.
    """
    transport = GeneratedPlan(
        actions=[saying("take", kitchen), saying("put", living_room)]
    )
    question = GeneratedPlan(actions=[saying("ask", kitchen)])
    commands = [
        plan.to_command("tasks", fallback_room_types) for plan in (transport, question)
    ]
    room_visits = RoomVisitPlanner().transform_plan_by_rooms(commands)
    assert [room_visit.room for room_visit in room_visits] == [Kitchen, LivingRoom]
    assert room_visits[0].actions == [
        transport.actions[0].action,
        question.actions[0].action,
    ]


# %% Finding a person


@pytest.fixture
def visitor_context(
    pr2_context: StatechartContext,
) -> StatechartContext:
    """
    The context of the PR2 in the apartment with a person standing in it.
    """
    with pr2_context.world.modify_world():
        Human.create_with_new_body_in_world(
            name="visitor",
            world=pr2_context.world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(x=1, y=2),
        )
    return pr2_context


def test_finding_a_person_drives_in_front_of_them_and_faces_them(
    visitor_context: StatechartContext,
) -> None:
    """
    The robot drives to the viewing distance in front of the person and faces them.
    """
    person = visitor_context.world.get_semantic_annotation_by_name("visitor")
    statechart = Statechart(context=visitor_context)
    finding = FindPersonAction(person=person)
    statechart.add_node(finding)
    drive, facing = finding.create_steps()
    expected_target = visitor_context.world.transform(
        Pose.from_xyz_rpy(
            x=finding.viewing_distance, yaw=numpy.pi, reference_frame=person.root
        ),
        visitor_context.world.root,
    )
    assert drive._type_ is PathPlanningNavigateAction
    assert numpy.allclose(drive._kwargs_["target"].to_np(), expected_target.to_np())
    assert isinstance(facing, FaceAtAction)


def test_finding_a_person_with_a_detection_asks_the_perception(
    visitor_context: StatechartContext,
) -> None:
    """
    With a detection, the robot asks the perception pipeline once it faces the person.
    """
    person = visitor_context.world.get_semantic_annotation_by_name("visitor")
    detection = RoboKudoQuery(object_type=DetectedObjectType.HUMAN)
    statechart = Statechart(context=visitor_context)
    finding = FindPersonAction(person=person, detection=detection)
    statechart.add_node(finding)
    *_, asking = finding.create_steps()
    assert isinstance(asking, RoboKudoQueryTask)
    assert asking.query is detection
