"""
Mapping the plans a command generator writes for a spoken command to the actions the
robot performs.
"""

from __future__ import annotations

# %% Imports
import json
import logging

import numpy
from dataclasses import dataclass
from enum import StrEnum

from typing_extensions import Any, Self

from challenges.GPSR.action_mapping.exceptions import (
    LocationNotInWorld,
    LocationWithoutBody,
    MissingActionArgument,
    NoFreeStandingPose,
    NoFreeSurface,
    NoFreeTableInRoom,
    NoGraspFitsGripper,
    NoPersonFound,
    NoStandingPoseOnFloor,
    NothingToFind,
    ObjectNotGraspable,
    ObjectNotInRoom,
    UnmappedActionType,
)
from challenges.semantic_annotation_lookup.class_finder import (
    SemanticAnnotationClassFinder,
)


from coraplex.locations.locations import ReachabilityLocation
from coraplex.perception import RoboKudoQuery, RoboKudoQueryTask
from coraplex.plans.context_extensions import RobotAccess
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.base import Action, SequentialAction
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
from coraplex.robot_plans.task_definitions import Command, Step
from cramph.context import ContextExtension, StatechartContext
from cramph.node import StatechartNode
from krrood.entity_query_language.factories import a
from krrood.entity_query_language.query.match import Match
from krrood_spatial_math.spatial_types import Point3, Pose
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import AllowAllCollisions
from semantic_digital_twin.reasoning.predicates import OverFloorPlan
from semantic_digital_twin.reasoning.queries import (
    find_free_space_in_room,
    find_free_space_next_to,
    find_objects_in_room,
    find_objects_room,
)
from semantic_digital_twin.semantic_annotations.mixins import (
    HasGraspCandidates,
    HasRootBody,
    HasSupportingSurface,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Room,
    Floor,
    Human,
    Table,
)

from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation


# %% Payload vocabulary


class PayloadKey(StrEnum):
    """
    The keys of a generated command's JSON payload.
    """

    PLANS = "plans"
    ACTION = "action"
    TEXT = "text"
    LOCATION = "location"
    OBJECT = "object"
    PERSON = "person"


class ActionType(StrEnum):
    """
    The kinds of action a generated plan consists of.
    """

    SAY = "say"
    NAVIGATE = "navigate"
    PICK_UP = "pick_up"
    PLACE = "place"
    FIND = "find"
    OPEN = "open"


class DetectedObjectType(StrEnum):
    """
    The kinds of object the robot asks its perception pipeline to detect.
    """

    HUMAN = "human"
    """
    A person, wherever they stand.
    """


# %% Perception


@dataclass
class PersonDetection(ContextExtension):
    """
    Lets a plan ask the perception pipeline to detect the person it found.
    """

    query: RoboKudoQuery
    """
    What the perception pipeline is asked once the robot faces a person.
    """


# %% Actions


@dataclass(eq=False, repr=False, kw_only=True)
class FindPersonAction(SequentialAction):
    """
    Drives around the furniture and walls to stand in front of a person, facing them,
    and, if a detection is given, asks the perception pipeline to detect them.
    """

    person: Human
    """
    The person to find.
    """

    viewing_distance: float = 2.0
    """
    How far in front of the person the robot stands, in meters.
    """

    detection: RoboKudoQuery | None = None
    """
    What the perception pipeline is asked once the robot faces the person; ``None`` to
    only drive in front of them.
    """

    def create_steps(self) -> list[StatechartNode | Match]:
        viewing_pose = Pose.from_xyz_rpy(
            x=self.viewing_distance,
            yaw=numpy.pi,
            reference_frame=self.person.root,
        )
        steps: list[StatechartNode | Match] = [
            a(PathPlanningNavigateAction)(
                target=self.world.transform(viewing_pose, self.world.root)
            ),
            FaceAtAction(target=Pose(reference_frame=self.person.root)),
        ]
        if self.detection is not None:
            steps.append(RoboKudoQueryTask(query=self.detection))
        return steps


@dataclass(eq=False, repr=False, kw_only=True)
class CollisionAllowingAction(SequentialAction):
    """
    Performs an action during which the robot may touch anything; collision avoidance
    holds again once the action ended.
    """

    performed_action: Action
    """
    The action performed while the robot may touch anything.
    """

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        Differs from :meth:`Action.create_temporary_collision_rules` in allowing every
        collision.
        """
        return [AllowAllCollisions()]

    def create_steps(self) -> list[StatechartNode]:
        return [self.performed_action]


@dataclass(eq=False, repr=False, kw_only=True)
class ManipulationAtStandingPoseAction(SequentialAction):
    """
    Drives around the furniture and walls, avoiding collisions, to one base radius in
    front of where the robot stands for a manipulation, then performs the manipulation,
    which drives the last stretch and during which the robot may touch anything.

    .. note:: The standing pose may lie closer to the furniture than a planned path
        keeps the base.
    """

    standing_pose: Pose
    """
    Where the robot's base stands while it performs :attr:`manipulation`.
    """

    manipulated_location: Pose
    """
    Where the manipulation reaches to: the object it picks up or the place it puts the
    object down.
    """

    manipulation: Action
    """
    What the robot does with its arm once it stands at :attr:`standing_pose`, starting
    with driving there.
    """

    def create_steps(self) -> list[StatechartNode | Match]:
        root = self.world.root
        world_T_standing = self.world.transform(self.standing_pose, root)
        world_P_manipulated = self.world.transform(
            self.manipulated_location, root
        ).position
        away_from_manipulated = numpy.array(
            [
                float(world_T_standing.x) - float(world_P_manipulated.x),
                float(world_T_standing.y) - float(world_P_manipulated.y),
            ]
        )
        step_back = (
            self.robot.mobile_base.base_radius
            * away_from_manipulated
            / numpy.linalg.norm(away_from_manipulated)
        )
        approach_pose = Pose(
            Point3(
                x=float(world_T_standing.x) + step_back[0],
                y=float(world_T_standing.y) + step_back[1],
                z=float(world_T_standing.z),
            ),
            world_T_standing.orientation,
            reference_frame=root,
        )
        return [
            a(PathPlanningNavigateAction)(target=approach_pose),
            CollisionAllowingAction(performed_action=self.manipulation),
        ]


# %% Plans


@dataclass
class PlannedAction:
    """
    One step of a generated plan, as the command generator described it.
    """

    action_type: Action | Match
    """
    What the robot does in this step, as an action or as a statement grounded when the
    step starts.
    """

    text: str | None = None
    """
    What the robot says, for :attr:`ActionType.SAY`.
    """

    location: SemanticAnnotation | None = None
    """
    The named place the step goes to, places at or opens.
    """

    object_annotation: HasRootBody | None = None
    """
    The object the step picks up or places.
    """

    person: Human | None = None
    """
    The person the step looks for.
    """

    @property
    def room_type(self) -> type[Room] | None:
        """
        :return: The kind of room this step is performed in: the room it names, else the
            room of the place or object it names; ``None`` if it is performed wherever
            the robot already is.
        """
        if isinstance(self.location, Room):
            return type(self.location)
        for located in (self.location, self.object_annotation):
            if not isinstance(located, HasRootBody):
                continue
            room = find_objects_room(located)
            return None if room is None else type(room)
        return None

    @classmethod
    def from_json(cls, data: dict[str, str], context: StatechartContext) -> Self:
        """
        :param data: One action of a generated plan.
        :param context: The context of the plan the action runs in, whose robot performs
            it in its world.
        :return: The action it describes.
        :raises MissingActionArgument: If the action lacks an argument its kind needs.
        :raises UnmappedActionType: If the robot cannot perform this kind of action.
        """
        world = context.world
        location_name = data.get(PayloadKey.LOCATION)
        location_type = (
            None
            if location_name is None
            else SemanticAnnotationClassFinder().find_place(location_name)
        )
        location = (
            None if location_type is None else cls.first_location(location_type, world)
        )
        object_name = data.get(PayloadKey.OBJECT)
        searched_object = (
            None
            if object_name is None
            else cls.find_object_annotation(
                object_name, data[PayloadKey.ACTION], location, world
            )
        )
        person_description = data.get(PayloadKey.PERSON)
        searched_person = (
            None
            if person_description is None
            else cls.find_person(person_description, location, world)
        )

        match data[PayloadKey.ACTION]:
            case ActionType.SAY:
                text = data.get(PayloadKey.TEXT)
                if text is None:
                    raise MissingActionArgument(
                        action=ActionType.SAY, argument=PayloadKey.TEXT
                    )
                action = TalkingAction(speaking_text=text)
            case ActionType.NAVIGATE:
                if location is None:
                    raise MissingActionArgument(
                        action=ActionType.NAVIGATE, argument=PayloadKey.LOCATION
                    )
                action = a(PathPlanningNavigateAction)(
                    target=cls.free_standing_pose(location, world)
                )
            case ActionType.PICK_UP:
                action = cls.pick_up_from_reachable_standing_pose(
                    cls.carried_object(ActionType.PICK_UP, searched_object), context
                )
            case ActionType.PLACE:
                held_object = cls.carried_object(ActionType.PLACE, searched_object)
                if location is None:
                    raise MissingActionArgument(
                        action=ActionType.PLACE, argument=PayloadKey.LOCATION
                    )
                action = cls.place_from_reachable_standing_pose(
                    held_object,
                    cls.placing_pose(location_type, location, held_object, world),
                    context,
                )
            case ActionType.FIND:
                action = cls.finding_action(searched_object, searched_person, context)
            case _:
                raise UnmappedActionType(action=data[PayloadKey.ACTION])

        return cls(
            action_type=action,
            text=data.get(PayloadKey.TEXT),
            location=location,
            object_annotation=searched_object,
            person=searched_person,
        )

    @staticmethod
    def first_location(
        location_type: type[SemanticAnnotation], world: World
    ) -> SemanticAnnotation:
        """
        :param location_type: The kind of place a step names.
        :param world: The world holding the place.
        :return: The first place of that kind in ``world``.
        :raises LocationNotInWorld: If ``world`` holds no place of that kind.
        """
        locations = world.get_semantic_annotations_by_type(location_type)
        if not locations:
            raise LocationNotInWorld(location_type=location_type)
        return locations[0]

    @staticmethod
    def free_standing_pose(location: SemanticAnnotation, world: World) -> Pose:
        """
        :param location: The place the robot is to stand at.
        :param world: The world holding the place.
        :return: A free spot for the robot's base: in ``location`` if it is a room, else
            next to it.
        :raises LocationWithoutBody: If ``location`` is neither a room nor has a body.
        :raises NoFreeStandingPose: If no spot is free.
        """
        if isinstance(location, Room):
            standing_pose = find_free_space_in_room(type(location), world)
        elif isinstance(location, HasRootBody):
            standing_pose = find_free_space_next_to(
                robot_diameter=0.7, semantic_annotation=location
            )
        else:
            raise LocationWithoutBody(location=location)
        if standing_pose is None:
            raise NoFreeStandingPose(location=location)
        return standing_pose

    @staticmethod
    def carried_object(
        action: ActionType, named_object: HasRootBody | None
    ) -> HasGraspCandidates:
        """
        :param action: The kind of action that picks up or puts down the object.
        :param named_object: The object the step names, if any.
        :return: ``named_object``, which the robot can grasp.
        :raises MissingActionArgument: If the step names no object.
        :raises ObjectNotGraspable: If the robot cannot grasp the object.
        """
        if named_object is None:
            raise MissingActionArgument(action=action, argument=PayloadKey.OBJECT)
        if not isinstance(named_object, HasGraspCandidates):
            raise ObjectNotGraspable(named_object=named_object)
        return named_object

    @classmethod
    def placing_pose(
        cls,
        location_type: type[SemanticAnnotation],
        location: SemanticAnnotation,
        held_object: HasGraspCandidates,
        world: World,
    ) -> Pose:
        """
        :param location_type: The kind of place the step names.
        :param location: The place the step names.
        :param held_object: The object to put down.
        :param world: The world holding the place.
        :return: Where to put the object down: on a table in a room, on the first free
            surface of the named kind, or else on the spot next to the place.
        :raises NoFreeTableInRoom: If no table in the named room has space.
        :raises NoFreeSurface: If no surface of the named kind has space.
        """
        if isinstance(location, Room):
            return cls.free_pose_on_table_in_room(location_type, held_object, world)
        if not isinstance(location, HasSupportingSurface):
            return cls.free_standing_pose(location, world)
        free_pose = cls.free_pose_on_first_free_surface(
            world.get_semantic_annotations_by_type(location_type), held_object
        )
        if free_pose is None:
            raise NoFreeSurface(surface_type=location_type, placed_object=held_object)
        return free_pose

    @staticmethod
    def finding_action(
        searched_object: HasRootBody | None,
        searched_person: Human | None,
        context: StatechartContext,
    ) -> Action:
        """
        :param searched_object: The object the step looks for, if any.
        :param searched_person: The person the step looks for, if any.
        :param context: The context of the plan, which may let the robot detect people.
        :return: An action that drives to the object, else to the person.
        :raises NothingToFind: If the step looks for neither.
        """
        # TODO: FIND currently only a placeholder, only works with objects already in the SemDT, find action has
        #  to be implemented first to completely implement this feature
        if searched_object is not None:
            return NavigateAction(target_location=searched_object.root.global_pose)
        if searched_person is None:
            raise NothingToFind()
        person_detection = context.get_extension(PersonDetection)
        return FindPersonAction(
            person=searched_person,
            detection=None if person_detection is None else person_detection.query,
        )

    @staticmethod
    def find_person(
        description: str, location: SemanticAnnotation | None, world: World
    ) -> Human:
        """
        :param description: The person as the step describes them, such as ``"person
            pointing to the right"``.
        :param location: The place the step names; a room restricts the search to it.
        :param world: The world holding the person.
        :return: The first person in the named room, or in ``world`` if the step names
            no room. The description is left for the perception to check.
        :raises NoPersonFound: If no person is there.
        """
        room = location if isinstance(location, Room) else None
        people = (
            world.get_semantic_annotations_by_type(Human)
            if room is None
            else find_objects_in_room(Human, type(room), world)
        )
        if not people:
            raise NoPersonFound(description=description, room=room)
        return people[0]

    @staticmethod
    def find_object_annotation(
        object_name: str,
        action: ActionType,
        location: SemanticAnnotation | None,
        world: World,
    ) -> HasRootBody:
        """
        :param object_name: The name of the object as the step names it.
        :param action: The kind of action the step performs.
        :param location: The place the step names.
        :param world: The world holding the object.
        :return: The object the step names: the first of its kind in the room the step
            names, unless the step places it there; else the one with that name.
        :raises ObjectNotInRoom: If the room holds no object of that kind.
        """
        if not isinstance(location, Room) or action == ActionType.PLACE:
            return world.get_semantic_annotation_by_name(object_name)
        object_type = SemanticAnnotationClassFinder().find(object_name)
        objects_in_room = find_objects_in_room(object_type, type(location), world)
        if not objects_in_room:
            raise ObjectNotInRoom(object_type=object_type, room=location)
        return objects_in_room[0]

    @staticmethod
    def pick_up_from_reachable_standing_pose(
        graspable: HasGraspCandidates, context: StatechartContext
    ) -> ManipulationAtStandingPoseAction:
        """
        :param graspable: The object to pick up.
        :param context: The context of the plan, whose robot picks the object up with
            its first arm.
        :return: A pick-up that drives to the first standing pose the reachability
            costmap offers, and takes the object by the grasp closest to the robot
            standing there that the gripper opens wide enough for, touching anything
            while it does.
        :raises NoGraspFitsGripper: If every grasp is wider than the gripper opens.
        """
        arm = context.require_extension(RobotAccess).robot.all_arms[0]
        standing_pose = PlannedAction.standing_pose_on_floor(
            ReachabilityLocation(
                target_pose=Pose(reference_frame=graspable.root),
                arm=arm,
                context=context,
            ),
            context.world,
        )
        grasps_closest_first = GraspsByCloseness(
            graspable, standing_pose, number_of_grasps=None
        )()
        fitting_grasp = next(
            (
                grasp
                for grasp in grasps_closest_first
                if grasp.width_along_closing_axis
                <= arm.end_effector.separation_when_open
            ),
            None,
        )
        if fitting_grasp is None:
            raise NoGraspFitsGripper(graspable=graspable, gripper=arm.end_effector)
        return ManipulationAtStandingPoseAction(
            standing_pose=standing_pose,
            manipulated_location=Pose(reference_frame=graspable.root),
            manipulation=MoveAndPickUpAction.from_standing_position(
                standing_pose, fitting_grasp, arm
            ),
        )

    @staticmethod
    def place_from_reachable_standing_pose(
        held_object: HasGraspCandidates,
        target_location: Pose,
        context: StatechartContext,
    ) -> ManipulationAtStandingPoseAction:
        """
        :param held_object: The object to put down, held by the robot's first arm.
        :param target_location: Where to put the object down.
        :param context: The context of the plan, whose robot places the object.
        :return: A placing that carries the object to the first standing pose the
            reachability costmap offers for ``target_location`` and puts it down there,
            touching anything while it does.
        """
        arm = context.require_extension(RobotAccess).robot.all_arms[0]
        standing_pose = PlannedAction.standing_pose_on_floor(
            ReachabilityLocation(target_pose=target_location, arm=arm, context=context),
            context.world,
        )
        return ManipulationAtStandingPoseAction(
            standing_pose=standing_pose,
            manipulated_location=target_location,
            manipulation=MoveAndPlaceAction.from_standing_position(
                standing_pose, target_location, held_object, arm
            ),
        )

    @staticmethod
    def standing_pose_on_floor(location: ReachabilityLocation, world: World) -> Pose:
        """
        :param location: Where the robot can stand to reach its target.
        :param world: The world the robot stands in.
        :return: The first candidate of ``location`` that lies over a floor.
        :raises NoStandingPoseOnFloor: If no candidate lies over a floor.
        """
        floors = world.get_semantic_annotations_by_type(Floor)
        for candidate in location:
            if any(
                OverFloorPlan(pose=candidate, floor=floor.root)() for floor in floors
            ):
                return candidate
        raise NoStandingPoseOnFloor(target_pose=location.target_pose)

    @staticmethod
    def free_pose_on_table_in_room(
        room_type: type[Room], placed_object: HasRootBody, world: World
    ) -> Pose:
        """
        :param room_type: The kind of room to put the object down in.
        :param placed_object: The object to put down.
        :param world: The world the room is part of.
        :return: A pose on the first table in the room where the object fits, as
            :meth:`free_pose_on_first_free_surface` samples it.
        :raises NoFreeTableInRoom: If no table in the room has space for the object.
        """
        free_pose = PlannedAction.free_pose_on_first_free_surface(
            find_objects_in_room(Table, room_type, world), placed_object
        )
        if free_pose is None:
            raise NoFreeTableInRoom(room_type=room_type, placed_object=placed_object)
        return free_pose

    @staticmethod
    def free_pose_on_first_free_surface(
        surfaces: list[HasSupportingSurface], placed_object: HasRootBody
    ) -> Pose | None:
        """
        :param surfaces: The surfaces to put the object down on, in order of preference.
        :param placed_object: The object to put down.
        :return: The most likely sampled pose where ``placed_object`` fits without
            touching other objects, on the first of ``surfaces`` that has one; ``None``
            if none has.
        """
        for surface in surfaces:
            free_points = surface.sample_points_from_surface(
                body_to_sample_for=placed_object
            )
            if free_points:
                return Pose(
                    position=free_points[0],
                    reference_frame=free_points[0].reference_frame,
                )
        return None


@dataclass
class GeneratedPlan:
    """
    The actions that carry out one task of a command.
    """

    actions: list[PlannedAction]
    """
    The actions in the order the robot performs them.
    """

    def rewrap_into_command(
        self, instruction: str, any_room: set[type[Room]]
    ) -> Command:
        """
        :param instruction: The command as the operator spoke it.
        :param any_room: The kinds of room a step that names no room may be performed in.
        :return: This plan as a command whose steps each hold the consecutive actions
            performed in one kind of room; an action performed wherever the robot
            already is joins the step around it, or the first step if it leads.
        """
        steps: list[Step] = []
        leading_actions: list[Action | Match] = []
        for planned_action in self.actions:
            room_type = planned_action.room_type
            if steps and (room_type is None or steps[-1].rooms == {room_type}):
                steps[-1].actions.append(planned_action.action_type)
                continue
            if room_type is None:
                leading_actions.append(planned_action.action_type)
                continue
            steps.append(
                Step(
                    rooms={room_type},
                    actions=[*leading_actions, planned_action.action_type],
                )
            )
            leading_actions = []
        if leading_actions:
            steps.append(Step(rooms=any_room, actions=leading_actions))
        return Command(instruction=instruction, steps=steps)


@dataclass
class GeneratedCommand:
    """
    A spoken command together with the plans the command generator derived from it.
    """

    instruction: str
    """
    The command as the operator spoke it.
    """

    plans: list[GeneratedPlan]
    """
    One plan for each task of the command, in the order they were given.
    """

    @classmethod
    def from_json(
        cls, data: dict[str, Any], instruction: str, context: StatechartContext
    ) -> Self:
        """
        :param data: The command generator's JSON payload.
        :param instruction: The command as the operator spoke it.
        :param context: The context of the plan the command runs in.
        :return: The command it describes.
        """
        return cls(
            instruction=instruction,
            plans=[
                GeneratedPlan(
                    actions=[
                        PlannedAction.from_json(action, context) for action in plan
                    ]
                )
                for plan in data[PayloadKey.PLANS]
            ],
        )


def main(
    context: StatechartContext, instruction: str, json_plan: str
) -> GeneratedCommand:
    """
    Map a command generator's plans to the actions the robot performs, and log them.

    :param context: The context of the plan the command runs in.
    :param instruction: The command as the operator spoke it.
    :param json_plan: The command generator's JSON payload.
    :return: The command with its plans of actions.
    """
    command = GeneratedCommand.from_json(json.loads(json_plan), instruction, context)
    logger = logging.getLogger(__name__)
    for plan in command.plans:
        for action in plan.actions:
            logger.info(
                "%s | location: %s | object: %s | person: %s",
                type(action.action_type).__name__,
                None if action.location is None else type(action.location).__name__,
                None
                if action.object_annotation is None
                else type(action.object_annotation).__name__,
                None if action.person is None else action.person.name,
            )
            logger.info("%s", action.action_type)
    return command
