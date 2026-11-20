from __future__ import annotations

# %% Imports
import json

import numpy
from dataclasses import dataclass
from enum import StrEnum

from typing_extensions import Any, Self

from challenges.GPSR.action_mapping.exceptions import (
    MissingActionArgument,
    NoFreeSurface,
    NoFreeTableInRoom,
    NoGraspFitsGripper,
    NoStandingPoseOnFloor,
    ObjectNotInRoom,
)
from challenges.semdt_helper_functions.find_class_by_name import (
    SemanticAnnotationClassFinder,
)


from coraplex.locations.locations import ReachabilityLocation
from coraplex.perception import RoboKudoQuery, RoboKudoQueryTask
from coraplex.plans.context_extensions import RobotAccess, StatementGrounding
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.base import Action, SequentialAction
from coraplex.robot_plans.actions.composite.transporting import (
    MoveAndPickUpAction,
    MoveAndPlaceAction,
)
from coraplex.robot_plans.actions.composite.searching import SearchAction
from coraplex.robot_plans.actions.core.navigation import (
    FaceAtAction,
    NavigateAction,
    PathPlanningNavigateAction,
)
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.talking import TalkingAction
from coraplex.robot_plans.task_definitions import Command, Step
from cramph.context import ContextExtension, StatechartContext
from cramph.node import StatechartNode
from krrood.entity_query_language.factories import a
from krrood.entity_query_language.query.match import Match
from krrood_spatial_math.spatial_types import Point3, Pose
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import AllowAllCollisions
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.reasoning.predicates import OverFloorPlan
from semantic_digital_twin.reasoning.queries import (
    find_free_space_in_room,
    find_free_space_next_to,
    find_objects_in_room,
    find_objects_room,
)
from semantic_digital_twin.robots.robot_parts import AbstractRobot
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

    action_type: Action | Match | None
    """
    What the robot does in this step, as an action or as a statement grounded when the
    step starts; ``None`` for a kind of step that is not mapped yet.
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

    person: str | None = None
    """
    The description of the person the step looks for.
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
        """
        world = context.world
        action = None
        semdt_finder = SemanticAnnotationClassFinder()
        if data.get(PayloadKey.LOCATION) is not None:
            semdt_type = semdt_finder.find_place(data.get(PayloadKey.LOCATION))
            print(semdt_type)
            location = world.get_semantic_annotations_by_type(semdt_type)[0]
        else:
            location = None

        # -- LOCATION ---------------------------------------------------------------------------------------------------
        location_coordinates = None
        if location is not None:
            if isinstance(location, Room):
                location_coordinates = find_free_space_in_room(semdt_type, world)
            else:
                location_coordinates = find_free_space_next_to(
                    robot_diameter=0.7, semantic_annotation=location
                )

        # -- OBJECT -----------------------------------------------------------------------------------------------------
        searched_object = (
            None
            if data.get(PayloadKey.OBJECT) is None
            else cls.find_object_annotation(
                data[PayloadKey.OBJECT], data[PayloadKey.ACTION], location, world
            )
        )

        # -- PERSON -----------------------------------------------------------------------------------------------------
        searched_person = None
        if data.get(PayloadKey.PERSON) is not None:
            searched_person = world.get_semantic_annotation_by_name(
                data.get(PayloadKey.PERSON)
            )

        match data[PayloadKey.ACTION]:
            case ActionType.SAY:
                action = TalkingAction(speaking_text=data.get(PayloadKey.TEXT))
            case ActionType.NAVIGATE:  # DONE
                if location_coordinates is None:
                    raise MissingActionArgument(
                        action=ActionType.NAVIGATE, argument=PayloadKey.LOCATION
                    )
                action = a(PathPlanningNavigateAction)(target=location_coordinates)
            case ActionType.PICK_UP:
                action = cls.pick_up_from_reachable_standing_pose(
                    searched_object, context
                )
            case ActionType.PLACE:
                if searched_object is None:
                    raise MissingActionArgument(
                        action=ActionType.PLACE, argument=PayloadKey.OBJECT
                    )
                if location_coordinates is None:
                    raise MissingActionArgument(
                        action=ActionType.PLACE, argument=PayloadKey.LOCATION
                    )
                if isinstance(location, Room):
                    target_location = cls.free_pose_on_table_in_room(
                        semdt_type, searched_object, world
                    )
                elif isinstance(location, HasSupportingSurface):
                    target_location = cls.free_pose_on_first_free_surface(
                        world.get_semantic_annotations_by_type(semdt_type),
                        searched_object,
                    )
                    if target_location is None:
                        raise NoFreeSurface(
                            surface_type=semdt_type, placed_object=searched_object
                        )
                else:
                    target_location = location_coordinates
                action = cls.place_from_reachable_standing_pose(
                    searched_object, target_location, context
                )
            case ActionType.FIND:
                if searched_object is not None:
                    action = NavigateAction(
                        target_location=searched_object.root.global_pose
                    )
                elif searched_person is not None:
                    person_detection = context.get_extension(PersonDetection)
                    action = FindPersonAction(
                        person=searched_person,
                        detection=(
                            None if person_detection is None else person_detection.query
                        ),
                    )
                else:
                    raise Exception("Nothing to find here.")
            case ActionType.OPEN:
                pass
            case _:
                action = None

        # TODO change from strings to semantic annotations
        return cls(
            action_type=action,
            text=data.get(PayloadKey.TEXT),
            location=location,
            object_annotation=searched_object,
            person=data.get(PayloadKey.PERSON),
        )

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
        leading_actions: list[Action | Match | None] = []
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


def main(context: StatechartContext, json_plan: str) -> GeneratedCommand:
    instruction = "Bring the cup from the kitchen to the living room. Ask what the person in the kitchen what their favorite drink is."
    #     json_ms0_mock = """
    # {"plans":[[{"action":"say","text":"I will pick up the milk from the kitchen and put it on the table"},{"action":"navigate","location":"kitchen"},{"action":"pick_up","object":"milk"},{"action":"navigate","location":"table"},{"action":"place","object":"milk","location":"table"}],
    # [{"action":"say","text":"I will find the person in the kitchen and ask what their favorite drink is"},{"action":"navigate","location":"kitchen"},{"action":"find","person":"person"},{"action":"say","text":"What is your favorite drink?"}]
    # ]}
    #     """
    #     json_string = """
    # {
    #   "plans": [
    #     [
    #       { "action": "say", "text": "I will pick up butter from the storage rack and put it on the laundry room trashcan" },
    #       { "action": "navigate", "location": "storage rack" },
    #       { "action": "pick_up", "object": "butter" },
    #       { "action": "navigate", "location": "laundry room trashcan" },
    #       { "action": "place", "object": "butter", "location": "laundry room trashcan" }
    #     ],
    #     [
    #       { "action": "say", "text": "I will go to the book shelf, find the person pointing to the right and tell them my team's name" },
    #       { "action": "navigate", "location": "book shelf" },
    #       { "action": "find", "person": "person pointing to the right" },
    #       { "action": "say", "text": "<team name>" }
    #     ],
    #     [
    #       { "action": "say", "text": "I will open the cupboard" },
    #       { "action": "navigate", "location": "cupboard" },
    #       { "action": "open", "location": "cupboard" }
    #     ]
    #   ]
    # }
    # """
    command = GeneratedCommand.from_json(json.loads(json_plan), instruction, context)
    for plan in command.plans:
        for action in plan.actions:
            print(
                "  ",
                type(action.action_type).__name__,
                "| location:",
                None if action.location is None else type(action.location).__name__,
                "| object:",
                None
                if action.object_annotation is None
                else type(action.object_annotation).__name__,
                "| person:",
                action.person,
            )
            print(action.action_type)
    return command


if __name__ == "__main__":
    main()
