"""
Exceptions raised by plans and demonstrations.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass
from typing_extensions import TYPE_CHECKING

from krrood.exceptions import DataclassException
from coraplex.plans.failures import PlanFailure

if TYPE_CHECKING:
    from datetime import timedelta

    from coraplex.plans.executors import PlanExecutor
    from coraplex.robot_plans.actions.base import Action
    from coraplex.robot_plans.generated_commands import ActionType, PayloadKey
    from coraplex.robot_plans.task_definitions import Command
    from cramph.node import StatechartNode
    from krrood_spatial_math.spatial_types import Pose
    from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm, EndEffector
    from semantic_digital_twin.semantic_annotations.mixins import (
        HasRootBody,
        HasSupportingSurface,
        IsGraspable,
    )
    from semantic_digital_twin.semantic_annotations.semantic_annotations import Room
    from semantic_digital_twin.world_description.world_entity import (
        SemanticAnnotation,
    )


# %% plan execution
@dataclass
class PlanNotCompiled(DataclassException):
    """
    Raised when a plan executor is asked to execute before it compiled a plan.
    """

    def error_message(self) -> str:
        return "No plan was compiled to execute."

    def suggest_correction(self) -> str:
        return "call compile with the plan before calling execute."


@dataclass
class DebuggingWithoutRosNode(DataclassException):
    """
    A plan executor is created in debug mode without a ROS node to show the worlds its
    candidates are tried in.
    """

    def error_message(self) -> str:
        return "Debug mode is selected without a ROS node to publish through."

    def suggest_correction(self) -> str:
        return "Pass a ROS node to the executor, or leave debug mode off."


@dataclass
class CannotMatchOnType(DataclassException):
    """
    Raised when a plan transformation is bound to a type that is neither a plan node nor
    a designator, leaving no rule by which it could select the nodes it rewrites.
    """

    transformation: type
    """
    The transformation class that carries the binding.
    """

    matched_type: type
    """
    The type it is bound to.
    """

    def error_message(self) -> str:
        return (
            f"{self.transformation.__name__} is bound to {self.matched_type}, which is "
            f"neither a plan node nor a designator."
        )

    def suggest_correction(self) -> str:
        return "bind the transformation to a plan node type or a designator type"


@dataclass
class ClosingGoalNotFound(DataclassException):
    """
    Raised when the goal closing the container is asked for, but no such goal lies below
    the close action's node.
    """

    plan_node: StatechartNode
    """
    The node of the close action.
    """

    def error_message(self) -> str:
        return f"{self.plan_node} has no goal closing its container below it."

    def suggest_correction(self) -> str:
        return "ask for the closing goal only once the close action has been expanded"


@dataclass
class ReachHasNoFinalApproach(DataclassException):
    """
    Raised when the final approach of a reach is asked for, but no tool center point
    motion lies below the reach's node.
    """

    plan_node: StatechartNode
    """
    The node of the reach.
    """

    def error_message(self) -> str:
        return f"{self.plan_node} has no tool center point motion below it."

    def suggest_correction(self) -> str:
        return "ask for the final approach only once the reach has been expanded"


@dataclass
class GraspHasNoClosingGoal(DataclassException):
    """
    Raised when the goal a grasp closes its gripper with is asked for, but no goal that
    shuts the gripper lies below the grasp's node.
    """

    plan_node: StatechartNode
    """
    The node of the grasp.
    """

    def error_message(self) -> str:
        return f"{self.plan_node} has no goal closing the gripper below it."

    def suggest_correction(self) -> str:
        return "ask for the closing goal only once the grasp has been expanded"


@dataclass
class ToolPathNotFound(DataclassException):
    """
    Raised when the goal moving a tool along its path is asked for, but no such goal
    lies below the tool action's node.
    """

    plan_node: StatechartNode
    """
    The node of the tool action.
    """

    def error_message(self) -> str:
        return f"{self.plan_node} has no goal moving its tool along a path below it."

    def suggest_correction(self) -> str:
        return "ask for the tool path only once the tool action has been expanded"


@dataclass
class MissingWaypoints(DataclassException):
    """
    Raised when a waypoint motion or tool action produced no waypoints to follow.
    """

    instance: Action
    """
    The designator that has no waypoints.
    """

    def error_message(self) -> str:
        return f"{self.instance} has no waypoints to follow."

    def suggest_correction(self) -> str:
        return "ensure the motion sequence samples at least one point."


@dataclass
class WipingTargetMissing(DataclassException):
    """
    Raised when a wiping action is created without a surface to wipe.
    """

    instance: Action
    """
    The wiping action that has no target.
    """

    def error_message(self) -> str:
        return f"{self.instance} has neither a container nor a target pose."

    def suggest_correction(self) -> str:
        return "provide either a container body or a target pose to wipe."


@dataclass
class MissingToolFrame(DataclassException):
    """
    Raised when no tool frame is available for the requested arm.
    """

    arm: Arm
    """
    The arm whose tool frame was requested.
    """

    robot: AbstractRobot
    """
    The robot whose arm was searched.
    """

    def error_message(self) -> str:
        return f"no tool frame available for arm {self.arm} of {self.robot}"

    def suggest_correction(self) -> str:
        return "ensure the arm's end effector defines a tool frame."


@dataclass(kw_only=True)
class MotionDidNotFinish(PlanFailure):
    """
    Raised when a plan ended without succeeding.
    """

    unfinished_motions: list[StatechartNode]
    """
    The nodes that did not succeed, whether they failed, were interrupted or never
    ended.
    """

    def error_message(self) -> str:
        reports = ", ".join(
            f"{motion.unique_name} ({motion.life_cycle_state.name})"
            for motion in self.unfinished_motions
        )
        return f"Motion did not finish, following motions did not succeed: {reports}"

    def suggest_correction(self) -> str:
        return ""


@dataclass
class PerceptionException(DataclassException, ABC):
    """
    Represents a custom exception specific to perception-related errors.
    """


@dataclass
class PerceptionExceptionWithSemanticAnnotation(PerceptionException, ABC):
    """
    For PerceptionExceptions that name the annotation the perception was about.
    """

    semantic_annotation: type[SemanticAnnotation]
    """
    The annotation the perception was about.
    """


@dataclass
class PerceivedObjectNotInWorld(PerceptionExceptionWithSemanticAnnotation):
    """
    Raised when a detection names an object the world does not hold, so there is nothing
    to write the perceived pose to.
    """

    def error_message(self) -> str:
        return (
            f"The world holds no {self.semantic_annotation.__name__} the perceived pose "
            f"could be written to."
        )

    def suggest_correction(self) -> str:
        return (
            "spawn the object before detecting it, and annotate it with the semantic "
            "annotation that was queried."
        )


@dataclass
class AmbiguousDetection(PerceptionExceptionWithSemanticAnnotation):
    """
    Raised when a detection's annotation describes several bodies, so the perceived pose
    cannot be assigned to one of them.
    """

    body_count: int
    """
    How many distinct bodies the annotation described.
    """

    def error_message(self) -> str:
        return (
            f"{self.semantic_annotation.__name__} describes {self.body_count} bodies in "
            f"the world."
        )

    def suggest_correction(self) -> str:
        return "narrow the query's semantic annotation so it names a single object."


@dataclass
class UnidentifiedDetections(PerceptionExceptionWithSemanticAnnotation):
    """
    Raised when a perception source reports several candidates it cannot tell apart.

    A pipeline that localizes without classifying gives no way to choose between them,
    so the choice is refused rather than made arbitrarily.
    """

    candidate_count: int
    """
    How many indistinguishable candidates were reported.
    """

    def error_message(self) -> str:
        return (
            f"The perception source reported {self.candidate_count} candidates for "
            f"{self.semantic_annotation.__name__} and none of them carry a class label."
        )

    def suggest_correction(self) -> str:
        return (
            "add a classifying annotator to the perception pipeline, or narrow what is "
            "in view so a single object is reported."
        )


@dataclass
class PerceptionSourceUnavailable(PerceptionException):
    """
    Raised when the perception pipeline does not answer within the configured timeout.
    """

    action_name: str
    """
    The action the source was expected on.
    """

    def error_message(self) -> str:
        return f"No perception source is serving '{self.action_name}'."

    def suggest_correction(self) -> str:
        return "start the perception pipeline before running the plan."


@dataclass(kw_only=True)
class PerceptionWithoutPhysicalSimulation(PerceptionException):
    """
    Raised when a demonstration is to perceive through simulated cameras in a run
    without physics, where no camera is rendered.
    """

    executor_type: type[PlanExecutor]
    """
    The executor the run was set up with.
    """

    def error_message(self) -> str:
        return (
            f"A run of {self.executor_type.__name__} renders no camera to perceive "
            f"through."
        )

    def suggest_correction(self) -> str:
        return (
            "run the demonstration with a PhysicallySimulatedPlanExecutor, or leave its "
            "simulated perception unset."
        )


@dataclass
class NoFloorBelowRobot(DataclassException):
    """
    Raised when a robot that has to plan its way over a floor stands over none.
    """

    robot: AbstractRobot
    """
    The robot that stands over no floor.
    """

    def error_message(self) -> str:
        return f"'{self.robot.name}' does not stand over any annotated floor."

    def suggest_correction(self) -> str:
        return (
            "annotate the surface the robot drives on as a Floor, or move the robot "
            "onto one that is already annotated."
        )


@dataclass
class NotOnASingleLevelException(DataclassException):
    """
    Raised when an entity is detected to be on None or multiple levels at the same time.
    """

    message: str

    def error_message(self) -> str:
        return self.message

    def suggest_correction(self) -> str:
        return f"Move the robot to a recognized level"


# %% transformation by rooms


@dataclass
class MissingTasksOrRooms(DataclassException):
    """
    Raised when the steps of tasks are to be ordered into room visits, but there are no
    tasks or no rooms to perform their steps in.
    """

    commands: list[Command]
    """
    The tasks that were to be ordered.
    """

    rooms: list[type[Room]]
    """
    The rooms the steps were allowed to be performed in.
    """

    def error_message(self) -> str:
        if len(self.commands) == 0:
            return "No tasks were given, so there is nothing to plan room visits for."
        return (
            "No rooms were given, so none of the steps of the "
            f"{len(self.commands)} tasks can be performed anywhere."
        )

    def suggest_correction(self) -> str:
        if len(self.commands) == 0:
            return "Pass at least one task."
        return "Give every step of every task at least one room it may be performed in."


@dataclass
class NoRoomVisitOrderFound(DataclassException):
    """
    Raised when the solver finds no order of room visits that performs every step of the
    given tasks.
    """

    commands: list[Command]
    """
    The tasks that were to be ordered.
    """

    search_time_limit: float
    """
    How long the solver was allowed to search, in seconds.
    """

    def error_message(self) -> str:
        return (
            f"No order of room visits performs every step of the {len(self.commands)} "
            f"tasks within {self.search_time_limit} seconds."
        )

    def suggest_correction(self) -> str:
        return "Give every step at least one allowed room, or raise the time limit."


# %% driving with Pumas


@dataclass
class NavigationGoalNotHeard(DataclassException):
    """
    Nothing listens for navigation goals, so the Pumas navigation is not running.
    """

    topic: str
    """
    The topic the goal would have been published on.
    """

    def error_message(self) -> str:
        return f"Nothing subscribes to {self.topic}."

    def suggest_correction(self) -> str:
        return (
            "Start the Pumas navigation (~/start_pumas_live.sh) and run the demo in the "
            "environment of ~/ros_clean_env.sh, so both use the same RMW."
        )


@dataclass
class NavigationTimedOut(DataclassException):
    """
    The base did not reach the goal it was sent to in time.
    """

    goal: Pose
    """
    Where the base was sent to, in the map frame.
    """

    timeout: timedelta
    """
    How long the base was given.
    """

    def error_message(self) -> str:
        return (
            f"The base did not reach ({float(self.goal.x):.2f}, {float(self.goal.y):.2f}) within "
            f"{self.timeout.total_seconds()} s."
        )

    def suggest_correction(self) -> str:
        return (
            "Check that the path is free (the safety limiter stops for anything closer "
            "than 0.6 m), that AMCL is localized and that the robot clock is synced."
        )


# %% mapping generated commands: actions


@dataclass
class MissingActionArgument(DataclassException):
    """
    Raised when a planned action lacks an argument its kind of action needs.
    """

    action: ActionType
    """
    The kind of action that lacks the argument.
    """

    argument: PayloadKey
    """
    The key of the missing argument.
    """

    def error_message(self) -> str:
        return f"The '{self.action}' action has no '{self.argument}'."

    def suggest_correction(self) -> str:
        return (
            f"Make the command generator give every '{self.action}' action a "
            f"'{self.argument}' that names something in the world."
        )


@dataclass
class UnmappedActionType(DataclassException):
    """
    Raised when a planned action is of a kind the robot cannot perform.
    """

    action: str
    """
    The kind of action as the command generator named it.
    """

    def error_message(self) -> str:
        return f"The robot cannot perform '{self.action}' actions."

    def suggest_correction(self) -> str:
        return "Map this kind of action to robot actions, or leave it out of the plan."


@dataclass
class LocationNotInWorld(DataclassException):
    """
    Raised when a step names a kind of place the world holds none of.
    """

    location_type: type[SemanticAnnotation]
    """
    The kind of place the step names.
    """

    def error_message(self) -> str:
        return f"The world holds no {self.location_type.__name__}."

    def suggest_correction(self) -> str:
        return "Name a place the world holds, or add one of this kind to the world."


@dataclass
class LocationWithoutBody(DataclassException):
    """
    Raised when the robot is to stand next to a place that has no body.
    """

    location: SemanticAnnotation
    """
    The place without a body.
    """

    def error_message(self) -> str:
        return f"'{self.location.name}' has no body the robot could stand next to."

    def suggest_correction(self) -> str:
        return "Name a room or a piece of furniture as the location."


@dataclass
class NoFreeStandingPose(DataclassException):
    """
    Raised when no free spot for the robot to stand on is near a place.
    """

    location: SemanticAnnotation
    """
    The place the robot was to stand at.
    """

    def error_message(self) -> str:
        return f"No spot near '{self.location.name}' is free for the robot to stand on."

    def suggest_correction(self) -> str:
        return "Clear the space around the place, or name another location."


@dataclass
class ObjectNotGraspable(DataclassException):
    """
    Raised when a step is to pick up or put down an object the robot cannot grasp.
    """

    named_object: HasRootBody
    """
    The object the step names.
    """

    def error_message(self) -> str:
        return f"'{self.named_object.name}' offers no grasps to hold it by."

    def suggest_correction(self) -> str:
        return "Name an object the robot can carry."


@dataclass
class NothingToFind(DataclassException):
    """
    Raised when a finding step names neither an object nor a person.
    """

    def error_message(self) -> str:
        return "The 'find' action names neither an object nor a person."

    def suggest_correction(self) -> str:
        return (
            "Make the command generator give every 'find' action an 'object' or a "
            "'person'."
        )


@dataclass
class NoPersonFound(DataclassException):
    """
    Raised when a step looks for a person where there is none.
    """

    description: str
    """
    The person as the step describes them.
    """

    room: Room | None
    """
    The room the person was looked for in; ``None`` for the whole world.
    """

    def error_message(self) -> str:
        where = "the world" if self.room is None else f"the room '{self.room.name}'"
        return f"No person is in {where} to be '{self.description}'."

    def suggest_correction(self) -> str:
        return "Add the person to the world, or look for them in another room."


@dataclass
class ObjectNotInRoom(DataclassException):
    """
    Raised when a step names an object in a room that holds no object of that kind.
    """

    object_type: type[HasRootBody]
    """
    The kind of object the step names.
    """

    room: Room
    """
    The room the object was looked for in.
    """

    def error_message(self) -> str:
        return f"No {self.object_type.__name__} stands in the room '{self.room.name}'."

    def suggest_correction(self) -> str:
        return (
            f"Put a {self.object_type.__name__} into '{self.room.name}', or name the "
            f"room it is in."
        )


# %% mapping generated commands: placing


@dataclass
class NoFreeTableInRoom(DataclassException):
    """
    Raised when no table in a room has space to put an object down on.
    """

    room_type: type[Room]
    """
    The kind of room the object was to be put down in.
    """

    placed_object: HasRootBody
    """
    The object that was to be put down.
    """

    def error_message(self) -> str:
        return (
            f"No table in the {self.room_type.__name__} has space for "
            f"'{self.placed_object.name}'."
        )

    def suggest_correction(self) -> str:
        return (
            "Name a placing location with a surface, or clear or add a table in the "
            "room."
        )


@dataclass
class NoFreeSurface(DataclassException):
    """
    Raised when no surface of the named kind has space to put an object down on.
    """

    surface_type: type[HasSupportingSurface]
    """
    The kind of surface the object was to be put down on.
    """

    placed_object: HasRootBody
    """
    The object that was to be put down.
    """

    def error_message(self) -> str:
        return (
            f"No {self.surface_type.__name__} has space for "
            f"'{self.placed_object.name}'."
        )

    def suggest_correction(self) -> str:
        return "Clear a surface of that kind, or name another placing location."


@dataclass
class NoStandingPoseOnFloor(DataclassException):
    """
    Raised when the robot can reach a target only from poses that lie over no floor.
    """

    target_pose: Pose
    """
    The pose the robot was to reach.
    """

    def error_message(self) -> str:
        return f"No pose over a floor lets the robot reach {self.target_pose}."

    def suggest_correction(self) -> str:
        return "Choose a target the robot can reach from inside the building."


# %% mapping generated commands: picking up


@dataclass
class NoGraspFitsGripper(DataclassException):
    """
    Raised when every grasp an object offers is wider than the gripper opens.
    """

    graspable: IsGraspable
    """
    The object none of whose grasps fits.
    """

    gripper: EndEffector
    """
    The gripper that was to take hold of it.
    """

    def error_message(self) -> str:
        return (
            f"No grasp of '{self.graspable.name}' fits {self.gripper.name}, which opens "
            f"only {self.gripper.separation_when_open:.3f} m wide."
        )

    def suggest_correction(self) -> str:
        return "Pick up a narrower object, or use a robot whose gripper opens wider."
