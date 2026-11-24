"""
Exceptions raised when a generated plan cannot be mapped to robot actions.
"""

from __future__ import annotations

# %% Imports
from dataclasses import dataclass

from krrood.exceptions import DataclassException
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from krrood_spatial_math.spatial_types import Pose
    from challenges.GPSR.action_mapping.action_mapping import ActionType, PayloadKey
    from semantic_digital_twin.robots.robot_parts import EndEffector
    from semantic_digital_twin.semantic_annotations.mixins import (
        HasGraspCandidates,
        HasRootBody,
        HasSupportingSurface,
    )
    from semantic_digital_twin.semantic_annotations.semantic_annotations import Room
    from semantic_digital_twin.world_description.world_entity import (
        SemanticAnnotation,
    )


# %% Mapping actions


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


# %% Placing


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


# %% Picking up


@dataclass
class NoGraspFitsGripper(DataclassException):
    """
    Raised when every grasp an object offers is wider than the gripper opens.
    """

    graspable: HasGraspCandidates
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
