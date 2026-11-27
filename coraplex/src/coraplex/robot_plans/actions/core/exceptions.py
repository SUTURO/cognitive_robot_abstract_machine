"""
Errors the core robot actions raise when they are given something they cannot carry out.
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import TYPE_CHECKING

from coraplex.plans.failures import PlanFailure
from krrood.exceptions import DataclassException

if TYPE_CHECKING:
    from coraplex.robot_plans.actions.core.misc import DetectAction
    from krrood_spatial_math.spatial_types import Pose2D
    from semantic_digital_twin.robots.robot_part_mixins import HasTwoFingers

# %% detecting


@dataclass
class DetectionTargetMissing(DataclassException):
    """
    Raised when a detection is given neither the type of object to look for nor the box
    to look in.
    """

    instance: DetectAction
    """
    The detection that has no target.
    """

    def error_message(self) -> str:
        return f"{self.instance} has neither an object type nor a detection box."

    def suggest_correction(self) -> str:
        return "provide the type of object to detect, a box to detect in, or both."


# %% sending the fingers to a width


@dataclass
class WidthBeyondGripperOpening(DataclassException):
    """
    Raised when the fingers of a gripper are sent to a width wider than the gripper
    opens.
    """

    gripper: HasTwoFingers
    """
    The gripper whose fingers were sent.
    """

    width: float
    """
    How far apart the fingertips' facing surfaces were sent to stand, in metres.
    """

    def error_message(self) -> str:
        return (
            f"{self.gripper.name} opens only {self.gripper.separation_when_open:.3f} m "
            f"wide, but its fingers were sent {self.width:.3f} m apart."
        )

    def suggest_correction(self) -> str:
        return (
            "grasp the object across a side narrower than the gripper opens, or with a "
            "gripper that opens wider."
        )


# %% driving along a planned path


@dataclass(kw_only=True)
class NavigationGoalOutsideFreeSpace(PlanFailure):
    """
    Raised when a drive along a planned path is sent to a goal its base cannot stand on,
    because furniture or a wall is too close to it or no floor lies below it.
    """

    goal: Pose2D
    """
    The goal the drive was sent to.
    """

    def error_message(self) -> str:
        """
        :return: Where the goal lies, in place of the generic message of
            :meth:`~coraplex.plans.failures.PlanFailure.error_message`.
        """
        return (
            f"The navigation goal at {self.goal.position} lies outside the free space "
            f"the robot's base can stand in."
        )

    def suggest_correction(self) -> str:
        """
        :return: How to choose a goal the base can stand at, where
            :meth:`~coraplex.plans.failures.PlanFailure.suggest_correction` suggests
            nothing.
        """
        return (
            "choose a goal farther from furniture and walls, or a smaller obstacle "
            "clearance."
        )
