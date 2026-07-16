from __future__ import annotations

from dataclasses import dataclass
from typing_extensions import TYPE_CHECKING

from krrood.utils import DataclassException

if TYPE_CHECKING:
    from pycram.plans.designator import Designator
    from semantic_digital_twin.robots.abstract_robot import AbstractRobot


@dataclass
class ContextIsUnavailable(DataclassException):
    """
    Raised when an instance that tries to access the context of a plan has no reference to the plan.

    Most likely raised when an action created a subplan without calling `ActionDescription.add_subplan`
    """

    instance: Designator
    """
    The instance where the plan node is None.
    """

    def __post_init__(self) -> None:
        self.message = (
            f"{self.instance} has no plan node. Did you forget to call `add_subplan` when creating"
            f"plans inside actions?"
        )


@dataclass
class UncontrolledRobotBaseError(DataclassException):
    """
    Raised when a motion needs the robot base to respect collision avoidance,
    but the base is not linked to the world through a controlled connection, so
    collision rules discard every base contact as an ignorable adjacency.
    """

    robot: AbstractRobot
    """
    The robot whose base drive is not controlled.
    """

    def __post_init__(self) -> None:
        self.message = (
            f"The drive connection of {self.robot.name} has no hardware "
            f"interface, so collision avoidance ignores the base entirely. Set "
            f"has_hardware_interface = True on the drive connection."
        )
        super().__post_init__()
