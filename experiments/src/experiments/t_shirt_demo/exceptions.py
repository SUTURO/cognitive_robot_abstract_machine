"""
Exceptions of the T-shirt demonstration.
"""

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException

# %% grasping


@dataclass
class TShirtNotPinned(DataclassException):
    """
    Raised when the gripper closed with no vertex of the T-shirt close enough to it to
    hold.
    """

    t_shirt_name: str
    """
    The name of the T-shirt that was to be held.
    """

    holding_body_name: str
    """
    The name of the body that was to hold the T-shirt.
    """

    reason: str
    """
    What the simulator reported when asked to pin the T-shirt.
    """

    def error_message(self) -> str:
        return (
            f"{self.holding_body_name} holds no part of {self.t_shirt_name}: "
            f"{self.reason}"
        )

    def suggest_correction(self) -> str:
        return (
            "Bring the gripper closer to the T-shirt before closing it, or widen the "
            "radius the T-shirt is pinned within."
        )


@dataclass
class NoTShirtWithinReach(DataclassException):
    """
    Raised when no part of the T-shirt lies on the table where the robot can reach down
    to take it.
    """

    t_shirt_name: str
    """
    The name of the T-shirt that was to be taken.
    """

    reach_into_table: float
    """
    How far behind the near edge of the table the robot reaches, in metres.
    """

    lateral_reach: float
    """
    How far to either side of the robot it reaches, in metres.
    """

    def error_message(self) -> str:
        return (
            f"No part of {self.t_shirt_name} lies within {self.reach_into_table} m "
            f"behind the near edge of the table and {self.lateral_reach} m to either "
            f"side of the robot."
        )

    def suggest_correction(self) -> str:
        return (
            "Let the T-shirt fall closer to the near edge of the table, or let the robot "
            "reach further."
        )
