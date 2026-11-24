"""
Actions that combine moving the robot's base with looking at a target.
"""

from __future__ import annotations

from dataclasses import dataclass

from cramph.node import StatechartNode
from coraplex.robot_plans.actions.base import ParallelAction, SequentialAction
from coraplex.robot_plans.actions.core.navigation import (
    FaceAtAction,
    LookAtAction,
    LookAtWhileDrivingAction,
    NavigateAction,
)

# %% looking after turning


@dataclass(eq=False, repr=False, kw_only=True)
class FaceAndLookAtAction(SequentialAction):
    """
    Turns the robot's base towards a target, then looks at it.
    """

    face_at: FaceAtAction
    """
    The turn of the base towards the target.
    """

    look_at: LookAtAction
    """
    The look at the target once the base faces it.
    """

    def create_steps(self) -> list[StatechartNode]:
        return [self.face_at, self.look_at]


# %% looking while driving


@dataclass(eq=False, repr=False, kw_only=True)
class NavigateAndLookAtAction(ParallelAction):
    """
    Drives the robot's base to a pose while its camera keeps looking at a target.
    """

    navigate: NavigateAction
    """
    The drive.
    """

    look_at: LookAtWhileDrivingAction
    """
    The look at the target, held until the base arrived.
    """

    def create_steps(self) -> list[StatechartNode]:
        return [self.navigate, self.look_at]
