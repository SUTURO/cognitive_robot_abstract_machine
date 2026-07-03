from __future__ import annotations

from dataclasses import dataclass

from pycram.plans.factories import execute_single
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.motions.wiping import WipeTableMotion


@dataclass
class WipeAction(ActionDescription):
    """Wipe a table surface with the admittance-controlled :class:`WipeTableMotion`.

    A thin host so the wipe runs through the normal plan/executor path (which adds
    interrupt handling and, for the real robot, ships the chart to Giskard). The
    motion itself carries collision avoidance and the FT node, so nothing else
    needs assembling.
    """

    motion: WipeTableMotion

    def execute(self) -> None:
        self.add_subplan(execute_single(self.motion)).perform()
