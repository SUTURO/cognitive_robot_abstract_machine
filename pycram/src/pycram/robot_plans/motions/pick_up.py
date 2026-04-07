from dataclasses import dataclass, field
from typing import Optional

from semantic_digital_twin.robots.abstract_robot import ParallelGripper, Manipulator
from semantic_digital_twin.world_description.world_entity import Body
from .base import BaseMotion
from giskardpy.motion_statechart.goals.pick_up import PickUp, PullUp, BoxGraspMagic


# todo docs and parameter description and why do u have the simulation param? and why is it called grasp magic
@dataclass
class PickupMotion(BaseMotion):
    """
    High-level motion for picking up an object with a parallel gripper.

    This motion wraps the Giskard PickUp goal and exposes it to the
    motion framework via the `_motion_chart` property.
    """

    # The gripper that will execute the pickup (must be a ParallelGripper)
    manipulator: Manipulator = field(default=None, kw_only=True)

    # The world object that should be picked up
    object_geometry: Body = field(default=None, kw_only=True)

    simulated: bool = field(default=True, kw_only=True)
    """
    Parsing simulation argument
    """

    # If True, the gripper is kept vertically aligned during the grasp
    # kw_only=True forces this to be passed as a keyword argument
    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)

    def perform(self):
        return

    @property
    def _motion_chart(self):
        """
        Creates and returns the underlying Giskard PickUp goal.

        The motion framework queries this property to insert the task
        into the MotionStatechart.
        """
        grasp_magic = BoxGraspMagic(
            manipulator=self.manipulator,
            object_geometry=self.object_geometry,
            gripper_vertical=self.gripper_vertical,
        )
        pickup = PickUp(
            simulated_execution=self.simulated,
            grasp_magic=grasp_magic,
        )
        return pickup
