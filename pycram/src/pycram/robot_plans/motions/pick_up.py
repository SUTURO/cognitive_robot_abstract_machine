from dataclasses import dataclass, field
from typing import Optional

from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.world_description.world_entity import Body
from .base import BaseMotion
from giskardpy.motion_statechart.goals.pick_up import PickUp, PullUp


@dataclass
class PickupMotion(BaseMotion):
    """
    High-level motion for picking up an object with a parallel gripper.

    This motion wraps the Giskard PickUp goal and exposes it to the
    motion framework via the `_motion_chart` property.
    """

    # The gripper that will execute the pickup (must be a ParallelGripper)
    manipulator: ParallelGripper = field(default=None, kw_only=True)

    # The world object that should be picked up
    object_geometry: Body = field(default=None, kw_only=True)

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
        print(f"Creating PickUp motion with {self.object_geometry}")
        pickup = PickUp(
            manipulator=self.manipulator,
            object_geometry=self.object_geometry,
            gripper_vertical=self.gripper_vertical,
        )
        return pickup


@dataclass
class PullUpMotion(BaseMotion):
    """
    High-level motion for picking up an object with a parallel gripper.

    This motion wraps the Giskard PickUp goal and exposes it to the
    motion framework via the `_motion_chart` property.
    """

    # The gripper that will execute the pickup (must be a ParallelGripper)
    manipulator: ParallelGripper = field(default=None, kw_only=True)

    # The world object that should be picked up
    object_geometry: Body = field(default=None, kw_only=True)

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
        print(f"Creating PickUp motion with {self.object_geometry}")
        pickup = PullUp(
            manipulator=self.manipulator, object_geometry=self.object_geometry
        )
        return pickup
