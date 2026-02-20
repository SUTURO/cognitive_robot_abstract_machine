from dataclasses import dataclass

import numpy

from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.pointing import Pointing

from .base import BaseMotion
from ...datastructures.pose import PoseStamped


@dataclass
class MoveMotion(BaseMotion):
    """
    Moves the robot to a designated location
    """

    target: PoseStamped
    """
    Location to which the robot should be moved
    """

    keep_joint_states: bool = False
    """
    Keep the joint states of the robot during/at the end of the motion
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):

        cp = CartesianPose(
            root_link=self.world.root,
            tip_link=self.robot_view.root,
            goal_pose=self.target.to_spatial_type(),
        )
        import numpy as np

        # Extract translation (last column of homogeneous transform)
        tip_pos = np.array(cp.tip_link.global_pose[:3, 3])  # tip x,y,z
        goal_pos = np.array(cp.goal_pose[:3, 3])  # goal x,y,z

        dist = np.linalg.norm(goal_pos - tip_pos)
        print("Tip translation:", tip_pos)
        print("Goal translation:", goal_pos)
        print("Distance to goal:", dist)

        return cp
