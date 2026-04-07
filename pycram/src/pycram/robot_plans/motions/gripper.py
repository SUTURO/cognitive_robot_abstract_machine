from dataclasses import dataclass, field
from typing import Optional, List

from giskardpy.motion_statechart.goals.pick_up import PullUp
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianPosition,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body
from pycram.robot_plans.motions.base import BaseMotion
from pycram.datastructures.enums import (
    Arms,
    MovementType,
    WaypointsMovementType,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.view_manager import ViewManager
from pycram.utils import translate_pose_along_local_axis


@dataclass
class ReachMotion(BaseMotion):
    """ """

    object_designator: Body
    """
    Object designator_description describing the object that should be picked up
    """
    arm: Arms
    """
    The arm that should be used for pick up
    """
    grasp_description: GraspDescription
    """
    The grasp description that should be used for picking up the object
    """
    movement_type: MovementType = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """
    reverse_pose_sequence: bool = False
    """
    Reverses the sequence of poses, i.e., moves away from the object instead of towards it. Used for placing objects.
    """

    def _calculate_pose_sequence(self) -> List[Pose]:
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot_view)

        target_pose = GraspDescription.get_grasp_pose(
            self.grasp_description, end_effector, self.object_designator
        )
        target_pose.rotate_by_quaternion(
            GraspDescription.calculate_grasp_orientation(
                self.grasp_description,
                end_effector.front_facing_orientation.to_np(),
            )
        )
        target_pre_pose = translate_pose_along_local_axis(
            target_pose,
            end_effector.front_facing_axis.to_np()[:3],
            -0.05,  # TODO: Maybe put these values in the semantic annotates
        )

        pose = self.world.transform(target_pre_pose, self.world.root)

        sequence = [target_pre_pose, pose]
        return sequence.reverse() if self.reverse_pose_sequence else sequence

    def perform(self):
        pass

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        nodes = [
            CartesianPose(
                root_link=self.robot_view.root,
                tip_link=tip,
                goal_pose=pose,
                threshold=0.005,
                name="Reach",
            )
            for pose in self._calculate_pose_sequence()
        ]
        return Sequence(nodes=nodes)


@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: GripperState
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: Arms
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        arm = ViewManager().get_end_effector_view(self.gripper, self.robot)

        return JointPositionList(
            goal_state=arm.get_joint_state_by_type(self.motion),
            name=(
                "OpenGripper" if self.motion == GripperState.OPEN else "CloseGripper"
            ),
        )


@dataclass
class MoveToolCenterPointMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: Pose
    """
    Target pose to which the TCP should be moved
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: Optional[MovementType] = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot).tool_frame
        root = self.world.root if self.robot.full_body_controlled else self.robot.root
        task = None
        if self.movement_type == MovementType.TRANSLATION:
            task = CartesianPosition(
                root_link=root,
                tip_link=tip,
                goal_point=self.target.to_position(),
                name="MoveTCP",
            )
        else:
            task = CartesianPose(
                root_link=root,
                tip_link=tip,
                goal_pose=self.target,
                name="MoveTCP",
            )
        return task


@dataclass
class MoveTCPWaypointsMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    waypoints: List[Pose]
    """
    Waypoints the TCP should move along 
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: WaypointsMovementType = (
        WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT
    )
    """
    The type of movement that should be performed.
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot).tool_frame
        root = self.world.root if self.robot.full_body_controlled else self.robot.root
        nodes = [
            CartesianPose(
                root_link=root,
                tip_link=tip,
                goal_pose=pose,
                # threshold=0.005,
            )
            for pose in self.waypoints
        ]
        return Sequence(nodes=nodes)


# todo docs and parameter description and why do u have the simulation param?
@dataclass
class PullUpMotion(BaseMotion):
    """
    High-level motion for pulling up an object with a parallel gripper.

    This motion wraps the Giskard PullUp goal and exposes it to the
    motion framework via the `_motion_chart` property.
    """

    # The gripper that will execute the pullUp (must be a ParallelGripper)
    manipulator: Manipulator = field(default=None, kw_only=True)

    # The world object that should be pulledUp
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
        pickup = PullUp(
            manipulator=self.manipulator, object_geometry=self.object_geometry
        )
        return pickup


#
# @dataclass
# class PlaceMotion(BaseMotion):
#     """
#     Motion for placing an object, i.e., moving the gripper to a certain pose
#     It creates a _motion_chart that is used by the motion framework
#     It directly calls the implemented PickUp of Giskard.
#     """
#
#     gripper: Manipulator = field(kw_only=True)
#     """
#     Name of the gripper that should be moved
#     """
#
#     object_designator: Body = field(kw_only=True)
#     """
#     Object designator_description describing the object that should be placed
#     """
#     goal_pose: HomogeneousTransformationMatrix | Point3 = field(kw_only=True)
#     """
#     The goal_pose at which the object should be placed
#     """
#
#     simulated: bool = field(default=True, kw_only=True)
#     """
#     Parsing simulation argument
#     """
#
#     allow_gripper_collision: Optional[bool] = None
#     """
#     If the gripper is allowed to collide with something
#     """
#
#     def perform(self):
#         return
#
#     @property
#     def _motion_chart(self):
#         goal_pose = self.goal_pose
#
#         return Place(
#             manipulator=self.gripper,
#             object_geometry=self.object_designator,
#             goal=goal_pose,
#             simulated=self.simulated,
#         )
#
#
# @dataclass
# class RetractMotion(BaseMotion):
#     """
#     Motion for placing an object, i.e., moving the gripper to a certain pose
#     It creates a _motion_chart that is used by the motion framework
#     It directly calls the implemented PickUp of Giskard.
#     """
#
#     gripper: Manipulator = field(kw_only=True)
#     """
#     Name of the gripper that should be moved
#     """
#     simulated: bool = field(default=True, kw_only=True)
#     """
#     Parsing simulation argument
#     """
#
#     def perform(self):
#         return
#
#     @property
#     def _motion_chart(self):
#         return Retracting(
#             manipulator=self.gripper,
#         )
#
#
# # TODO currently still missing the class that just sits within the Pickup of Giskard
# @dataclass
# class GiskardMoveGripperMotion(BaseMotion):
#     """
#     Opens or closes the gripper
#     """
#
#     motion: GripperState
#     """
#     Motion that should be performed, either 'open' or 'close'
#     """
#     simulated: bool = True
#     """
#     Parsing simulation argument
#     """
#
#     def perform(self):
#         return
#
#     @property
#     def _motion_chart(self):
#         if self.motion == GripperState.OPEN:
#             return OpenHand(simulated_execution=self.simulated)
#         if self.motion == GripperState.CLOSE:
#             return CloseHand(simulated_execution=self.simulated)
#         else:
#             raise ValueError(f"Unknown motion {self.motion}")
