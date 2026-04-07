from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import timedelta

from typing_extensions import Optional, Any

from pycram.datastructures.enums import (
    Arms,
    MovementType,
    FindBodyInRegionMethod,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.plans.factories import sequential, execute_single
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from pycram.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


@dataclass
class ReachAction(ActionDescription):
    """
    Let the robot reach a specific pose.
    """

    target_pose: Pose
    """
    Pose that should be reached.
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp_description: GraspDescription
    """
    The grasp description that should be used for picking up the object
    """

    object_designator: Body = None
    """
    Object designator_description describing the object that should be picked up
    """

    reverse_reach_order: bool = False

    def execute(self) -> None:

        target_pre_pose, target_pose, _ = self.grasp_description._pose_sequence(
            self.target_pose, self.object_designator, reverse=self.reverse_reach_order
        )
        self.add_subplan(
            sequential(
                children=[
                    MoveToolCenterPointMotion(
                        target_pre_pose, self.arm, allow_gripper_collision=False
                    ),
                    MoveToolCenterPointMotion(
                        target_pose,
                        self.arm,
                        allow_gripper_collision=False,
                        movement_type=MovementType.CARTESIAN,
                    ),
                ]
            )
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if object is contained in the gripper such that it can be grasped and picked up.
        """
        fingers_link_names = self.arm_chain.end_effector.fingers_link_names
        if fingers_link_names:
            if not is_body_between_fingers(
                self.object_designator,
                fingers_link_names,
                method=FindBodyInRegionMethod.MultiRay,
            ):
                raise ObjectNotInGraspingArea(
                    self.object_designator,
                    World.robot,
                    self.arm,
                    self.grasp_description,
                )
        else:
            logger.warning(
                f"Cannot validate reaching to pick up action for arm {self.arm} as no finger links are defined."
            )


@dataclass
class PickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

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
    The GraspDescription that should be used for picking up the object
    """

    def execute(self) -> None:
        self.add_subplan(
            sequential(
                children=[
                    MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm),
                    ReachAction(
                        target_pose=self.object_designator.global_pose,
                        object_designator=self.object_designator,
                        arm=self.arm,
                        grasp_description=self.grasp_description,
                    ),
                    MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm),
                ]
            )
        ).perform()

        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot)

        # Attach the object to the end effector
        with self.world.modify_world():
            self.world.move_branch_with_fixed_connection(
                self.object_designator, end_effector.tool_frame
            )

        _, _, lift_to_pose = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )
        self.add_subplan(
            execute_single(
                MoveToolCenterPointMotion(
                    lift_to_pose,
                    self.arm,
                    allow_gripper_collision=True,
                    movement_type=MovementType.TRANSLATION,
                )
            )
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if picked up object is in contact with the gripper.
        """
        if not has_gripper_grasped_body(self.arm, self.object_designator):
            raise ObjectNotGraspedError(
                self.object_designator, World.robot, self.arm, self.grasp_description
            )


@dataclass
class GraspingAction(ActionDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    object_designator: Body
    """
    Object Designator for the object that should be grasped
    """
    arm: Arms
    """
    The arm that should be used to grasp
    """
    grasp_description: GraspDescription
    """
    The distance in meters the gripper should be at before grasping the object
    """

    def execute(self) -> None:
        pre_pose, grasp_pose, _ = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )

        self.add_subplan(
            sequential(
                [
                    MoveToolCenterPointMotion(pre_pose, self.arm),
                    MoveGripperMotion(GripperState.OPEN, self.arm),
                    MoveToolCenterPointMotion(
                        grasp_pose, self.arm, allow_gripper_collision=True
                    ),
                    MoveGripperMotion(
                        GripperState.CLOSE, self.arm, allow_gripper_collision=True
                    ),
                ]
            )
        ).perform()


@dataclass
class GiskardPickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

    simulated: bool = field(default=True, kw_only=True)
    """
    Parsing simulation argument
    """

    object_designator: Body = field(default=None, kw_only=True)
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms = field(default=Arms.LEFT, kw_only=True)
    """
    arms that should be used for pick up
    """

    gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
    """
    If True, the gripper is kept vertically aligned during the grasp
    kw_only=True forces this to be passed as a keyword argument
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> bool:
        try:
            from ...motions.pick_up import PickupMotion
            from ... import GiskardRetractActionDescription, ParkArmsActionDescription
            from pycram.robot_plans.motions.navigation import MoveMotion
            from pycram.external_interfaces import nav2_move

        except ImportError:
            raise ImportError(
                "The GiskardPickUpAction requires Giskardpy_ros, not only giskardpy."
            )

        # Register attach as a post-perform callback BEFORE queuing the motion
        robot_pose_pre_manipulation = PoseStamped.from_spatial_type(
            self.context.robot.root.global_pose
        )

        grasped: bool = False

        # try to grasp the object, if it is not grasped, throw an ObjectNotGraspedError so one can react within the demo
        try:
            SequentialPlan(
                self.context,
                GiskardGraspActionDescription(
                    simulated=self.simulated,
                    arm=self.arm,
                    object_designator=self.object_designator,
                    gripper_vertical=self.gripper_vertical,
                ),
            ).perform()
            #
            # if not self.validate_grasped():
            #     print("object has not been grasped")
            #     raise ObjectNotGraspedError(
            #         obj=self.object_designator, robot=self.context.robot, arm=self.arm
            #     )
            grasped = True
        except Exception as e:
            SequentialPlan(
                self.context,
                GiskardRetractActionDescription(
                    simulated=self.simulated,
                    arm=Arms.LEFT,
                    back_off_pose=robot_pose_pre_manipulation,
                ),
                ParkArmsActionDescription(Arms.BOTH),
            ).perform()
            logger.error(f"Internal PickUpError with error message: {e}")
            grasped = False
            return grasped

        SequentialPlan(
            self.context,
            GiskardPullUpActionDescription(
                simulated=self.simulated,
                arm=self.arm,
                object_designator=self.object_designator,
            ),
        ).perform()

        from pycram.external_interfaces import nav2_move

        os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
        goal = robot_pose_pre_manipulation.ros_message()
        print(f"Moving to {robot_pose_pre_manipulation}'")
        nav2_move.start_nav_to_pose(goal)

        SequentialPlan(self.context, ParkArmsActionDescription(Arms.BOTH)).perform()

        return grasped

    # implement sometime, currently not implemented, since Motions have weird heirachys
    def item_between_fingertips(
        self,
        fingertip_distance: float,
        closed_value: float = -0.1007,
        open_value: float = 0.1338,
        threshhold: float = 0.05,
    ) -> bool:
        """
        Returns True if the gripper is not fully closed and not fully open,
        which can indicate that an item is between the fingertips.

        Args:
            fingertip_distance: Current value from /gripper_command/fingertip_distance
            closed_value: Typical fully closed value
            open_value: Typical fully open value
            threshhold: Tolerance around the reference values

        Returns:
            True if the distance suggests an object is between the fingertips.
        """
        closed_min = closed_value - threshhold
        closed_max = closed_value + threshhold
        open_min = open_value - threshhold
        open_max = open_value + threshhold

        is_closed = closed_min <= fingertip_distance <= closed_max
        is_open = open_min <= fingertip_distance <= open_max

        # Object likely present if it is neither clearly open nor clearly closed
        return not is_closed and not is_open

    def validate_grasped(self):
        node = rclpy.create_node("fingertip_distance_subscriber")
        msg = None

        def callback(data: None):
            nonlocal msg
            msg = data

        # TODO change msg time, idk what msg type it is
        subscription = node.create_subscription(
            msg_type=None,
            topic="/gripper_command/fingertip_distance",
            callback=callback,
            qos_profile=10,
        )

        while msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        logger.info(f"Gripper fingertip distance: {msg.data}")
        node.destroy_node()

        is_object_between_fingertips: bool = self.item_between_fingertips(
            fingertip_distance=msg.smth
        )
        if not is_object_between_fingertips:
            raise ObjectNotGraspedError(
                obj=self.object_designator, robot=self.context.robot, arm=self.arm
            )

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        arm: Union[Iterable[Arms], Arms] = None,
        gripper_vertical: Union[Iterable[bool], bool] = True,
        simulated: bool = True,
    ) -> PartialDesignator[GiskardPickUpAction]:
        return PartialDesignator[GiskardPickUpAction](
            GiskardPickUpAction,
            object_designator=object_designator,
            arm=arm,
            simulated=simulated,
            gripper_vertical=gripper_vertical,
        )


# todo fix
#
# @dataclass
# class GiskardGraspAction(ActionDescription):
#     """
#     Let the robot pick up an object.
#     """
#
#     simulated: bool = field(default=True, kw_only=True)
#     """
#     Parsing simulation argument
#     """
#
#     object_designator: Body = field(default=None, kw_only=True)
#     """
#     Object designator_description describing the object that should be picked up
#     """
#
#     arm: Arms = field(default=Arms.LEFT, kw_only=True)
#     """
#     arms that should be used for pick up
#     """
#
#     gripper_vertical: Optional[bool] = field(default=True, kw_only=True)
#     """
#     If True, the gripper is kept vertically aligned during the grasp
#     kw_only=True forces this to be passed as a keyword argument
#     """
#
#     _pre_perform_callbacks = []
#     """
#     List to save the callbacks which should be called before performing the action.
#     """
#
#     def __post_init__(self):
#         super().__post_init__()
#
#     def execute(self) -> None:
#         try:
#             from ...motions.pick_up import PickupMotion
#         except ImportError:
#             raise ImportError(
#                 "The GiskardPickUpAction requires Giskardpy_ros, not only giskardpy."
#             )
#
#         # Register attach as a post-perform callback BEFORE queuing the motion
#
#         manipulator = ViewManager.get_end_effector_view(self.arm, self.robot_view)
#         SequentialPlan(
#             self.context,
#             PickupMotion(
#                 simulated=self.simulated,
#                 manipulator=manipulator,
#                 object_geometry=self.object_designator,
#                 gripper_vertical=self.gripper_vertical,
#             ),
#         ).perform()
#
#     @classmethod
#     def description(
#         cls,
#         object_designator: Union[Iterable[Body], Body],
#         arm: Union[Iterable[Arms], Arms] = None,
#         gripper_vertical: Union[Iterable[bool], bool] = True,
#         simulated: bool = True,
#     ) -> PartialDesignator[GiskardGraspAction]:
#         return PartialDesignator[GiskardGraspAction](
#             GiskardGraspAction,
#             simulated=simulated,
#             object_designator=object_designator,
#             arm=arm,
#             gripper_vertical=gripper_vertical,
#         )
#
#
# @dataclass
# class GiskardPullUpAction(ActionDescription):
#     """
#     Let the robot pick up an object.
#     """
#
#     simulated: bool = field(default=True, kw_only=True)
#     """
#     Parsing simulation argument
#     """
#     object_designator: Body = field(default=None, kw_only=True)
#     """
#     Object designator_description describing the object that should be picked up
#     """
#     arm: Arms = field(default=Arms.LEFT, kw_only=True)
#     """
#     arms that should be used for pick up
#     """
#     _pre_perform_callbacks = []
#     """
#     List to save the callbacks which should be called before performing the action.
#     """
#
#     def __post_init__(self):
#         super().__post_init__()
#
#     def execute(self) -> None:
#         try:
#             from ...motions.pick_up import PickupMotion
#         except ImportError:
#             raise ImportError(
#                 "The GiskardPickUpAction requires Giskardpy_ros, not only giskardpy."
#             )
#
#         # Register attach as a post-perform callback BEFORE queuing the motion
#
#         manipulator = ViewManager.get_end_effector_view(self.arm, self.robot_view)
#         attach_object_to_hsrb(
#             world=self.world, object_designator=self.object_designator
#         )
#         SequentialPlan(
#             self.context,
#             PullUpMotion(
#                 simulated=self.simulated,
#                 manipulator=manipulator,
#                 object_geometry=self.object_designator,
#             ),
#         ).perform()
#
#     def validate(self):
#         pass
#
#     @classmethod
#     def description(
#         cls,
#         object_designator: Union[Iterable[Body], Body],
#         arm: Union[Iterable[Arms], Arms] = None,
#         simulated: bool = True,
#     ) -> PartialDesignator[GiskardPullUpAction]:
#         return PartialDesignator[GiskardPullUpAction](
#             GiskardPullUpAction,
#             object_designator=object_designator,
#             arm=arm,
#             simulated=simulated,
#         )
#
#
# ReachActionDescription = ReachAction.description
# PickUpActionDescription = PickUpAction.description
# GraspingActionDescription = GraspingAction.description
# GiskardPickUpActionDescription = GiskardPickUpAction.description
# GiskardGraspActionDescription = GiskardGraspAction.description
# GiskardPullUpActionDescription = GiskardPullUpAction.description
