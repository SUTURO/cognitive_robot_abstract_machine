from __future__ import annotations

import os
from dataclasses import dataclass, field
from datetime import timedelta

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Optional, Type, Any, Iterable

from .pick_up import PickUpAction
from pycram.robot_plans.motions.gripper import PlaceMotion
from ...motions.gripper import MoveTCPMotion
from ...motions.hri_handover import HandoverMotion
from ....datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....failures import ObjectNotPlacedAtTargetLocation, ObjectStillInContact
from ....language import SequentialPlan, CodePlan
from ....view_manager import ViewManager
from ....robot_plans.actions.base import ActionDescription
from ....validation.error_checkers import PoseErrorChecker


@dataclass
class PlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """
    target_location: PoseStamped
    """
    Pose in the world at which the object should be placed
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """
    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot_view)
        manipulator = arm.manipulator

        previous_pick = self.plan.get_previous_node_by_designator_type(
            self.plan_node, PickUpAction
        )
        previous_grasp = (
            previous_pick.designator_ref.grasp_description
            if previous_pick
            else GraspDescription(
                ApproachDirection.FRONT, VerticalAlignment.NoAlignment, manipulator
            )
        )

        SequentialPlan(
            self.context,
            ReachActionDescription(
                self.target_location,
                self.arm,
                previous_grasp,
                self.object_designator,
                reverse_reach_order=True,
            ),
            MoveGripperMotion(GripperState.OPEN, self.arm),
        ).perform()

        # Detaches the object from the robot
        world_root = self.world.root
        obj_transform = self.world.compute_forward_kinematics(
            world_root, self.object_designator
        )
        with self.world.modify_world():
            self.world.remove_connection(self.object_designator.parent_connection)
            connection = Connection6DoF.create_with_dofs(
                parent=world_root, child=self.object_designator, world=self.world
            )
            self.world.add_connection(connection)
            connection.origin = obj_transform

        _, _, retract_pose = previous_grasp._pose_sequence(
            self.target_location, self.object_designator, reverse=True
        )

        SequentialPlan(self.context, MoveTCPMotion(retract_pose, self.arm)).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if the object is placed at the target location.
        """
        self.validate_loss_of_contact()
        self.validate_placement_location()

    def validate_loss_of_contact(self):
        """
        Check if the object is still in contact with the robot after placing it.
        """
        contact_links = self.object_designator.get_contact_points_with_body(
            World.robot
        ).get_all_bodies()
        if contact_links:
            raise ObjectStillInContact(
                self.object_designator,
                contact_links,
                self.target_location,
                World.robot,
                self.arm,
            )

    def validate_placement_location(self):
        """
        Check if the object is placed at the target location.
        """
        pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_error_checker.is_error_acceptable(
            self.object_designator.pose, self.target_location
        ):
            raise ObjectNotPlacedAtTargetLocation(
                self.object_designator, self.target_location, World.robot, self.arm
            )

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
    ) -> PartialDesignator[PlaceAction]:
        return PartialDesignator[PlaceAction](
            PlaceAction,
            object_designator=object_designator,
            target_location=target_location,
            arm=arm,
        )


@dataclass
class GiskardPlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm. By directly called GiskardMotion
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """

    target_location: PoseStamped
    """
    Pose in the world at which the object should be placed
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    simulated: bool = field(default=True, kw_only=True)
    """
    Parsing simulation argument
    """

    ignore_orientation: bool = field(default=False, kw_only=True)
    """
    If True, the orientation of the object will be ignored.
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot_view)
        manipulator = arm.manipulator
        print("Transformer")
        if self.ignore_orientation:
            goal = self.target_location.pose.to_spatial_type().to_position()
        else:
            goal = self.target_location.pose.to_spatial_type()
        goal.reference_frame = self.target_location.frame_id
        print("PlaceMotion")
        SequentialPlan(
            self.context,
            PlaceMotion(
                object_designator=self.object_designator,
                simulated=self.simulated,
                goal_pose=goal,
                gripper=manipulator,
                allow_gripper_collision=False,
            ),
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if the object is placed at the target location.
        """
        self.validate_loss_of_contact()
        self.validate_placement_location()

    def validate_loss_of_contact(self):
        """
        Check if the object is still in contact with the robot after placing it.
        """
        manipulator = ViewManager.get_arm_view(
            self.arm, self.robot_view
        ).manipulator.tool_frame
        contact_links = self.object_designator.get_contact_points_with_body(
            self.robot_view
        ).get_all_bodies()
        if contact_links:
            raise ObjectStillInContact(
                self.object_designator,
                contact_links,
                self.target_location,
                World.robot,
                self.arm,
            )

    def validate_placement_location(self):
        """
        Check if the object is placed at the target location.
        """
        pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_error_checker.is_error_acceptable(
            self.object_designator.pose, self.target_location
        ):
            raise ObjectNotPlacedAtTargetLocation(
                self.object_designator, self.target_location, World.robot, self.arm
            )

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
        simulated: bool = True,
        ignore_orientation: bool = False,
    ) -> PartialDesignator[GiskardPlaceAction]:
        return PartialDesignator[GiskardPlaceAction](
            GiskardPlaceAction,
            object_designator=object_designator,
            target_location=target_location,
            arm=arm,
            simulated=simulated,
            ignore_orientation=ignore_orientation,
        )


@dataclass
class GiskardPlaceAndDetachAction(ActionDescription):
    """
    Places an Object at a position using an arm. By directly called GiskardMotion
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """

    target_location: PoseStamped
    """
    Pose in the world at which the object should be placed
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    simulated: bool = field(default=True, kw_only=True)
    """
    Parsing simulation argument
    """

    ignore_orientation: bool = field(default=False, kw_only=True)
    """
    If True, the orientation of the object will be ignored.
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        from ... import ParkArmsActionDescription

        robot_pre_action_pose = PoseStamped.from_spatial_type(
            self.robot_view.root.global_pose
        )
        print("Performing PlaceAction")
        SequentialPlan(
            self.context,
            GiskardPlaceActionDescription(
                simulated=self.simulated,
                object_designator=self.object_designator,
                arm=Arms.LEFT,
                target_location=self.target_location,
                ignore_orientation=self.ignore_orientation,
            ),
        ).perform()
        print("Placed object")

        print("Detach object")
        with self.world.modify_world():
            self.world.move_branch_with_fixed_connection(
                self.object_designator, self.world.root
            )
        print("Detached object")

        print("Retracting")
        SequentialPlan(
            self.context,
            GiskardRetractActionDescription(
                simulated=self.simulated,
                arm=self.arm,
                back_off_pose=robot_pre_action_pose,
            ),
            ParkArmsActionDescription(Arms.BOTH),
        ).perform()
        print("Retracted")

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        target_location: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
        simulated: bool = True,
        ignore_orientation: bool = False,
    ) -> PartialDesignator[GiskardPlaceAndDetachAction]:
        return PartialDesignator[GiskardPlaceAndDetachAction](
            GiskardPlaceAndDetachAction,
            object_designator=object_designator,
            target_location=target_location,
            arm=arm,
            simulated=simulated,
            ignore_orientation=ignore_orientation,
        )


@dataclass
class GiskardRetractAction(ActionDescription):
    """
    Places an Object at a position using an arm. By directly called GiskardMotion
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    simulated: bool = field(default=True, kw_only=True)
    """
    Parsing simulation argument
    """

    back_off_pose: PoseStamped = field(default=None, kw_only=True)

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

    def execute(self) -> None:
        from ... import RetractMotion, GiskardMoveGripperMotion
        from pycram.robot_plans import nav2NavigateActionDescription

        arm = ViewManager.get_arm_view(self.arm, self.robot_view)
        manipulator = arm.manipulator
        SequentialPlan(
            self.context,
            GiskardMoveGripperMotion(GripperState.OPEN, self.simulated),
            RetractMotion(
                simulated=self.simulated,
                gripper=manipulator,
            ),
            nav2NavigateActionDescription(
                simulated=self.simulated, target_location=self.back_off_pose
            ),
        ).perform()

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        """
        Check if the object is placed at the target location.
        """
        self.validate_loss_of_contact()
        self.validate_placement_location()

    def validate_loss_of_contact(self):
        """
        Check if the object is still in contact with the robot after placing it.
        """
        manipulator = ViewManager.get_arm_view(
            self.arm, self.robot_view
        ).manipulator.tool_frame
        contact_links = self.object_designator.get_contact_points_with_body(
            self.robot_view
        ).get_all_bodies()
        if contact_links:
            raise ObjectStillInContact(
                self.object_designator,
                contact_links,
                self.target_location,
                World.robot,
                self.arm,
            )

    def validate_placement_location(self):
        """
        Check if the object is placed at the target location.
        """
        pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_error_checker.is_error_acceptable(
            self.object_designator.pose, self.target_location
        ):
            raise ObjectNotPlacedAtTargetLocation(
                self.object_designator, self.target_location, World.robot, self.arm
            )

    @classmethod
    def description(
        cls,
        arm: Union[Iterable[Arms], Arms],
        simulated: bool,
        back_off_pose: Union[Iterable[PoseStamped], PoseStamped] | None = None,
    ) -> PartialDesignator[GiskardRetractAction]:
        return PartialDesignator[GiskardRetractAction](
            GiskardRetractAction,
            arm=arm,
            simulated=simulated,
            back_off_pose=back_off_pose,
        )


@dataclass
class HandoverAction(ActionDescription):

    world: World = field(kw_only=True, default=None)

    def execute(self) -> None:
        SequentialPlan(
            self.context,
            HandoverMotion(world=self.world),
        ).perform()

    @classmethod
    def description(
        cls,
        world: World | None = None,
    ) -> PartialDesignator[HandoverAction]:
        return PartialDesignator[HandoverAction](
            HandoverAction,
            world=world,
        )


PlaceActionDescription = PlaceAction.description
GiskardPlaceActionDescription = GiskardPlaceAction.description
GiskardPlaceAndDetachActionDescription = GiskardPlaceAndDetachAction.description
GiskardRetractActionDescription = GiskardRetractAction.description
HandoverActionDescription = HandoverAction.description
