"""
Actions that move a robot's base or point it at something: driving to a pose, along a
planned path or by elevator, and looking or facing at a target.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field

import numpy as np

from coraplex.exceptions import NoFloorBelowRobot, NotOnASingleLevelException
from cramph.node import StatechartNode
from coraplex.robot_plans.actions.base import (
    Action,
    GoalAction,
    ParallelAction,
    SequentialAction,
)
from coraplex.robot_plans.mixins import (
    HasCameraParameter,
    HasLookTargetParameter,
    HasPlanarGoalPoseParameter,
    ReattachesBranches,
)
from cramph.composites import Parallel, PausedUntilTrue
from giskardpy.motion_statechart.tasks.drive_tasks import (
    MobileBasePoseGoal,
    DifferentialDriveBaseGoal,
    HolonomicBaseGoal,
)
from giskardpy.motion_statechart.mixins import HasThresholdParameter
from giskardpy.motion_statechart.monitors.joint_monitors import (
    JointPositionReached,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition
from giskardpy.motion_statechart.tasks.pointing import Pointing
from semantic_digital_twin.exceptions import MissingMovableJointError
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Level,
    Elevator,
    Floor,
)
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point2,
    Point3,
    Pose2D,
    Vector3,
)
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.kinematic_chain import KinematicChain
from semantic_digital_twin.world_description.graph_of_convex_sets.adjoining import (
    AdjoiningFreeSpaces,
    PlaceInFreeSpace,
)

# %% driving the base


@dataclass(eq=False, repr=False, kw_only=True)
class DrivesBase(Action, HasThresholdParameter, ABC):
    """
    Base class for the actions that move the robot's base to a pose.
    """

    threshold: float = DifferentialDriveBaseGoal.threshold
    """
    How far the base may be from a pose it drives to, in meters and in radians, to count
    as arrived.
    """

    base_goal_type: type[MobileBasePoseGoal] = field(
        default=HolonomicBaseGoal, init=False
    )
    """
    The kind of node putting the base at the poses this action drives it to, as set by
    the plan transformations configuring the action.
    """


@dataclass(eq=False, repr=False, kw_only=True)
class NavigateAction(DrivesBase, GoalAction, HasPlanarGoalPoseParameter):
    """
    Navigates the Robot to a position.
    """

    def create_goal(self) -> StatechartNode:
        return self.base_goal_type.moving_base_of(
            robot=self.robot,
            goal_pose=self.robot.mobile_base.pose_facing(heading=self.planar_goal_pose),
            threshold=self.threshold,
        )


@dataclass(eq=False, repr=False, kw_only=True)
class LookAtAction(ParallelAction, HasLookTargetParameter, HasCameraParameter):
    """
    Lets the robot look at a position, following it while the look runs when it is given
    relative to a body that moves.
    """

    def create_steps(self) -> list[StatechartNode]:
        camera = self.pointed_camera(robot=self.robot)
        pointing = Pointing(
            kinematic_chain=KinematicChain(
                root=self.robot.get_torso().root, tip=camera.root
            ),
            goal_point=self._world_P_target(),
            pointing_axis=camera.viewing_direction,
        )
        return [pointing, *self.keep_base_still()]

    def _world_P_target(self) -> Point3:
        """
        :return: Where :attr:`look_target` is in the world frame, as an expression of where
            the body it is given relative to stands while the look runs.
        """
        target_frame = self.look_target.reference_frame or self.world.root
        world_T_target_frame = self.world.compose_forward_kinematics_expression(
            self.world.root, target_frame
        )
        world_T_target_frame.reference_frame = self.world.root
        world_T_target_frame.child_frame = target_frame
        return world_T_target_frame @ self.look_target.position


@dataclass(eq=False, repr=False, kw_only=True)
class FaceAtAction(ParallelAction, HasLookTargetParameter):
    """
    Turns the robot's base on the spot until its front faces a target, of which only
    the horizontal position matters.

    The base keeps the position it has when the action starts, so the turn is towards
    the target from wherever an earlier action left it.
    """

    def create_steps(self) -> list[StatechartNode]:
        return [
            Pointing(
                kinematic_chain=KinematicChain(
                    root=self.world.root, tip=self.robot.root
                ),
                goal_point=self._target_at_base_height(),
                pointing_axis=Vector3(
                    *self.robot.mobile_base.forward_axis.to_np()[:3],
                    reference_frame=self.robot.root,
                ),
            ),
            CartesianPosition(
                kinematic_chain=KinematicChain(
                    root=self.world.root, tip=self.robot.root
                ),
                goal_point=Point3(reference_frame=self.robot.root),
            ),
        ]

    def _target_at_base_height(self) -> Point3:
        """
        :return: :attr:`look_target` moved vertically to the height of the base, which can
            only turn about the vertical and so can only point level.
        """
        root_P_target = self.world.transform(self.look_target, self.world.root).position
        root_P_target.z = self.robot.root.global_pose.z
        return root_P_target


@dataclass(eq=False, repr=False, kw_only=True)
class PathPlanningNavigateAction(
    DrivesBase, SequentialAction, HasPlanarGoalPoseParameter
):
    """
    Navigates the robot to a pose around the known furniture and walls, through the
    free space of the floors of its storey, so also into rooms other than its own.

    .. note:: Obstacles added during navigation are not avoided.
    """

    start: Pose2D | None = None
    """
    Where the robot's base stands when this drive begins, such as the target of the
    drive before it, or None for where it stands when the action is expanded.
    """

    obstacle_clearance: float = 0.0
    """
    How much further than its radius the base keeps away from obstacles, in m, so that
    parts reaching out past the base, such as a parked arm, pass them too.
    """

    floor_top_tolerance: float = 0.01
    """
    How far the top of a floor may lie above the robot's base, in m, for the robot to
    still stand on it, and how far apart the tops of two floors of one storey may lie.
    """

    def create_steps(self) -> list[StatechartNode]:
        return [
            self.base_goal_type.moving_base_of(
                robot=self.robot, goal_pose=waypoint.pose, threshold=self.threshold
            )
            for waypoint in self._path()
        ]

    @property
    def _start_position(self) -> Point2:
        """
        :return: Where on the floor the robot begins to drive, see :attr:`start`.
        """
        if self.start is None:
            return Point2.from_pose(self.robot.root.global_pose)
        return self.world.transform(self.start.position, self.world.root)

    @property
    def _floor(self) -> Floor:
        """
        The floor the robot stands on at :attr:`start`, where the path begins.

        A world with several storeys puts more than one floor below the robot; the one
        it stands on is the topmost of those it stands within the footprint of.

        :raises NoFloorBelowRobot: If the robot stands over no annotated floor.
        :return: The floor the robot drives on first.
        """
        floors_below = [
            floor
            for floor in self.world.get_semantic_annotations_by_type(Floor)
            if self._stands_on(floor=floor)
        ]
        if not floors_below:
            raise NoFloorBelowRobot(self.robot)
        return max(floors_below, key=lambda floor: self._extent_of(floor).max_z)

    def _extent_of(self, floor: Floor) -> VolumetricBoundingBox:
        """
        :param floor: The floor to measure.
        :return: The floor's bounding box in the world's root frame.
        """
        return floor.as_bounding_box_collection_at_origin(
            HomogeneousTransformationMatrix(reference_frame=self.world.root)
        ).bounding_box()

    def _stands_on(self, *, floor: Floor) -> bool:
        """
        :param floor: The floor to test.
        :return: Whether the robot's base, at :attr:`start`, rests within this floor's
            footprint and no more than :attr:`floor_top_tolerance` below its top.
        """
        return self._footprint_contains(
            floor=floor, position=self._start_position
        ) and self._extent_of(floor).max_z <= (
            float(self.robot.root.global_pose.z) + self.floor_top_tolerance
        )

    def _footprint_contains(self, *, floor: Floor, position: Point2) -> bool:
        """
        :param floor: The floor to test.
        :param position: The position to test, in the world's root frame.
        :return: Whether the position lies within the floor's footprint.
        """
        extent = self._extent_of(floor)
        return (
            extent.min_x <= float(position.x) <= extent.max_x
            and extent.min_y <= float(position.y) <= extent.max_y
        )

    def _floors_level_with(self, *, floor: Floor) -> list[Floor]:
        """
        :param floor: The floor whose storey is wanted.
        :return: The floors whose tops lie within :attr:`floor_top_tolerance` of the top
            of *floor*, *floor* included.
        """
        floor_top = self._extent_of(floor).max_z
        return [
            other
            for other in self.world.get_semantic_annotations_by_type(Floor)
            if abs(self._extent_of(other).max_z - floor_top) <= self.floor_top_tolerance
        ]

    def _path(self) -> list[Pose2D]:
        """
        The poses the robot drives to, one per leg of the path.

        Each pose faces the waypoint after it, so the leg leaving a waypoint no longer
        has to begin by turning. The waypoint the robot already stands on is left out,
        and the last pose is the requested target.

        .. note::
            The orientation aims the base's x-axis, which is the axis a drive travels
            along, rather than the base's
            :attr:`~semantic_digital_twin.robots.robot_parts.MobileBase.forward_axis`.
            The two differ on a base whose front is not its direction of travel, and it
            is travel that these orientations exist to line up.

        :return: The poses to drive to, in order.
        """
        waypoints = self._waypoints()
        poses = []
        for waypoint, next_waypoint in zip(waypoints[1:], waypoints[2:]):
            next_position = self.world.transform(
                next_waypoint, waypoint.reference_frame
            )
            poses.append(
                Pose2D(
                    waypoint,
                    yaw=np.arctan2(
                        float(next_position.y - waypoint.y),
                        float(next_position.x - waypoint.x),
                    ),
                )
            )
        return poses + [self.planar_goal_pose]

    def _waypoints(self) -> list[Point2]:
        """
        The points the robot travels through to get from :attr:`start` to the target.

        :return: The path, beginning at :attr:`start` and ending at the target's
            position, each waypoint in the frame of the free space above the floor it
            lies on.
        """
        start_floor = self._floor
        target_position = self.world.transform(
            self.planar_goal_pose.position, self.world.root
        )
        level_floors = self._floors_level_with(floor=start_floor)
        target_floor = next(
            (
                floor
                for floor in level_floors
                if self._footprint_contains(floor=floor, position=target_position)
            ),
            start_floor,
        )
        floors = [start_floor] if target_floor is start_floor else level_floors
        free_spaces = [
            floor.navigable_space(robot=self.robot, margin=self.obstacle_clearance)
            for floor in floors
        ]
        return AdjoiningFreeSpaces(
            world=self.world, free_spaces=free_spaces
        ).path_from_to(
            start=PlaceInFreeSpace(
                free_space=free_spaces[floors.index(start_floor)],
                position=self._start_position,
            ),
            goal=PlaceInFreeSpace(
                free_space=free_spaces[floors.index(target_floor)],
                position=target_position,
            ),
        )


@dataclass(eq=False, repr=False, kw_only=True)
class ElevatorNavigation(ReattachesBranches, SequentialAction):
    """
    Navigates a robot to another level of a building using an elevator, the robot drives
    in the elevator and waits there until the doors open again and the elevator is at
    the right level.
    """

    elevator: Elevator
    """
    Elevator the robot rides.
    """

    target_floor: Level
    """
    Level of the building the robot should end up on.
    """

    exit_clearance: float = 0.5
    """
    Distance the robot keeps from the elevator's opening after driving out, on top of
    half the cabin's depth.
    """

    arrival_threshold: float = 0.01
    """
    Position error within which the elevator's drive and doors count as having arrived.
    """

    def create_steps(self) -> list[StatechartNode]:
        return [
            NavigateAction(planar_goal_pose=self._pose_infront_of_elevator),
            PausedUntilTrue(
                monitor=self._elevator_open_at_floor(self._current_floor),
                monitored_node=NavigateAction(
                    planar_goal_pose=Pose2D(reference_frame=self.elevator.root)
                ),
            ),
            *self.reattach(branch_root=self.robot.root, new_parent=self.elevator.root),
            PausedUntilTrue(
                monitor=self._elevator_open_at_floor(self.target_floor),
                monitored_node=NavigateAction(
                    planar_goal_pose=self._pose_infront_of_elevator
                ),
            ),
            *self.reattach(branch_root=self.robot.root, new_parent=self.world.root),
        ]

    @property
    def _current_floor(self) -> Level:
        """
        Finds the floor the robot is currently on, based on its position in the world.

        Raises :class:`WrongLevelException` if the robot is not on any floor or on
        multiple floors at once.
        :return: The semantic annotation for the floor
        """
        current_floor = [
            floor
            for floor in self.world.get_semantic_annotations_by_type(Level)
            if InsideOf(body=self.robot.bodies_with_collision[0], other=floor.root)()
            > 0.9
        ]
        if len(current_floor) == 0:
            raise NotOnASingleLevelException("Robot is not on any recognized floor.")
        if len(current_floor) > 1:
            raise NotOnASingleLevelException("Robot is on multiple floors at once.")
        return current_floor[0]

    @property
    def _pose_infront_of_elevator(self) -> Pose2D:
        """
        Where the robot stands in front of the cabin's opening.
        """
        return Pose2D.from_xy_yaw(
            x=self.elevator.hole_direction[0]
            * (self.elevator.scale.x / 2 + self.exit_clearance),
            reference_frame=self.elevator.root,
        )

    def _elevator_open_at_floor(self, target_floor: Level) -> Parallel:
        """
        Observes True once the cabin serves :attr:`target_floor` with its doors open.
        """
        nodes = []
        for door in self.elevator.doors:
            if door.movable_joint is None:
                raise MissingMovableJointError(door)
            nodes.append(
                JointPositionReached(
                    connection=door.movable_joint,
                    position=door.movable_joint.dof.limits.upper.position,
                    threshold=self.arrival_threshold,
                    name=f"{door.name}Open",
                )
            )
        if self.elevator.movable_joint is None:
            raise MissingMovableJointError(self.elevator)
        nodes.append(
            JointPositionReached(
                connection=self.elevator.movable_joint,
                position=self.elevator.drive_position_for_floor(target_floor),
                threshold=self.arrival_threshold,
                name="ElevatorAtTargetFloor",
            )
        )
        return Parallel(
            nodes=nodes,
            name="ElevatorOpenAtTargetFloor",
        )
