"""
Tests for looking at a target while the robot's base drives.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from coraplex.robot_plans.actions.composite.facing import NavigateAndLookAtAction
from coraplex.robot_plans.actions.core.navigation import (
    LookAtAction,
    LookAtWhileDrivingAction,
    NavigateAction,
)
from cramph.composites import Sequence
from cramph.data_types import LifeCycleValue
from cramph.executor import StatechartExecutor, TickingExecutorExtension
from giskardpy.motion_statechart.tasks.cartesian_tasks import HoldPose
from giskardpy.motion_statechart.tasks.pointing import Pointing
from krrood_spatial_math.spatial_types import Pose, Pose2D
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from ..conftest import nodes_of_type
from ...plan_running import expand, run_plan

# %% measuring where the camera points


@dataclass(kw_only=True)
class ViewingErrorRecorder(TickingExecutorExtension):
    """
    Records how far the camera's viewing direction is off a target on every tick an
    action runs.
    """

    robot: AbstractRobot
    """
    The robot whose default camera looks.
    """

    look_target: Pose
    """
    What the camera should look at.
    """

    action: NavigateAndLookAtAction
    """
    The action during which the errors are recorded.
    """

    errors: list[float] = field(default_factory=list)
    """
    The angles between the viewing direction and the target, in radians.
    """

    base_positions: list[np.ndarray] = field(default_factory=list)
    """
    Where the base stood when each of the :attr:`errors` was recorded.
    """

    def after_tick(self, *, executor: StatechartExecutor) -> None:
        if self.action.life_cycle_state != LifeCycleValue.RUNNING:
            return
        camera = self.robot.get_default_camera()
        world = executor.context.world
        camera_P_target = world.transform(self.look_target, camera.root).position
        direction = camera_P_target.to_np()[:3]
        viewing_direction = camera.viewing_direction.to_np()[:3]
        cosine = direction @ viewing_direction / np.linalg.norm(direction)
        self.errors.append(float(np.arccos(np.clip(cosine, -1, 1))))
        self.base_positions.append(self.robot.root.global_pose.position.to_np()[:3])


# %% looking while driving


def drive_past_target(
    robot: AbstractRobot, look_target: Pose
) -> NavigateAndLookAtAction:
    """
    :param robot: The robot that drives.
    :param look_target: What it looks at meanwhile.
    :return: A drive of one meter to the robot's left.
    """
    return NavigateAndLookAtAction(
        navigate=NavigateAction(
            planar_goal_pose=Pose2D.from_xy_yaw(
                x=1.5, y=3.0, reference_frame=robot._world.root
            )
        ),
        look_at=LookAtWhileDrivingAction(look_target=look_target),
    )


def test_camera_stays_on_the_target_during_the_drive(hsr_apartment_context):
    """
    A camera that looks at the target before the drive stays on it until the base
    arrived.
    """
    world, robot, extensions = hsr_apartment_context
    look_target = Pose.from_xyz_rpy(3.5, 2.0, 0.7, reference_frame=world.root)
    action = drive_past_target(robot, look_target)
    recorder = ViewingErrorRecorder(robot=robot, look_target=look_target, action=action)

    run_plan(
        Sequence(nodes=[LookAtAction(look_target=look_target), action]),
        extensions,
        plan_adaptations=[],
        extensions=[recorder],
    )

    travelled = np.linalg.norm(recorder.base_positions[-1] - recorder.base_positions[0])
    assert travelled > 0.9
    assert max(recorder.errors) < 0.1


def test_look_while_driving_leaves_the_base_free(hsr_apartment_context):
    """
    Looking while driving holds no base still, even on a robot that does so for a look.
    """
    world, robot, extensions = hsr_apartment_context
    robot.mobile_base.full_body_controlled = False
    look_target = Pose.from_xyz_rpy(3.5, 2.0, 0.7, reference_frame=world.root)
    action = drive_past_target(robot, look_target)

    expand(action, extensions)

    assert len(nodes_of_type(action, Pointing)) == 1
    assert nodes_of_type(action, HoldPose) == []
