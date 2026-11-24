"""
Tests for driving a base that could move in any direction only forwards.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pytest

from cramph.context import StatechartContext
from cramph.executor import (
    StatechartExecutor,
    TickingExecutorExtension,
)
from giskardpy.motion_statechart.graph_node import EndMotion
from cramph.statechart import Statechart
from giskardpy.motion_control import MotionControl
from giskardpy.motion_statechart.tasks.drive_tasks import ForwardBaseGoal
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.robots.robot_parts import MobileBase
from semantic_digital_twin.world_description.world_entity import Body

# %% recording the drive


@dataclass(kw_only=True)
class BasePoseRecorder(TickingExecutorExtension):
    """
    Records the planar pose of a body on every tick.
    """

    body: Body
    """
    The body to record.
    """

    planar_poses: list[list[float]] = field(default_factory=list)
    """
    The x, y and yaw of the body.
    """

    def after_tick(self, *, executor: StatechartExecutor) -> None:
        pose = self.body.global_pose
        self.planar_poses.append([float(pose.x), float(pose.y), float(pose.yaw)])


# %% driving forwards


@pytest.mark.parametrize(
    "goal_pose",
    [
        Pose.from_xyz_rpy(x=2, y=-0.5),
        Pose.from_xyz_rpy(x=-1, y=0.4),
        Pose.from_xyz_rpy(x=-0.5, y=-2, yaw=-np.pi / 2),
        Pose.from_xyz_rpy(yaw=np.pi / 2),
    ],
)
def test_base_faces_the_way_it_moves_along_a_straight_line(
    *, cylinder_bot_world, goal_pose
):
    """
    The base moves along the straight line to the goal, facing along it, and ends at the
    goal pose.
    """
    goal_pose.reference_frame = cylinder_bot_world.root
    bot = cylinder_bot_world.get_body_by_name("bot")
    [mobile_base] = cylinder_bot_world.get_semantic_annotations_by_type(MobileBase)
    recorder = BasePoseRecorder(body=bot)
    executor = StatechartExecutor(
        context=StatechartContext(world=cylinder_bot_world),
        extensions=[MotionControl(), recorder],
    )
    statechart = Statechart(context=executor.context)
    goal = ForwardBaseGoal(mobile_base=mobile_base, goal_pose=goal_pose)
    statechart.add_node(node=goal)
    statechart.add_node(node=EndMotion.when_true(node=goal))

    executor.compile(statechart=statechart)
    executor.tick_until_end()

    poses = np.array(recorder.planar_poses)
    steps = np.diff(poses[:, :2], axis=0)
    moving = np.linalg.norm(steps, axis=1) > 2e-3
    directions = np.arctan2(steps[moving, 1], steps[moving, 0])
    deviations = directions - poses[:-1][moving, 2]
    deviations = np.abs(np.arctan2(np.sin(deviations), np.cos(deviations)))
    goal_position = goal_pose.to_np()[:2, 3]
    line_normal = np.array([-goal_position[1], goal_position[0]])
    distances = np.abs(poses[:, :2] @ line_normal) / max(
        np.linalg.norm(line_normal), 1e-9
    )

    assert np.all(deviations < 0.1)
    assert np.all(distances < 0.02)
    assert bot.global_transform.to_np() == pytest.approx(goal_pose.to_np(), abs=2e-2)
