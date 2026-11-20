"""
Motion control of a world a physics moves, which leaves every degree of freedom outside
the robots to the physics.
"""

from __future__ import annotations

import pytest

from cramph.context import StatechartContext
from cramph.executor import StatechartExecutor
from cramph.statechart import Statechart
from giskardpy.motion_control import MotionControl
from giskardpy.motion_statechart.tasks.grasped_mechanism import (
    ArticulateGraspedMechanism,
)
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    WashingMachine,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World

# %% a door the robot opens


@pytest.fixture
def hsrb_before_the_cracked_open_washing_machine() -> World:
    world = KitchenEnvironment().create_world()
    [washing_machine] = world.get_semantic_annotations_by_type(WashingMachine)
    [door] = washing_machine.doors
    door.root.parent_connection.position = 0.2
    standing_pose = Pose.from_xyz_rpy(x=-0.9, reference_frame=washing_machine.root)
    RobotSpecification(
        HSRB,
        localization_pose=world.transform(standing_pose, world.root).homogeneous_matrix,
    ).spawn(world)
    return world


def test_the_controller_leaves_a_door_hinge_to_the_physics(
    hsrb_before_the_cracked_open_washing_machine,
):
    """
    The controller still commands the hinge, but only the physics, through the gripper
    pulling the door, may move it.
    """
    world = hsrb_before_the_cracked_open_washing_machine
    [robot] = world.get_semantic_annotations_by_type(HSRB)
    [washing_machine] = world.get_semantic_annotations_by_type(WashingMachine)
    [door] = washing_machine.doors
    hinge = door.root.parent_connection
    start = hinge.position
    executor = StatechartExecutor(
        context=StatechartContext(world=world),
        extensions=[MotionControl(environment_moved_by_physics=True)],
    )
    statechart = Statechart(context=executor.context)
    statechart.add_node(
        node=ArticulateGraspedMechanism(
            tip_link=robot.all_arms[0].end_effector.tool_frame,
            environment_link=door.handle.root,
            goal_joint_state=hinge.dof.limits.upper.position,
        )
    )
    executor.compile(statechart=statechart)

    for _ in range(20):
        executor.tick()

    assert world.state[hinge.dof.id].velocity != 0.0
    assert hinge.position == start
