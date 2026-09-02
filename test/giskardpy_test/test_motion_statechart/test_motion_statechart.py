import json
import time
from dataclasses import dataclass
from math import radians
from time import sleep
from typing import Type

import numpy as np
import pytest

from giskardpy.data_types.exceptions import DuplicateNameException
from giskardpy.executor import Executor, SimulationPacer
from giskardpy.motion_statechart.binding_policy import GoalBindingPolicy
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import (
    LifeCycleValues,
    ObservationStateValues,
    DefaultWeights,
)
from giskardpy.motion_statechart.exceptions import (
    NotInMotionStatechartError,
    EndMotionInGoalError,
    InputNotExpressionError,
    SelfInStartConditionError,
    NonObservationVariableError,
    NodeAlreadyBelongsToDifferentNodeError,
)
from giskardpy.motion_statechart.goals.cartesian_goals import (
    DifferentialDriveBaseGoal,
    CartesianPoseStraight,
)
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
    SelfCollisionAvoidance,
    ExternalCollisionDistanceMonitor,
    SelfCollisionDistanceMonitor,
    UpdateTemporaryCollisionRules,
)
from giskardpy.motion_statechart.goals.open_close import Open, Close
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.motion_statechart.graph_node import (
    EndMotion,
    CancelMotion,
)
from giskardpy.motion_statechart.graph_node import ThreadPayloadMonitor
from giskardpy.motion_statechart.monitors.joint_monitors import JointPositionReached
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from giskardpy.motion_statechart.monitors.overwrite_state_monitors import (
    SetSeedConfiguration,
    SetOdometry,
)
from giskardpy.motion_statechart.monitors.payload_monitors import (
    Print,
    Pulse,
    CountSeconds,
    CountControlCycles,
)
from giskardpy.motion_statechart.motion_statechart import (
    MotionStatechart,
)
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.grasp_bar import GraspBar
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianOrientation,
    CartesianPosition,
    CartesianPositionStraight,
    CartesianVelocityLimit,
    CartesianPositionVelocityLimit,
    CartesianRotationVelocityLimit,
    CartesianPositionTrajectory,
)
from giskardpy.motion_statechart.tasks.feature_functions import (
    AngleGoal,
    AlignPerpendicular,
    DistanceGoal,
    HeightGoal,
)
from giskardpy.motion_statechart.tasks.admittance_tasks import (
    AdmittanceCartesianPosition,
    LowerUntilContact,
    NonPositiveVirtualMassError,
)
from giskardpy.motion_statechart.goals.wipe_goals import WipeGoal
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.tasks.pointing import Pointing, PointingCone
from giskardpy.motion_statechart.test_nodes.test_nodes import (
    ChangeStateOnEvents,
    ConstTrueNode,
    TestGoal,
    TestNestedGoal,
    ConstFalseNode,
    TestRunAfterStop,
    TestRunAfterStopFromPause,
    TestEndBeforeStart,
    TestUnpauseUnknownFromParentPause,
)
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSensorUpdater,
)
from giskardpy.ros_executor import Ros2Executor
from giskardpy.qp.constraint import EqualityConstraint
from giskardpy.qp.constraint_collection import ConstraintCollection
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.utils.math import angle_between_vector
from krrood.symbolic_math.symbolic_math import (
    trinary_logic_and,
    trinary_logic_not,
    trinary_logic_or,
    FloatVariable,
    shortest_angular_distance,
)
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AvoidCollisionBetweenGroups,
    AvoidExternalCollisions,
    AvoidAllCollisions,
    AllowAllCollisions,
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import (
    AbstractRobot,
    ForceTorqueSensor,
    Manipulator,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.minimal_robot import MinimalRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
    Door,
    Hinge,
)
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
    Vector3,
    Point3,
    RotationMatrix,
)
from semantic_digital_twin.spatial_types.derivatives import DerivativeMap
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    RevoluteConnection,
    ActiveConnection1DOF,
    FixedConnection,
    OmniDrive,
    PrismaticConnection,
)
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedom,
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.geometry import (
    Cylinder,
    Box,
    Scale,
    Color,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)
from semantic_digital_twin.world_description.world_state import WorldStateTrajectory
from semantic_digital_twin.world_description.world_state_trajectory_plotter import (
    WorldStateTrajectoryPlotter,
)


@pytest.fixture()
def better_pr2_pose():
    return {
        "r_shoulder_pan_joint": -1.7125,
        "r_shoulder_lift_joint": -0.25672,
        "r_upper_arm_roll_joint": -1.46335,
        "r_elbow_flex_joint": -2.12,
        "r_forearm_roll_joint": 1.76632,
        "r_wrist_flex_joint": -0.10001,
        "r_wrist_roll_joint": 0.05106,
        "l_shoulder_pan_joint": 1.9652,
        "l_shoulder_lift_joint": -0.26499,
        "l_upper_arm_roll_joint": 1.3837,
        "l_elbow_flex_joint": -2.12,
        "l_forearm_roll_joint": 16.99,
        "l_wrist_flex_joint": -0.10001,
        "l_wrist_roll_joint": 0,
        "torso_lift_joint": 0.2,
        "l_gripper_l_finger_joint": 0.55,
        "r_gripper_l_finger_joint": 0.55,
        "head_pan_joint": 0,
        "head_tilt_joint": 0,
    }


@pytest.fixture(scope="function")
def pr2_with_box(pr2_world_copy) -> World:
    with pr2_world_copy.modify_world():
        box = Body(
            name=PrefixedName("box"),
            visual=ShapeCollection(shapes=[Box(scale=Scale(1, 1, 1))]),
            collision=ShapeCollection(shapes=[Box(scale=Scale(1, 1, 1))]),
        )
        root_C_box = FixedConnection(
            parent=pr2_world_copy.root,
            child=box,
            parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=1.2, z=0.3, reference_frame=pr2_world_copy.root
            ),
        )
        pr2_world_copy.add_connection(root_C_box)
    return pr2_world_copy


def test_condition_to_str():
    msc = MotionStatechart()
    node1 = ConstTrueNode()
    msc.add_node(node1)
    node2 = ConstTrueNode()
    msc.add_node(node2)
    node3 = ConstTrueNode()
    msc.add_node(node3)
    end = EndMotion()
    msc.add_node(end)

    end.start_condition = trinary_logic_and(
        node1.observation_variable,
        trinary_logic_or(
            node2.observation_variable,
            trinary_logic_not(node3.observation_variable),
        ),
    )
    a = str(end._start_condition)
    assert a == '("ConstTrueNode#0" and ("ConstTrueNode#1" or not "ConstTrueNode#2"))'


def test_motion_statechart_to_dot(tmp_path):
    msc = MotionStatechart()
    node1 = ConstTrueNode()
    msc.add_node(node1)
    node2 = ConstTrueNode()
    msc.add_node(node2)
    end = EndMotion()
    msc.add_node(end)
    node1.end_condition = node2.observation_variable
    end.start_condition = trinary_logic_and(
        node1.observation_variable, node2.observation_variable
    )
    msc.draw(str(tmp_path / "muh.pdf"))


def test_print():
    msc = MotionStatechart()
    print_node1 = Print(name="cow", message="muh")
    msc.add_node(print_node1)
    print_node2 = Print(name="cow2", message="muh")
    msc.add_node(print_node2)

    node1 = ConstTrueNode()
    msc.add_node(node1)
    end = EndMotion()
    msc.add_node(end)

    node1.start_condition = print_node1.observation_variable
    print_node2.start_condition = node1.observation_variable
    end.start_condition = print_node2.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=World()))
    kin_sim.compile(motion_statechart=msc)

    assert len(msc.nodes) == 4
    assert len(msc.edges) == 3

    assert print_node1.observation_state == ObservationStateValues.UNKNOWN
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert print_node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert print_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    kin_sim.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert print_node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    kin_sim.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.TRUE
    assert print_node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    kin_sim.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.TRUE
    assert print_node2.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert not msc.is_end_motion()

    kin_sim.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.TRUE
    assert print_node2.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.TRUE

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert msc.is_end_motion()


def test_draw_with_invisible_node(tmp_path):
    msc = MotionStatechart()
    msc.add_nodes(
        [
            sequence := Sequence(
                nodes=[s1n1 := ConstTrueNode(), s1n2 := ConstTrueNode()]
            ),
            sequence2 := Sequence(
                nodes=[s2n1 := ConstTrueNode(), s2n2 := ConstTrueNode()]
            ),
        ]
    )
    msc.add_node(EndMotion.when_all_true(msc.nodes))

    sequence.plot_specs.visible = False
    s1n2.plot_specs.visible = False
    s2n2.plot_specs.visible = False

    kin_sim = Executor(MotionStatechartContext(world=World()))
    kin_sim.compile(motion_statechart=msc)
    msc.draw(str(tmp_path / "muh.pdf"))


class TestConditions:
    def test_InvalidConditionError(self):
        node = ConstTrueNode()
        with pytest.raises(InputNotExpressionError):
            node.end_condition = node

    def test_nodes_cannot_have_themselves_as_start_condition(self):
        msc = MotionStatechart()
        node1 = ConstTrueNode()
        msc.add_node(node1)
        with pytest.raises(SelfInStartConditionError):
            node1.start_condition = node1.observation_variable

    def test_non_observation_variable_in_condition(self):
        msc = MotionStatechart()
        msc.add_node(node := ConstTrueNode())
        with pytest.raises(NonObservationVariableError):
            node.start_condition = FloatVariable(name="muh")

    def test_add_node_to_multiple_goals(self):
        msc = MotionStatechart()
        node = ConstTrueNode()
        msc.add_node(Sequence([node]))
        msc.add_node(Sequence([node]))

        kin_sim = Executor(
            MotionStatechartContext(
                world=World(),
            )
        )
        with pytest.raises(NodeAlreadyBelongsToDifferentNodeError):
            kin_sim.compile(motion_statechart=msc)

    def test_add_node_to_multiple_goals2(self):
        msc = MotionStatechart()
        node = ConstTrueNode()
        msc.add_node(node)
        msc.add_node(Sequence([node]))

        kin_sim = Executor(
            MotionStatechartContext(
                world=World(),
            )
        )
        with pytest.raises(NodeAlreadyBelongsToDifferentNodeError):
            kin_sim.compile(motion_statechart=msc)


def test_two_goals(pr2_world_state_reset: World):
    torso_joint = pr2_world_state_reset.get_connection_by_name("torso_lift_joint")
    r_wrist_roll_joint = pr2_world_state_reset.get_connection_by_name(
        "r_wrist_roll_joint"
    )
    msc = MotionStatechart()
    msc.add_nodes(
        [
            JointPositionList(goal_state=JointState.from_mapping({torso_joint: 0.1})),
            local_min := LocalMinimumReached(),
        ]
    )
    msc.add_node(EndMotion.when_true(local_min))

    kin_sim = Executor(
        MotionStatechartContext(
            world=pr2_world_state_reset,
        )
    )
    kin_sim.compile(motion_statechart=msc)

    kin_sim.tick_until_end()
    assert np.isclose(torso_joint.position, 0.1, atol=1e-4)

    msc = MotionStatechart()
    msc.add_node(
        joint_goal := JointPositionList(
            goal_state=JointState.from_mapping({r_wrist_roll_joint: 1})
        )
    )
    msc.add_node(EndMotion.when_true(joint_goal))

    kin_sim = Executor(
        MotionStatechartContext(
            world=pr2_world_state_reset,
        )
    )
    kin_sim.compile(motion_statechart=msc)

    kin_sim.tick_until_end()
    assert np.isclose(torso_joint.position, 0.1, atol=1e-4)
    assert np.allclose(pr2_world_state_reset.state.velocities, 0)
    assert np.allclose(pr2_world_state_reset.state.accelerations, 0)
    assert np.allclose(pr2_world_state_reset.state.jerks, 0)


@dataclass(eq=False, repr=False)
class _TestThreadMonitor(ThreadPayloadMonitor):
    delay: float = 0.05
    return_value: float = ObservationStateValues.TRUE

    def _compute_observation(self):
        time.sleep(self.delay)
        return self.return_value


def test_thread_payload_monitor_non_blocking_and_caching():
    msc = MotionStatechart()
    mon = _TestThreadMonitor(
        delay=0.05,
        return_value=ObservationStateValues.TRUE,
    )
    msc.add_node(mon)
    # First call should be non-blocking and return Unknown until worker completes at least once
    start = time.perf_counter()
    val0 = mon.compute_observation()
    elapsed = time.perf_counter() - start
    assert elapsed < mon.delay / 4.0
    assert val0 == ObservationStateValues.UNKNOWN
    # Wait for worker to finish and cache
    time.sleep(mon.delay * 2)
    val1 = mon.compute_observation()
    assert val1 == ObservationStateValues.TRUE


class TestMotionStatechartLogic:

    def test_transition_triggers(self, tmp_path):
        msc = MotionStatechart()

        changer = ChangeStateOnEvents()
        msc.add_node(changer)

        node1 = Pulse()
        msc.add_node(node1)

        node2 = Pulse()
        msc.add_node(node2)
        node2.start_condition = node1.observation_variable

        node3 = Pulse()
        msc.add_node(node3)
        node3.start_condition = trinary_logic_and(
            trinary_logic_not(node1.observation_variable),
            trinary_logic_not(node2.observation_variable),
        )

        node4 = Pulse()
        msc.add_node(node4)
        node4.start_condition = node3.observation_variable

        changer.start_condition = node1.observation_variable
        changer.pause_condition = node2.observation_variable
        changer.end_condition = node3.observation_variable
        changer.reset_condition = node4.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)

        assert changer.state is None

        kin_sim.tick()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert changer.life_cycle_state == LifeCycleValues.RUNNING
        assert changer.state == "on_start"

        kin_sim.tick()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert changer.life_cycle_state == LifeCycleValues.PAUSED
        assert changer.state == "on_pause"

        kin_sim.tick()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert changer.life_cycle_state == LifeCycleValues.RUNNING
        assert changer.state == "on_unpause"

        kin_sim.tick()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert changer.life_cycle_state == LifeCycleValues.DONE
        assert changer.state == "on_end"

        kin_sim.tick()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert changer.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert changer.state == "on_reset"

    def test_not_not_in_motion_statechart(self):
        node = ConstTrueNode()
        with pytest.raises(NotInMotionStatechartError):
            muh = node.observation_variable
        with pytest.raises(NotInMotionStatechartError):
            muh = node.life_cycle_variable
        with pytest.raises(NotInMotionStatechartError):
            node.start_condition = node.observation_variable
        with pytest.raises(NotInMotionStatechartError):
            node.pause_condition = node.observation_variable
        with pytest.raises(NotInMotionStatechartError):
            node.end_condition = node.observation_variable
        with pytest.raises(NotInMotionStatechartError):
            node.reset_condition = node.observation_variable

    def test_cancel_motion(self, tmp_path):
        msc = MotionStatechart()
        node1 = ConstTrueNode()
        msc.add_node(node1)
        cancel = CancelMotion(exception=Exception("muh"))
        msc.add_node(cancel)
        cancel.start_condition = node1.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)

        kin_sim.tick()  # first tick, cancel goes into running
        with pytest.raises(Exception):
            kin_sim.tick()  # second tick, cancel goes true and triggers
        msc.draw(str(tmp_path / "muh.pdf"))

    def test_motion_statechart(self):
        msc = MotionStatechart()

        node1 = ConstTrueNode()
        msc.add_node(node1)
        node2 = ConstTrueNode()
        msc.add_node(node2)
        node3 = ConstTrueNode()
        msc.add_node(node3)
        end = EndMotion()
        msc.add_node(end)

        node1.start_condition = trinary_logic_or(
            node3.observation_variable, node2.observation_variable
        )
        end.start_condition = node1.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)

        assert len(msc.nodes) == 4
        assert len(msc.edges) == 3
        kin_sim.tick_until_end()

        assert len(msc.history) == 5
        # %% node1
        assert msc.history.get_life_cycle_history_of_node(node1) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(node1) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]
        # %% node2
        assert msc.history.get_life_cycle_history_of_node(node2) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(node2) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]
        # %% node3
        assert msc.history.get_life_cycle_history_of_node(node3) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(node3) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]
        # %% end
        assert msc.history.get_life_cycle_history_of_node(end) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(end) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
        ]

    def test_goal(self, tmp_path):
        msc = MotionStatechart()

        node1 = ConstTrueNode()
        msc.add_node(node1)

        goal = TestGoal()
        msc.add_node(goal)

        goal.start_condition = node1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = goal.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=World()))

        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        assert len(msc.history) == 7
        # %% goal
        assert msc.history.get_life_cycle_history_of_node(goal) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(goal) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]
        # %% node1
        assert msc.history.get_life_cycle_history_of_node(node1) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(node1) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]
        # %% sub_node1
        assert msc.history.get_life_cycle_history_of_node(goal.sub_node1) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.DONE,
            LifeCycleValues.DONE,
            LifeCycleValues.DONE,
            LifeCycleValues.DONE,
        ]
        assert msc.history.get_observation_history_of_node(goal.sub_node1) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]
        # %% sub_node2
        assert msc.history.get_life_cycle_history_of_node(goal.sub_node2) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(goal.sub_node2) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]
        # %% sub_node2
        assert msc.history.get_life_cycle_history_of_node(end) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(end) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
        ]
        msc.draw(str(tmp_path / "muh.pdf"))

    def test_reset(self, tmp_path):
        msc = MotionStatechart()
        node1 = ConstTrueNode()
        msc.add_node(node1)
        node2 = ConstTrueNode()
        msc.add_node(node2)
        node3 = ConstTrueNode()
        msc.add_node(node3)
        end = EndMotion()
        msc.add_node(end)
        node1.reset_condition = node2.observation_variable
        node2.start_condition = node1.observation_variable
        node3.start_condition = node2.observation_variable
        node2.end_condition = node2.observation_variable
        end.start_condition = trinary_logic_and(
            node1.observation_variable,
            node2.observation_variable,
            node3.observation_variable,
        )

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        msc.draw(str(tmp_path / "muh.pdf"))

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert node2.observation_state == ObservationStateValues.UNKNOWN
        assert end.observation_state == ObservationStateValues.UNKNOWN
        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert node2.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert node2.observation_state == ObservationStateValues.TRUE
        assert end.observation_state == ObservationStateValues.UNKNOWN
        assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert node2.life_cycle_state == LifeCycleValues.DONE
        assert node3.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.UNKNOWN
        assert node2.observation_state == ObservationStateValues.TRUE
        assert node3.observation_state == ObservationStateValues.TRUE
        assert end.observation_state == ObservationStateValues.UNKNOWN
        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert node2.life_cycle_state == LifeCycleValues.DONE
        assert node3.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert node2.observation_state == ObservationStateValues.TRUE
        assert node3.observation_state == ObservationStateValues.TRUE
        assert end.observation_state == ObservationStateValues.UNKNOWN
        assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert node2.life_cycle_state == LifeCycleValues.DONE
        assert node3.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.RUNNING
        assert not msc.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.UNKNOWN
        assert node2.observation_state == ObservationStateValues.TRUE
        assert node3.observation_state == ObservationStateValues.TRUE
        assert end.observation_state == ObservationStateValues.TRUE
        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert node2.life_cycle_state == LifeCycleValues.DONE
        assert node3.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.RUNNING
        assert msc.is_end_motion()

    def test_nested_goals(self):
        msc = MotionStatechart()

        node1 = ConstTrueNode(name="w")
        msc.add_node(node1)

        outer = TestNestedGoal()
        msc.add_node(outer)
        outer.start_condition = node1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = outer.observation_variable

        json_data = msc.to_json()
        json_str = json.dumps(json_data)
        new_json_data = json.loads(json_str)
        msc_copy = MotionStatechart.from_json(new_json_data)

        for node in msc.nodes:
            assert node.index == msc_copy.get_node_by_index(node.index).index

        kin_sim = Executor(MotionStatechartContext(world=World()))
        node1 = msc_copy.get_nodes_by_type(ConstTrueNode)[0]
        outer = msc_copy.get_nodes_by_type(TestNestedGoal)[0]
        end = msc_copy.get_nodes_by_type(EndMotion)[0]
        kin_sim.compile(motion_statechart=msc_copy)

        assert node1.depth == 0
        assert outer.depth == 0
        assert end.depth == 0
        assert outer.inner.depth == 1
        assert outer.inner.sub_node1.depth == 2
        assert outer.inner.sub_node2.depth == 2

        assert node1.observation_state == ObservationStateValues.UNKNOWN
        assert outer.inner.sub_node1.observation_state == ObservationStateValues.UNKNOWN
        assert outer.inner.sub_node2.observation_state == ObservationStateValues.UNKNOWN
        assert outer.inner.observation_state == ObservationStateValues.UNKNOWN
        assert outer.observation_state == ObservationStateValues.UNKNOWN
        assert end.observation_state == ObservationStateValues.UNKNOWN

        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node1.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert outer.inner.sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert outer.inner.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert outer.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc_copy.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node1.observation_state == ObservationStateValues.UNKNOWN
        assert outer.inner.sub_node2.observation_state == ObservationStateValues.UNKNOWN
        assert outer.inner.observation_state == ObservationStateValues.UNKNOWN
        assert outer.observation_state == ObservationStateValues.UNKNOWN
        assert end.observation_state == ObservationStateValues.UNKNOWN

        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert outer.inner.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc_copy.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node2.observation_state == ObservationStateValues.UNKNOWN
        assert outer.inner.observation_state == ObservationStateValues.UNKNOWN
        assert outer.observation_state == ObservationStateValues.UNKNOWN
        assert end.observation_state == ObservationStateValues.UNKNOWN

        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node1.life_cycle_state == LifeCycleValues.DONE
        assert outer.inner.sub_node2.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc_copy.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node2.observation_state == ObservationStateValues.TRUE
        assert outer.inner.observation_state == ObservationStateValues.UNKNOWN
        assert outer.observation_state == ObservationStateValues.UNKNOWN
        assert end.observation_state == ObservationStateValues.UNKNOWN

        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node1.life_cycle_state == LifeCycleValues.DONE
        assert outer.inner.sub_node2.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc_copy.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node2.observation_state == ObservationStateValues.TRUE
        assert outer.inner.observation_state == ObservationStateValues.TRUE
        assert outer.observation_state == ObservationStateValues.UNKNOWN
        assert end.observation_state == ObservationStateValues.UNKNOWN

        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node1.life_cycle_state == LifeCycleValues.DONE
        assert outer.inner.sub_node2.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert not msc_copy.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node2.observation_state == ObservationStateValues.TRUE
        assert outer.inner.observation_state == ObservationStateValues.TRUE
        assert outer.observation_state == ObservationStateValues.TRUE
        assert end.observation_state == ObservationStateValues.UNKNOWN

        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node1.life_cycle_state == LifeCycleValues.DONE
        assert outer.inner.sub_node2.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.RUNNING
        assert not msc_copy.is_end_motion()

        kin_sim.tick()
        assert node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node1.observation_state == ObservationStateValues.TRUE
        assert outer.inner.sub_node2.observation_state == ObservationStateValues.TRUE
        assert outer.inner.observation_state == ObservationStateValues.TRUE
        assert outer.observation_state == ObservationStateValues.TRUE
        assert end.observation_state == ObservationStateValues.TRUE

        assert node1.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.sub_node1.life_cycle_state == LifeCycleValues.DONE
        assert outer.inner.sub_node2.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.inner.life_cycle_state == LifeCycleValues.RUNNING
        assert outer.life_cycle_state == LifeCycleValues.RUNNING
        assert end.life_cycle_state == LifeCycleValues.RUNNING
        assert msc_copy.is_end_motion()


def test_set_seed_configuration(pr2_world_state_reset):
    msc = MotionStatechart()
    goal = 0.1

    connection: ActiveConnection1DOF = pr2_world_state_reset.get_connection_by_name(
        "torso_lift_joint"
    )

    node1 = SetSeedConfiguration(
        seed_configuration=JointState.from_mapping({connection: goal})
    )
    end = EndMotion()
    msc.add_node(node1)
    msc.add_node(end)
    node1.end_condition = node1.observation_variable
    end.start_condition = node1.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)

    kin_sim.tick_until_end()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert node1.life_cycle_state == LifeCycleValues.DONE
    assert end.observation_state == ObservationStateValues.TRUE
    assert end.life_cycle_state == LifeCycleValues.RUNNING

    assert np.isclose(connection.position, goal)


def test_set_seed_odometry(pr2_world_state_reset):
    msc = MotionStatechart()

    goal = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=1,
        y=-1,
        z=1,
        roll=1,
        pitch=1,
        yaw=1,
        reference_frame=pr2_world_state_reset.root,
    )
    expected = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=1, y=-1, yaw=1, reference_frame=pr2_world_state_reset.root
    )

    node1 = SetOdometry(
        base_pose=goal,
    )
    end = EndMotion()
    msc.add_node(node1)
    msc.add_node(end)
    node1.end_condition = node1.observation_variable
    end.start_condition = node1.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)

    kin_sim.tick_until_end()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert node1.life_cycle_state == LifeCycleValues.DONE
    assert end.observation_state == ObservationStateValues.TRUE
    assert end.life_cycle_state == LifeCycleValues.RUNNING

    assert np.allclose(
        expected.to_np(),
        pr2_world_state_reset.compute_forward_kinematics_np(
            pr2_world_state_reset.root, node1.odom_connection.child
        ),
    )


class TestJointTasks:
    def test_joint_goal(self, tmp_path):
        world = World()
        with world.modify_world():
            root = Body(name=PrefixedName("root"))
            tip = Body(name=PrefixedName("tip"))
            tip2 = Body(name=PrefixedName("tip2"))
            ul = DerivativeMap()
            ul.velocity = 1
            ll = DerivativeMap()
            ll.velocity = -1
            dof = DegreeOfFreedom(
                name=PrefixedName("dof", "a"),
                limits=DegreeOfFreedomLimits(lower=ll, upper=ul),
            )
            world.add_degree_of_freedom(dof)
            root_C_tip = RevoluteConnection(
                parent=root, child=tip, axis=Vector3.Z(), dof_id=dof.id
            )
            world.add_connection(root_C_tip)

            dof = DegreeOfFreedom(
                name=PrefixedName("dof", "b"),
                limits=DegreeOfFreedomLimits(lower=ll, upper=ul),
            )
            world.add_degree_of_freedom(dof)
            root_C_tip2 = RevoluteConnection(
                parent=root, child=tip2, axis=Vector3.Z(), dof_id=dof.id
            )
            world.add_connection(root_C_tip2)

        msc = MotionStatechart()

        task1 = JointPositionList(goal_state=JointState.from_mapping({root_C_tip: 1}))
        always_true = ConstTrueNode()
        msc.add_node(always_true)
        msc.add_node(task1)
        end = EndMotion()
        msc.add_node(end)

        task1.start_condition = always_true.observation_variable
        end.start_condition = trinary_logic_and(
            task1.observation_variable, always_true.observation_variable
        )

        kin_sim = Executor(
            MotionStatechartContext(
                world=world,
                qp_controller_config=QPControllerConfig(
                    target_frequency=20,
                    prediction_horizon=7,
                ),
            )
        )
        kin_sim.compile(motion_statechart=msc)

        assert task1.observation_state == ObservationStateValues.UNKNOWN
        assert end.observation_state == ObservationStateValues.UNKNOWN
        assert task1.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
        msc.draw(str(tmp_path / "muh.pdf"))
        kin_sim.tick_until_end()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert len(msc.history) == 6
        assert (
            msc.history.get_observation_history_of_node(task1)[-1]
            == ObservationStateValues.TRUE
        )
        assert (
            msc.history.get_observation_history_of_node(end)[-1]
            == ObservationStateValues.TRUE
        )
        assert (
            msc.history.get_life_cycle_history_of_node(task1)[-1]
            == LifeCycleValues.RUNNING
        )
        assert (
            msc.history.get_life_cycle_history_of_node(end)[-1]
            == LifeCycleValues.RUNNING
        )

    def test_continuous_joint(self, pr2_world_state_reset):
        r_wrist_roll_joint = pr2_world_state_reset.get_connection_by_name(
            "r_wrist_roll_joint"
        )
        l_wrist_roll_joint = pr2_world_state_reset.get_connection_by_name(
            "l_wrist_roll_joint"
        )
        msc = MotionStatechart()
        joint_goal = JointPositionList(
            goal_state=JointState.from_mapping(
                {
                    r_wrist_roll_joint: -np.pi,
                    l_wrist_roll_joint: -2.1 * np.pi,
                },
            ),
        )
        msc.add_node(joint_goal)
        end = EndMotion()
        msc.add_node(end)
        end.start_condition = joint_goal.observation_variable

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        assert np.isclose(
            shortest_angular_distance(r_wrist_roll_joint.position, -np.pi),
            0,
            atol=0.005,
        )
        assert np.isclose(
            shortest_angular_distance(l_wrist_roll_joint.position, -2.1 * np.pi),
            0,
            atol=0.005,
        )

    def test_revolute_joint(self, pr2_world_state_reset):
        head_pan_joint = pr2_world_state_reset.get_connection_by_name("head_pan_joint")
        head_tilt_joint = pr2_world_state_reset.get_connection_by_name(
            "head_tilt_joint"
        )
        msc = MotionStatechart()
        joint_goal = JointPositionList(
            goal_state=JointState.from_mapping(
                {
                    head_pan_joint: 0.042,
                    head_tilt_joint: -0.37,
                },
            ),
        )
        msc.add_node(joint_goal)
        end = EndMotion()
        msc.add_node(end)
        end.start_condition = joint_goal.observation_variable

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        assert np.isclose(head_pan_joint.position, 0.042, atol=1e-3)
        assert np.isclose(head_tilt_joint.position, -0.37, atol=1e-2)

    def test_joint_sequence(self, pr2_world_state_reset):
        msc = MotionStatechart()
        msc.add_node(
            sequence := Sequence(
                [
                    JointPositionList(
                        goal_state=JointState.from_str_dict(
                            {"torso_lift_joint": 0.1}, world=pr2_world_state_reset
                        )
                    ),
                    JointPositionList(
                        goal_state=JointState.from_str_dict(
                            {"torso_lift_joint": 0.2}, world=pr2_world_state_reset
                        )
                    ),
                ]
            )
        )
        msc.add_node(EndMotion.when_true(sequence))

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()


def test_long_goal(pr2_world_state_reset: World):
    msc = MotionStatechart()
    msc.add_nodes(
        [
            cart_goal := CartesianPose(
                root_link=pr2_world_state_reset.root,
                tip_link=pr2_world_state_reset.get_kinematic_structure_entity_by_name(
                    "base_footprint"
                ),
                goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=50, reference_frame=pr2_world_state_reset.root
                ),
            ),
            JointPositionList(
                goal_state=JointState.from_str_dict(
                    {
                        "torso_lift_joint": 0.2999225173357618,
                        "head_pan_joint": 0.042,
                        "head_tilt_joint": -0.37,
                        "r_upper_arm_roll_joint": -0.9487714747527726,
                        "r_shoulder_pan_joint": -1.0047307505973626,
                        "r_shoulder_lift_joint": 0.48736790658811985,
                        "r_forearm_roll_joint": -14.895833882874182,
                        "r_elbow_flex_joint": -1.392377908925028,
                        "r_wrist_flex_joint": -0.4548695149411013,
                        "r_wrist_roll_joint": 0.11426798984097819,
                        "l_upper_arm_roll_joint": 1.7383062350263658,
                        "l_shoulder_pan_joint": 1.8799810286792007,
                        "l_shoulder_lift_joint": 0.011627231224188975,
                        "l_forearm_roll_joint": 312.67276414458695,
                        "l_elbow_flex_joint": -2.0300928925694675,
                        "l_wrist_flex_joint": -0.1,
                        "l_wrist_roll_joint": -6.062015047706399,
                    },
                    world=pr2_world_state_reset,
                )
            ),
        ]
    )
    msc.add_node(EndMotion.when_true(cart_goal))

    kin_sim = Executor(
        MotionStatechartContext(
            world=pr2_world_state_reset,
        )
    )
    kin_sim.compile(motion_statechart=msc)
    t = time.perf_counter()
    kin_sim.tick_until_end(1_000_000)
    after = time.perf_counter()
    diff = after - t
    print(diff / kin_sim.control_cycles)


class TestCartesianPositionTrajectory:

    def _points_to_np(self, positions: list[Point3] | np.ndarray) -> np.ndarray:
        """
        Convert a sequence of `Point3` or an `ndarray` of shape (N, 3) into an `ndarray` of shape (N, 3).
        """
        if isinstance(positions, np.ndarray):
            if positions.ndim != 2 or positions.shape[1] != 3:
                raise ValueError("positions ndarray must have shape (N, 3)")
            return positions.astype(float)
        pts = [
            p.to_np()[:-1] if isinstance(p, Point3) else np.asarray(p, dtype=float)
            for p in positions
        ]
        arr = np.vstack(pts).astype(float)
        if arr.ndim != 2 or arr.shape[1] != 3:
            raise ValueError("positions must convert to shape (N, 3)")
        return arr

    def compare_trajectories(
        self,
        positions: list[Point3] | np.ndarray,
        world_state_trajectory: WorldStateTrajectory,
        root_link: KinematicStructureEntity,
        tip_link: KinematicStructureEntity,
        tolerance: float = 0.01,
    ):
        """
        Compare an executed Cartesian path against a reference list of positions.

        The executed path is reconstructed from the `world_state_trajectory` by computing
        forward kinematics for `tip_link` in the `root_link` frame at each recorded state.
        For each executed point, the minimum Euclidean distance to the reference path is
        computed. Aggregate statistics and pass/fail against `tolerance` are returned.

        :param positions: Reference path as `Point3` iterable or an array of shape (N, 3). All points are with respect to root_link
        :param world_state_trajectory: Recorded joint-space trajectory with access to the world.
        :param root_link: Root kinematic frame for forward kinematics.
        :param tip_link: Tip kinematic frame for forward kinematics.
        :param tolerance: Maximum allowed distance to the reference path for all samples.
        :return: Dictionary containing distances per sample and summary metrics.
        """
        ref_np = self._points_to_np(positions)

        world = world_state_trajectory.world
        executed_points = []

        # Reconstruct executed Cartesian path by FK at each recorded state
        for state_view in world_state_trajectory.values():
            # Temporarily set the world's state to the recorded one
            world.state._data[:] = state_view.data
            world.notify_state_change()
            p = (
                world.compute_forward_kinematics(root_link, tip_link)
                .to_position()
                .evaluate()[:-1]
                .astype(float)
            )
            executed_points.append(p.copy())

        executed_np = np.vstack(executed_points)

        # Distance of each executed point to the nearest reference point
        def _min_dist_to_ref(p: np.ndarray) -> float:
            return float(np.min(np.linalg.norm(executed_np - p, axis=1)))

        distances = np.apply_along_axis(_min_dist_to_ref, 1, ref_np)

        assert np.max(distances) <= tolerance

    def test_cartesian_position_trajectory_spiral(self, cylinder_bot_world: World):
        points = []
        a = 0.05  # spiral growth factor (tunes how fast radius grows)

        for i in range(10000):
            t = (
                i * np.pi / 5000.0
            )  # angle parameter; adjust divisor for tighter/looser turns
            r = a * t  # radius grows linearly with t
            points.append(
                Point3(
                    r * np.cos(t),
                    r * np.sin(t),
                    0,
                    reference_frame=cylinder_bot_world.root,
                )
            )
        msc = MotionStatechart()
        cart_traj = CartesianPositionTrajectory(
            root_link=cylinder_bot_world.root,
            tip_link=cylinder_bot_world.get_kinematic_structure_entity_by_name("bot"),
            goal_points=points,
        )
        msc.add_node(cart_traj)
        msc.add_node(EndMotion.when_true(cart_traj))

        kin_sim = Executor(
            context=MotionStatechartContext(
                world=cylinder_bot_world,
            ),
            trajectory_plotter=WorldStateTrajectoryPlotter(),
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        self.compare_trajectories(
            points,
            kin_sim.trajectory_plotter.world_state_trajectory,
            cart_traj.root_link,
            cart_traj.tip_link,
        )

    def test_cartesian_position_trajectory_circle(self, cylinder_bot_world: World):
        points = []
        a = 0.1

        for i in range(5000):
            t = (
                i * np.pi / 500.0
            )  # angle parameter; adjust divisor for tighter/looser turns
            points.append(
                Point3(
                    a * np.cos(t),
                    a * np.sin(t),
                    0,
                    reference_frame=cylinder_bot_world.root,
                )
            )
        msc = MotionStatechart()
        cart_traj = CartesianPositionTrajectory(
            root_link=cylinder_bot_world.root,
            tip_link=cylinder_bot_world.get_kinematic_structure_entity_by_name("bot"),
            goal_points=points,
            maximum_skip_ahead=20,
        )
        msc.add_node(cart_traj)
        msc.add_node(EndMotion.when_true(cart_traj))

        kin_sim = Executor(
            context=MotionStatechartContext(
                world=cylinder_bot_world,
            ),
            trajectory_plotter=WorldStateTrajectoryPlotter(),
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        self.compare_trajectories(
            points,
            kin_sim.trajectory_plotter.world_state_trajectory,
            cart_traj.root_link,
            cart_traj.tip_link,
        )

    def test_cartesian_position_trajectory_spiral_pr2(
        self, pr2_world_state_reset: World, better_pr2_pose
    ):
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )

        SetSeedConfiguration(
            seed_configuration=JointState.from_str_dict(
                better_pr2_pose, world=pr2_world_state_reset
            )
        ).on_start(MotionStatechartContext(world=pr2_world_state_reset))

        points = []
        root_points = []
        a = 0.05  # spiral growth factor (tunes how fast radius grows)
        for i in range(10000):
            t = (
                i * np.pi / 5000.0
            )  # angle parameter; adjust divisor for tighter/looser turns
            r = a * t  # radius grows linearly with t
            point = Point3(
                r * np.cos(t),
                r * np.sin(t),
                0,
                reference_frame=tip,
            )
            points.append(point)
            root_points.append(pr2_world_state_reset.transform(point, root))
        msc = MotionStatechart()

        msc.add_node(
            cart_traj := CartesianPositionTrajectory(
                root_link=root,
                tip_link=tip,
                goal_points=points,
            )
        )
        msc.add_node(EndMotion.when_true(cart_traj))

        kin_sim = Executor(
            context=MotionStatechartContext(
                world=pr2_world_state_reset,
            ),
            trajectory_plotter=WorldStateTrajectoryPlotter(),
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        self.compare_trajectories(
            root_points,
            kin_sim.trajectory_plotter.world_state_trajectory,
            cart_traj.root_link,
            cart_traj.tip_link,
        )


class TestCartesianTasks:
    """Test suite for all Cartesian motion tasks."""

    def test_simple_cartesian_pose(self, cylinder_bot_world: World):
        tip = cylinder_bot_world.get_kinematic_structure_entity_by_name("bot")

        msc = MotionStatechart()
        msc.add_nodes(
            [
                goal := CartesianPose(
                    root_link=cylinder_bot_world.root,
                    tip_link=tip,
                    goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=1, reference_frame=cylinder_bot_world.root
                    ),
                ),
            ]
        )
        msc.add_node(EndMotion.when_true(goal))

        kin_sim = Executor(MotionStatechartContext(world=cylinder_bot_world))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

    def test_cart_goal_1eef(self, pr2_world_state_reset: World):
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        tip_goal = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=-0.2, reference_frame=tip
        )
        expected = pr2_world_state_reset.transform(tip_goal, root)

        msc = MotionStatechart()
        cart_goal = CartesianPose(
            root_link=root,
            tip_link=tip,
            goal_pose=tip_goal,
        )
        msc.add_node(cart_goal)
        end = EndMotion()
        msc.add_node(end)
        end.start_condition = cart_goal.observation_variable

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert np.allclose(
            kin_sim.context.world.compute_forward_kinematics(root, tip),
            expected,
            atol=cart_goal.threshold,
        )

    def test_front_facing_orientation(self, hsr_world_setup: World):
        """Test combined position and orientation control in parallel."""
        with hsr_world_setup.modify_world():
            box = Body(
                name=PrefixedName("muh"),
                collision=ShapeCollection([Box(scale=Scale(0.1, 0.1, 0.1))]),
            )
            connection = FixedConnection(
                parent=hsr_world_setup.root,
                child=box,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2, z=0.5
                ),
            )
            hsr_world_setup.add_connection(connection)

        hsr = hsr_world_setup.get_semantic_annotations_by_type(HSRB)[0]
        hand = hsr_world_setup.get_semantic_annotations_by_type(Manipulator)[0]
        msc = MotionStatechart()
        orientation_goal = hand.front_facing_orientation.to_rotation_matrix()
        orientation_goal.reference_frame = hsr_world_setup.get_body_by_name(
            "base_footprint"
        )
        msc.add_node(
            goal := Parallel(
                [
                    CartesianOrientation(
                        root_link=hsr_world_setup.root,
                        tip_link=hand.tool_frame,
                        goal_orientation=orientation_goal,
                    ),
                    CartesianPosition(
                        root_link=hsr_world_setup.root,
                        tip_link=hand.tool_frame,
                        goal_point=hsr_world_setup.bodies[
                            -1
                        ].global_transform.to_position(),
                    ),
                ]
            )
        )
        msc.add_node(EndMotion.when_true(goal))

        kin_sim = Executor(MotionStatechartContext(world=hsr_world_setup))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

    def test_cart_goal_sequence_at_build(self, pr2_world_state_reset: World):
        """
        Test CartesianPose sequence with Bind_at_build policy.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        tip_goal1 = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=-2, reference_frame=tip
        )
        tip_goal2 = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=0.2, reference_frame=tip
        )

        msc = MotionStatechart()
        cart_goal1 = CartesianPose(
            root_link=root,
            tip_link=tip,
            goal_pose=tip_goal1,
        )
        msc.add_node(cart_goal1)

        cart_goal2 = CartesianPose(
            root_link=root,
            tip_link=tip,
            goal_pose=tip_goal2,
            binding_policy=GoalBindingPolicy.Bind_at_build,
        )
        msc.add_node(cart_goal2)

        cart_goal1.end_condition = cart_goal1.observation_variable
        cart_goal2.start_condition = cart_goal1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = trinary_logic_and(
            cart_goal1.observation_variable, cart_goal2.observation_variable
        )

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )

        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)
        assert np.allclose(fk, tip_goal2.to_np(), atol=cart_goal2.threshold)

    def test_cart_goal_sequence_on_start(self, pr2_world_state_reset: World):
        """
        Test CartesianPose sequence with Bind_on_start policy (default).
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        tip_goal1 = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=-0.2, reference_frame=tip
        )
        tip_goal2 = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=0.2, reference_frame=tip
        )

        msc = MotionStatechart()
        cart_goal1 = CartesianPose(
            root_link=root,
            tip_link=tip,
            goal_pose=tip_goal1,
        )
        msc.add_node(cart_goal1)

        cart_goal2 = CartesianPose(
            root_link=root,
            tip_link=tip,
            goal_pose=tip_goal2,
        )
        msc.add_node(cart_goal2)

        cart_goal1.end_condition = cart_goal1.observation_variable
        cart_goal2.start_condition = cart_goal1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = trinary_logic_and(
            cart_goal1.observation_variable, cart_goal2.observation_variable
        )

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)
        expected = np.eye(4)
        assert np.allclose(fk, expected, atol=cart_goal2.threshold)

    def test_CartesianOrientation(self, pr2_world_state_reset: World):
        """Test basic CartesianOrientation goal."""
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        tip_goal = RotationMatrix.from_axis_angle(Vector3.Z(), 4.0, reference_frame=tip)

        msc = MotionStatechart()
        cart_goal = CartesianOrientation(
            root_link=root,
            tip_link=tip,
            goal_orientation=tip_goal,
        )
        msc.add_node(cart_goal)
        end = EndMotion()
        msc.add_node(end)
        end.start_condition = cart_goal.observation_variable

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)
        assert np.allclose(fk, tip_goal.to_np(), atol=cart_goal.threshold)

    def test_cartesian_position_sequence_at_build(self, pr2_world_state_reset: World):
        """
        Test CartesianPosition with Bind_at_build policy.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        tip_goal1 = Point3(-0.2, 0, 0, reference_frame=tip)
        tip_goal2 = Point3(0.2, 0, 0, reference_frame=tip)

        msc = MotionStatechart()
        cart_goal1 = CartesianPosition(
            root_link=root,
            tip_link=tip,
            goal_point=tip_goal1,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(cart_goal1)

        cart_goal2 = CartesianPosition(
            root_link=root,
            tip_link=tip,
            goal_point=tip_goal2,
            binding_policy=GoalBindingPolicy.Bind_at_build,
        )
        msc.add_node(cart_goal2)

        cart_goal1.end_condition = cart_goal1.observation_variable
        cart_goal2.start_condition = cart_goal1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = trinary_logic_and(
            cart_goal1.observation_variable, cart_goal2.observation_variable
        )

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)
        # goal2 was captured at build time, so should end at that absolute position
        expected = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=0.2, reference_frame=pr2_world_state_reset.root
        ).to_np()
        assert np.allclose(fk[:3, 3], expected[:3, 3], atol=cart_goal2.threshold)

    def test_cartesian_position_sequence_on_start(self, pr2_world_state_reset: World):
        """
        Test CartesianPosition with Bind_on_start policy (default).
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        tip_goal1 = Point3(-0.2, 0, 0, reference_frame=tip)
        tip_goal2 = Point3(0.2, 0, 0, reference_frame=tip)

        msc = MotionStatechart()
        cart_goal1 = CartesianPosition(
            root_link=root,
            tip_link=tip,
            goal_point=tip_goal1,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(cart_goal1)

        cart_goal2 = CartesianPosition(
            root_link=root,
            tip_link=tip,
            goal_point=tip_goal2,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(cart_goal2)

        cart_goal1.end_condition = cart_goal1.observation_variable
        cart_goal2.start_condition = cart_goal1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = trinary_logic_and(
            cart_goal1.observation_variable, cart_goal2.observation_variable
        )

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)
        # Both goals captured when tasks start, so should return near origin
        expected = np.eye(4)
        assert np.allclose(fk[:3, 3], expected[:3, 3], atol=cart_goal2.threshold)

    def test_cartesian_position_with_sequence_node(self, pr2_world_state_reset: World):
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        tip_goal1 = Point3(-0.2, 0, 0, reference_frame=tip)
        tip_goal2 = Point3(0.2, 0, 0, reference_frame=tip)

        msc = MotionStatechart()
        cart_goal1 = CartesianPosition(
            root_link=root,
            tip_link=tip,
            goal_point=tip_goal1,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )

        cart_goal2 = CartesianPosition(
            root_link=root,
            tip_link=tip,
            goal_point=tip_goal2,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(seq := Sequence(nodes=[cart_goal1, cart_goal2]))

        msc.add_node(EndMotion.when_true(seq))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)
        # Both goals captured when tasks start, so should return near origin
        expected = np.eye(4)
        assert np.allclose(fk[:3, 3], expected[:3, 3], atol=cart_goal2.threshold)

    def test_cartesian_orientation_sequence_at_build(
        self, pr2_world_state_reset: World
    ):
        """
        Test CartesianOrientation with Bind_at_build policy.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        # Store initial orientation for comparison
        initial_fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)

        tip_rot1 = RotationMatrix.from_axis_angle(
            Vector3.Z(), np.pi / 6, reference_frame=tip
        )
        tip_rot2 = RotationMatrix.from_axis_angle(
            Vector3.Z(), -np.pi / 6, reference_frame=tip
        )

        msc = MotionStatechart()
        cart_goal1 = CartesianOrientation(
            root_link=root,
            tip_link=tip,
            goal_orientation=tip_rot1,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(cart_goal1)

        cart_goal2 = CartesianOrientation(
            root_link=root,
            tip_link=tip,
            goal_orientation=tip_rot2,
            binding_policy=GoalBindingPolicy.Bind_at_build,
        )
        msc.add_node(cart_goal2)

        cart_goal1.end_condition = cart_goal1.observation_variable
        cart_goal2.start_condition = cart_goal1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = trinary_logic_and(
            cart_goal1.observation_variable, cart_goal2.observation_variable
        )

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)

        # goal2 captured at build, so ends at -pi/6 from original
        expected = tip_rot2.to_np()
        assert np.allclose(fk, expected, atol=cart_goal2.threshold)

    def test_cartesian_orientation_sequence_on_start(
        self, pr2_world_state_reset: World
    ):
        """
        Test CartesianOrientation with Bind_on_start policy (default).
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        tip_rot1 = RotationMatrix.from_axis_angle(
            Vector3.Z(), np.pi / 6, reference_frame=tip
        )
        tip_rot2 = RotationMatrix.from_axis_angle(
            Vector3.Z(), -np.pi / 6, reference_frame=tip
        )

        msc = MotionStatechart()
        cart_goal1 = CartesianOrientation(
            root_link=root,
            tip_link=tip,
            goal_orientation=tip_rot1,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(cart_goal1)

        cart_goal2 = CartesianOrientation(
            root_link=root,
            tip_link=tip,
            goal_orientation=tip_rot2,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(cart_goal2)

        cart_goal1.end_condition = cart_goal1.observation_variable
        cart_goal2.start_condition = cart_goal1.observation_variable

        end = EndMotion()
        msc.add_node(end)
        end.start_condition = trinary_logic_and(
            cart_goal1.observation_variable, cart_goal2.observation_variable
        )

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        fk = pr2_world_state_reset.compute_forward_kinematics_np(root, tip)
        # Both goals captured when tasks start, so rotates +pi/6 then -pi/6 = back to origin
        expected = np.eye(4)
        assert np.allclose(fk, expected, atol=cart_goal2.threshold)

    def test_cartesian_position_straight(self, pr2_world_state_reset: World):
        """
        Test CartesianPositionStraight basic functionality.

        Verifies that the tip reaches the goal and (ideally) follows a straight path.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        goal_point = Point3(0.1, 0, 0, reference_frame=tip)

        msc = MotionStatechart()
        cart_straight = CartesianPositionStraight(
            root_link=root,
            tip_link=tip,
            goal_point=goal_point,
            binding_policy=GoalBindingPolicy.Bind_on_start,
            threshold=0.015,
        )
        msc.add_node(cart_straight)
        end = EndMotion()
        msc.add_node(end)
        end.start_condition = cart_straight.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        # Verify goal was achieved
        assert cart_straight.observation_state == ObservationStateValues.TRUE

    def test_cartesian_pose_straight(self, pr2_world_state_reset: World):
        """Test CartesianPositionStraight basic functionality."""
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        goal_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
            0.1, 2, 0, reference_frame=tip
        )

        msc = MotionStatechart()
        cart_straight = CartesianPoseStraight(
            root_link=root,
            tip_link=tip,
            goal_pose=goal_pose,
            binding_policy=GoalBindingPolicy.Bind_on_start,
        )
        msc.add_node(cart_straight)
        msc.add_node(EndMotion.when_true(cart_straight))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        # Verify task detected completion
        assert cart_straight.observation_state == ObservationStateValues.TRUE

        assert np.allclose(
            cart_straight.goal_pose.to_np(), goal_pose.to_np(), atol=0.015
        )


class TestDiffDriveBaseGoal:
    @pytest.mark.parametrize(
        "goal_pose",
        [
            Pose.from_xyz_rpy(x=0.489, y=-0.598, z=0.000),
            Pose.from_xyz_quaternion(
                pos_x=-0.026,
                pos_y=0.569,
                pos_z=0.0,
                quat_x=0.0,
                quat_y=0.0,
                quat_z=0.916530200374776,
                quat_w=0.3999654882623912,
            ),
            Pose.from_xyz_rpy(x=1, y=1, yaw=np.pi / 4),
            Pose.from_xyz_rpy(x=2, y=0, yaw=-np.pi / 4),
            Pose.from_xyz_rpy(yaw=-np.pi / 4),
            Pose.from_xyz_rpy(x=-1, y=-1, yaw=np.pi / 4),
            Pose.from_xyz_rpy(x=-2, y=-1, yaw=-np.pi / 4),
            Pose.from_xyz_rpy(x=0.01, y=0.5, yaw=np.pi / 8),
            Pose.from_xyz_rpy(x=-0.01, y=-0.5, yaw=np.pi / 5),
            Pose.from_xyz_rpy(x=1.1, y=2.0, yaw=-np.pi),
            Pose.from_xyz_rpy(y=1),
        ],
    )
    def test_drive(
        self,
        cylinder_bot_diff_world,
        goal_pose: Pose,
    ):
        bot = cylinder_bot_diff_world.get_body_by_name("bot")
        msc = MotionStatechart()
        goal_pose.reference_frame = cylinder_bot_diff_world.root
        msc.add_node(goal := DifferentialDriveBaseGoal(goal_pose=goal_pose))
        msc.add_node(EndMotion.when_true(goal))

        kin_sim = Executor(MotionStatechartContext(world=cylinder_bot_diff_world))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert np.allclose(
            cylinder_bot_diff_world.compute_forward_kinematics(
                cylinder_bot_diff_world.root, bot
            ),
            goal_pose,
            atol=1e-2,
        )


class TestFeatureFunctions:
    """Test suite for feature function tasks (HeightGoal, DistanceGoal, etc.)."""

    def test_height_goal_within_bounds(self, pr2_world_state_reset: World):
        """
        Test that HeightGoal successfully constrains the vertical distance
        between tip and reference points within specified bounds.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )

        tip_point = Point3(0, 0, 0, reference_frame=tip)
        reference_point = Point3(0, 0, 0, reference_frame=root)

        lower_limit = 0.3
        upper_limit = 0.5

        msc = MotionStatechart()
        height_goal = HeightGoal(
            root_link=root,
            tip_link=tip,
            tip_point=tip_point,
            reference_point=reference_point,
            lower_limit=lower_limit,
            upper_limit=upper_limit,
        )
        msc.add_node(height_goal)
        msc.add_node(EndMotion.when_true(height_goal))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert height_goal.observation_state == ObservationStateValues.TRUE

        # Compute actual height difference
        root_P_tip = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_point
        )
        root_P_ref = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_point
        )
        height_diff = (root_P_tip.to_np()[:3] - root_P_ref.to_np()[:3])[2]

        assert (
            lower_limit <= height_diff <= upper_limit
        ), f"Height {height_diff:.4f} not in [{lower_limit}, {upper_limit}]"

    def test_height_goal_negative_bounds(self, pr2_world_state_reset: World):
        """
        Test HeightGoal with negative height bounds (tip below reference).
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )

        tip_point = Point3(0, 0, 0, reference_frame=tip)
        reference_point = Point3(0, 0, 1.0, reference_frame=root)

        lower_limit = -0.5
        upper_limit = -0.2

        msc = MotionStatechart()
        height_goal = HeightGoal(
            root_link=root,
            tip_link=tip,
            tip_point=tip_point,
            reference_point=reference_point,
            lower_limit=lower_limit,
            upper_limit=upper_limit,
        )
        msc.add_node(height_goal)
        msc.add_node(EndMotion.when_true(height_goal))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert height_goal.observation_state == ObservationStateValues.TRUE

        # Verify actual height difference
        root_P_tip = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_point
        )
        root_P_ref = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_point
        )
        height_diff = (root_P_tip.to_np()[:3] - root_P_ref.to_np()[:3])[2]

        assert (
            lower_limit <= height_diff <= upper_limit
        ), f"Height {height_diff:.4f} not in [{lower_limit}, {upper_limit}]"

    def test_distance_goal_within_bounds(self, pr2_world_state_reset: World):
        """
        Test that DistanceGoal successfully constrains the horizontal distance
        (in x-y plane) between tip and reference points within specified bounds.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )

        tip_point = Point3(0, 0, 0, reference_frame=tip)
        reference_point = Point3(0, 0, 0, reference_frame=root)

        lower_limit = 0.5
        upper_limit = 0.7

        msc = MotionStatechart()
        distance_goal = DistanceGoal(
            root_link=root,
            tip_link=tip,
            tip_point=tip_point,
            reference_point=reference_point,
            lower_limit=lower_limit,
            upper_limit=upper_limit,
        )
        msc.add_node(distance_goal)
        msc.add_node(EndMotion.when_true(distance_goal))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert distance_goal.observation_state == ObservationStateValues.TRUE

        # Compute actual horizontal distance
        root_P_tip = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_point
        )
        root_P_ref = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_point
        )
        diff = root_P_tip.to_np()[:3] - root_P_ref.to_np()[:3]
        # Distance in x-y plane only (ignore z)
        horizontal_distance = np.sqrt(diff[0] ** 2 + diff[1] ** 2)

        assert (
            lower_limit <= horizontal_distance <= upper_limit
        ), f"Distance {horizontal_distance:.4f} not in [{lower_limit}, {upper_limit}]"

    def test_distance_goal_zero_distance(self, pr2_world_state_reset: World):
        """
        Test DistanceGoal with bounds that include zero (tip and reference at same x-y position).
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )

        tip_point = Point3(0, 0, 0, reference_frame=tip)
        # Reference point at same x-y but different z
        reference_point = Point3(0, 0, 0.5, reference_frame=root)

        lower_limit = 0.0
        upper_limit = 0.1

        msc = MotionStatechart()
        distance_goal = DistanceGoal(
            root_link=root,
            tip_link=tip,
            tip_point=tip_point,
            reference_point=reference_point,
            lower_limit=lower_limit,
            upper_limit=upper_limit,
        )
        msc.add_node(distance_goal)
        msc.add_node(EndMotion.when_true(distance_goal))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert distance_goal.observation_state == ObservationStateValues.TRUE

        # Verify horizontal distance is near zero
        root_P_tip = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_point
        )
        root_P_ref = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_point
        )
        diff = root_P_tip.to_np()[:3] - root_P_ref.to_np()[:3]
        horizontal_distance = np.sqrt(diff[0] ** 2 + diff[1] ** 2)

        assert (
            lower_limit <= horizontal_distance <= upper_limit
        ), f"Distance {horizontal_distance:.4f} not in [{lower_limit}, {upper_limit}]"

    def test_distance_goal_ignores_z_axis(self, pr2_world_state_reset: World):
        """
        Test that DistanceGoal only considers x-y plane distance and ignores z-axis.
        Even with large z difference, if x-y distance is within bounds, goal succeeds.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )

        tip_point = Point3(0, 0, 0, reference_frame=tip)
        # Reference point at specific x-y position but very different z
        reference_point = Point3(0.3, 0.4, 2.0, reference_frame=root)

        lower_limit = 0.45
        upper_limit = 0.55

        msc = MotionStatechart()
        distance_goal = DistanceGoal(
            root_link=root,
            tip_link=tip,
            tip_point=tip_point,
            reference_point=reference_point,
            lower_limit=lower_limit,
            upper_limit=upper_limit,
        )
        msc.add_node(distance_goal)
        msc.add_node(EndMotion.when_true(distance_goal))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert distance_goal.observation_state == ObservationStateValues.TRUE

        # Verify z difference is large but goal still succeeded
        root_P_tip = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_point
        )
        root_P_ref = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_point
        )
        diff = root_P_tip.to_np()[:3] - root_P_ref.to_np()[:3]

        horizontal_distance = np.sqrt(diff[0] ** 2 + diff[1] ** 2)

        assert (
            lower_limit <= horizontal_distance <= upper_limit
        ), f"Distance {horizontal_distance:.4f} not in [{lower_limit}, {upper_limit}]"

    def test_height_and_distance_combined(self, pr2_world_state_reset: World):
        """
        Test combining HeightGoal and DistanceGoal in parallel to constrain
        both vertical and horizontal distances simultaneously.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )

        tip_point = Point3(0, 0, 0, reference_frame=tip)
        reference_point = Point3(0, 0, 0, reference_frame=root)

        height_lower = 0.3
        height_upper = 0.5

        distance_lower = 0.4
        distance_upper = 0.6

        msc = MotionStatechart()
        combined_goal = Parallel(
            [
                HeightGoal(
                    root_link=root,
                    tip_link=tip,
                    tip_point=tip_point,
                    reference_point=reference_point,
                    lower_limit=height_lower,
                    upper_limit=height_upper,
                ),
                DistanceGoal(
                    root_link=root,
                    tip_link=tip,
                    tip_point=tip_point,
                    reference_point=reference_point,
                    lower_limit=distance_lower,
                    upper_limit=distance_upper,
                ),
            ]
        )
        msc.add_node(combined_goal)
        msc.add_node(EndMotion.when_true(combined_goal))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert combined_goal.observation_state == ObservationStateValues.TRUE

        # Verify both constraints are satisfied
        root_P_tip = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_point
        )
        root_P_ref = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_point
        )
        diff = root_P_tip.to_np()[:3] - root_P_ref.to_np()[:3]

        height_diff = diff[2]
        horizontal_distance = np.sqrt(diff[0] ** 2 + diff[1] ** 2)

        assert (
            height_lower <= height_diff <= height_upper
        ), f"Height {height_diff:.4f} not in [{height_lower}, {height_upper}]"
        assert (
            distance_lower <= horizontal_distance <= distance_upper
        ), f"Distance {horizontal_distance:.4f} not in [{distance_lower}, {distance_upper}]"

    def test_distance_height_angle_perpendicular_combined(
        self, pr2_world_state_reset: World
    ):
        """
        Test combining DistanceGoal, HeightGoal, and AlignPerpendicular
        to constrain horizontal distance, vertical distance, and perpendicular
        alignment simultaneously.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )

        tip_point = Point3(0, 0, 0, reference_frame=tip)
        reference_point = Point3(0, 0, 0, reference_frame=root)

        tip_normal = Vector3(1, 0, 0, reference_frame=tip)
        reference_normal = Vector3(1, 0, 0, reference_frame=root)

        height_lower = 0.3
        height_upper = 0.5

        distance_lower = 0.4
        distance_upper = 0.6

        perpendicular_threshold = 0.01

        msc = MotionStatechart()
        height_goal = HeightGoal(
            root_link=root,
            tip_link=tip,
            tip_point=tip_point,
            reference_point=reference_point,
            lower_limit=height_lower,
            upper_limit=height_upper,
        )
        distance_goal = DistanceGoal(
            root_link=root,
            tip_link=tip,
            tip_point=tip_point,
            reference_point=reference_point,
            lower_limit=distance_lower,
            upper_limit=distance_upper,
        )
        align_perpendicular = AlignPerpendicular(
            root_link=root,
            tip_link=tip,
            tip_normal=tip_normal,
            reference_normal=reference_normal,
            threshold=perpendicular_threshold,
        )

        combined_goal = Parallel([height_goal, distance_goal, align_perpendicular])
        msc.add_node(combined_goal)
        msc.add_node(EndMotion.when_true(combined_goal))

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert combined_goal.observation_state == ObservationStateValues.TRUE
        assert height_goal.observation_state == ObservationStateValues.TRUE
        assert distance_goal.observation_state == ObservationStateValues.TRUE
        assert align_perpendicular.observation_state == ObservationStateValues.TRUE

        root_P_tip = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_point
        )
        root_P_ref = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_point
        )
        diff = root_P_tip.to_np()[:3] - root_P_ref.to_np()[:3]

        height_diff = diff[2]
        horizontal_distance = np.sqrt(diff[0] ** 2 + diff[1] ** 2)

        assert (
            height_lower <= height_diff <= height_upper
        ), f"Height {height_diff:.4f} not in [{height_lower}, {height_upper}]"
        assert (
            distance_lower <= horizontal_distance <= distance_upper
        ), f"Distance {horizontal_distance:.4f} not in [{distance_lower}, {distance_upper}]"

        root_V_tip_normal = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=tip_normal
        )
        root_V_tip_normal.scale(1)
        root_V_ref_normal = pr2_world_state_reset.transform(
            target_frame=root, spatial_object=reference_normal
        )
        root_V_ref_normal.scale(1)

        v_tip_normal = root_V_tip_normal.to_np()[:3]
        v_ref_normal = root_V_ref_normal.to_np()[:3]

        eps = 1e-9
        assert np.linalg.norm(v_ref_normal) > eps, "reference normal became zero-length"
        assert np.linalg.norm(v_tip_normal) > eps, "tip normal became zero-length"

        perp_angle = angle_between_vector(v_tip_normal, v_ref_normal)
        target = np.pi / 2

        assert abs(perp_angle - target) <= perpendicular_threshold, (
            f"AlignPerpendicular failed: final angle {perp_angle:.6f} rad, "
            f"target {target:.6f} rad, threshold {perpendicular_threshold:.6f} rad"
        )


def test_pointing(pr2_world_state_reset: World):
    tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
        "r_gripper_tool_frame"
    )
    root = pr2_world_state_reset.get_kinematic_structure_entity_by_name("odom_combined")

    msc = MotionStatechart()

    goal_point = Point3(2, 0, 0, reference_frame=root)
    pointing_axis = Vector3.X(reference_frame=tip)

    pointing = Pointing(
        root_link=root,
        tip_link=tip,
        goal_point=goal_point,
        pointing_axis=pointing_axis,
    )
    msc.add_node(pointing)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = pointing.observation_variable

    kin_sim = Executor(
        MotionStatechartContext(
            world=pr2_world_state_reset,
        )
    )
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()


def test_pointing_cone(pr2_world_state_reset: World):
    tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
        "r_gripper_tool_frame"
    )
    root = pr2_world_state_reset.get_kinematic_structure_entity_by_name("odom_combined")

    msc = MotionStatechart()

    goal_point = Point3(-1, 0, 5, reference_frame=root)
    pointing_axis = Vector3.X(tip)
    cone_theta = radians(20)
    pointing_cone = PointingCone(
        root_link=root,
        tip_link=tip,
        goal_point=goal_point,
        pointing_axis=pointing_axis,
        cone_theta=cone_theta,
    )
    msc.add_node(pointing_cone)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = pointing_cone.observation_variable

    kin_sim = Executor(
        MotionStatechartContext(
            world=pr2_world_state_reset,
        )
    )
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    # Check if angle between pointing axis and tip->goal vector is within the cone
    root_V_pointing_axis = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=pointing_axis
    )
    root_V_pointing_axis.scale(1)
    v_pointing = root_V_pointing_axis.to_np()[:3]

    root_P_goal = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=goal_point
    )
    tip_origin_in_root = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=Point3(0, 0, 0, reference_frame=tip)
    )
    root_V_goal_axis = root_P_goal - tip_origin_in_root
    root_V_goal_axis.scale(1)
    v_goal = root_V_goal_axis.to_np()[:3]

    eps = 1e-9
    assert np.linalg.norm(v_pointing) > eps, "pointing axis became zero-length"
    assert np.linalg.norm(v_goal) > eps, "tip->goal vector became zero-length"

    angle = angle_between_vector(v_pointing, v_goal)

    assert (
        angle <= cone_theta + pointing_cone.threshold
    ), f"PointingCone failed: final angle {angle:.6f} rad > cone_theta {cone_theta:.6f} rad"


def test_align_planes(pr2_world_state_reset: World):
    tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
        "r_gripper_tool_frame"
    )
    root = pr2_world_state_reset.get_kinematic_structure_entity_by_name("odom_combined")

    msc = MotionStatechart()

    goal_normal = Vector3.X(reference_frame=root)
    tip_normal = Vector3.Y(reference_frame=tip)

    align_planes = AlignPlanes(
        root_link=root, tip_link=tip, goal_normal=goal_normal, tip_normal=tip_normal
    )
    msc.add_node(align_planes)

    end = EndMotion()
    msc.add_node(end)
    end.start_condition = align_planes.observation_variable

    kin_sim = Executor(
        MotionStatechartContext(
            world=pr2_world_state_reset,
        )
    )
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    # Check if the angle between normal vectors is below the threshold
    root_V_goal_normal = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=goal_normal
    )
    root_V_goal_normal.scale(1)
    root_V_tip_normal = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=tip_normal
    )
    root_V_tip_normal.scale(1)
    v_tip = root_V_tip_normal.to_np()[:3]
    v_goal = root_V_goal_normal.to_np()[:3]

    eps = 1e-9
    assert np.linalg.norm(v_goal) > eps, "goal normal became zero-length"
    assert np.linalg.norm(v_tip) > eps, "tip normal became zero-length"

    angle = angle_between_vector(v_tip, v_goal)

    assert (
        angle <= align_planes.threshold
    ), f"AlignPlanes failed: final angle {angle:.6f} rad > threshold {align_planes.threshold:.6f} rad"


def test_align_perpendicular(pr2_world_state_reset: World):
    tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
        "r_gripper_tool_frame"
    )
    root = pr2_world_state_reset.get_kinematic_structure_entity_by_name("odom_combined")

    msc = MotionStatechart()

    goal_normal = Vector3.X(reference_frame=root)
    tip_normal = Vector3.X(reference_frame=tip)

    align_perp = AlignPerpendicular(
        root_link=root,
        tip_link=tip,
        reference_normal=goal_normal,
        tip_normal=tip_normal,
    )
    msc.add_node(align_perp)

    end = EndMotion()
    msc.add_node(end)
    end.start_condition = align_perp.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    # Check if the angle between normals is (approximately) 90 degrees
    root_V_goal_normal = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=goal_normal
    )
    root_V_goal_normal.scale(1)
    root_V_tip_normal = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=tip_normal
    )
    root_V_tip_normal.scale(1)

    v_tip = root_V_tip_normal.to_np()[:3]
    v_goal = root_V_goal_normal.to_np()[:3]

    eps = 1e-9
    assert np.linalg.norm(v_goal) > eps, "goal normal became zero-length"
    assert np.linalg.norm(v_tip) > eps, "tip normal became zero-length"

    angle = angle_between_vector(v_tip, v_goal)
    target = np.pi / 2

    assert abs(angle - target) <= align_perp.threshold, (
        f"AlignPerpendicular failed: final angle {angle:.6f} rad, "
        f"target {target:.6f} rad, threshold {align_perp.threshold:.6f} rad"
    )


def test_angle_goal(pr2_world_state_reset: World):
    """
    Ensure AngleGoal drives the angle between tip_vector and reference_vector
    into the interval [lower_angle, upper_angle].
    """
    tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
        "r_gripper_tool_frame"
    )
    root = pr2_world_state_reset.get_kinematic_structure_entity_by_name("odom_combined")

    msc = MotionStatechart()

    tip_vector = Vector3.Y(reference_frame=tip)
    reference_vector = Vector3.X(reference_frame=root)

    lower_angle = radians(30)
    upper_angle = radians(32)

    angle_goal = AngleGoal(
        root_link=root,
        tip_link=tip,
        tip_vector=tip_vector,
        reference_vector=reference_vector,
        lower_angle=lower_angle,
        upper_angle=upper_angle,
    )
    msc.add_node(angle_goal)

    msc.add_node(EndMotion.when_true(angle_goal))

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    root_V_tip = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=tip_vector
    )
    root_V_tip.scale(1)
    root_V_ref = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=reference_vector
    )
    root_V_ref.scale(1)

    v_tip = root_V_tip.to_np()[:3]
    v_ref = root_V_ref.to_np()[:3]

    eps = 1e-9
    assert np.linalg.norm(v_tip) > eps, "tip_vector became zero-length"
    assert np.linalg.norm(v_ref) > eps, "reference_vector became zero-length"

    angle = angle_between_vector(v_tip, v_ref)

    assert (
        lower_angle <= angle <= upper_angle
    ), f"AngleGoal failed: final angle {angle:.6f} rad not in [{lower_angle:.6f}, {upper_angle:.6f}]"


class TestVelocityTasks:
    def _build_msc(self, goal_node, limit_node) -> MotionStatechart:
        """
        Build a small MSC: goal_node -> limit_node -> EndMotion(when_true=goal_node)
        Returns the MotionStatechart but does not compile or run it.
        """
        msc = MotionStatechart()
        msc.add_node(goal_node)
        msc.add_node(limit_node)
        msc.add_node(EndMotion.when_true(goal_node))
        return msc

    def _compile_msc_and_run_until_end(self, world: World, goal_node, limit_node):
        """
        Build the MSC (no extra nodes), compile into an Executor,
        run until end and return (control_cycles, executor)
        """
        msc = self._build_msc(goal_node=goal_node, limit_node=limit_node)
        kin_sim = Executor(MotionStatechartContext(world=world))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        return kin_sim.control_cycles, kin_sim

    @pytest.mark.parametrize(
        "goal_type, limit_cls",
        [
            ("position", CartesianVelocityLimit),
            ("position", CartesianPositionVelocityLimit),
            ("rotation", CartesianVelocityLimit),
            ("rotation", CartesianRotationVelocityLimit),
        ],
        ids=["pos/generic", "pos/position-only", "rot/generic", "rot/rotation-only"],
    )
    def test_observation_variable(
        self, pr2_world_state_reset: World, goal_type: str, limit_cls: Type
    ):
        """
        Tests that velocity limit's observation variable can trigger a CancelMotion
        when the optimizer chooses to violate the limit.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        if goal_type == "position":
            goal = CartesianPosition(
                root_link=root,
                tip_link=tip,
                goal_point=Point3(1, 0, 0, reference_frame=tip),
                weight=DefaultWeights.WEIGHT_ABOVE_CA,
            )
        else:
            goal = CartesianOrientation(
                root_link=root,
                tip_link=tip,
                goal_orientation=RotationMatrix.from_rpy(
                    yaw=np.pi / 2, reference_frame=tip
                ),
                weight=DefaultWeights.WEIGHT_ABOVE_CA,
            )

        low_weight_limit = limit_cls(
            root_link=root, tip_link=tip, weight=DefaultWeights.WEIGHT_BELOW_CA
        )
        msc = self._build_msc(goal_node=goal, limit_node=low_weight_limit)
        cancel_motion = CancelMotion(exception=Exception("test"))
        cancel_motion.start_condition = trinary_logic_not(
            low_weight_limit.observation_variable
        )
        msc.add_node(cancel_motion)

        kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
        kin_sim.compile(motion_statechart=msc)

        with pytest.raises(Exception):
            kin_sim.tick_until_end()

    @pytest.mark.parametrize(
        "limit_cls",
        [CartesianVelocityLimit, CartesianPositionVelocityLimit],
        ids=["generic_linear", "position_only_linear"],
    )
    def test_cartesian_position_velocity_limit(
        self, pr2_world_state_reset: World, limit_cls: Type
    ):
        """
        Position velocity limit: check slower limit increases cycles.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        point = Point3(1, 0, 0, reference_frame=tip)
        position_goal = CartesianPosition(
            root_link=root, tip_link=tip, goal_point=point
        )

        usual_limit = limit_cls(root_link=root, tip_link=tip, max_linear_velocity=0.1)
        half_velocity_limit = limit_cls(
            root_link=root,
            tip_link=tip,
            max_linear_velocity=(usual_limit.max_linear_velocity / 2.1),
        )

        loose_cycles, _ = self._compile_msc_and_run_until_end(
            world=pr2_world_state_reset, goal_node=position_goal, limit_node=usual_limit
        )
        tight_cycles, _ = self._compile_msc_and_run_until_end(
            world=pr2_world_state_reset,
            goal_node=position_goal,
            limit_node=half_velocity_limit,
        )

        assert (
            tight_cycles >= 2 * loose_cycles
        ), f"tight ({tight_cycles}) should take >= loose ({2 * loose_cycles}) control cycles"

    @pytest.mark.parametrize(
        "limit_cls",
        [CartesianVelocityLimit, CartesianRotationVelocityLimit],
        ids=["generic_angular", "rotation_only_angular"],
    )
    def test_cartesian_rotation_velocity_limit(
        self, pr2_world_state_reset: World, limit_cls: Type
    ):
        """
        Rotation velocity limit: check slower limit increases cycles.
        """
        tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        root = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "odom_combined"
        )

        rotation = RotationMatrix.from_rpy(yaw=np.pi / 2, reference_frame=tip)
        orientation = CartesianOrientation(
            root_link=root, tip_link=tip, goal_orientation=rotation
        )

        usual_limit = limit_cls(root_link=root, tip_link=tip, max_angular_velocity=0.3)
        half_velocity_limit = limit_cls(
            root_link=root,
            tip_link=tip,
            max_angular_velocity=(usual_limit.max_angular_velocity / 2.1),
        )

        loose_cycles, _ = self._compile_msc_and_run_until_end(
            world=pr2_world_state_reset, goal_node=orientation, limit_node=usual_limit
        )
        tight_cycles, _ = self._compile_msc_and_run_until_end(
            world=pr2_world_state_reset,
            goal_node=orientation,
            limit_node=half_velocity_limit,
        )

        assert (
            tight_cycles >= 2 * loose_cycles
        ), f"tight ({tight_cycles}) should take >= loose ({2 * loose_cycles}) control cycles"


def test_counting():
    class FakeClock:
        def __init__(self):
            self.t = 0.0

        def time(self):
            return self.t

        def advance(self, dt):
            self.t += dt

    clock = FakeClock()

    msc = MotionStatechart()
    seconds = 1
    msc.add_nodes(
        [counter := CountSeconds(seconds=seconds, _now=clock.time), pulse := Pulse()]
    )

    pulse.start_condition = counter.observation_variable
    counter.reset_condition = pulse.observation_variable

    msc.add_node(end := EndMotion())

    end.start_condition = trinary_logic_and(
        counter.observation_variable, trinary_logic_not(pulse.observation_variable)
    )

    kin_sim = Executor(
        MotionStatechartContext(
            world=World(),
        )
    )
    kin_sim.compile(motion_statechart=msc)

    # Advance fake time deterministically without wall-clock sleeps
    step = 0.1
    while not msc.is_end_motion():
        kin_sim.tick()
        clock.advance(step)
        if kin_sim.control_cycles > 1000:
            raise TimeoutError("test stuck")

    # it takes 2 * seconds to finish the counters
    # + 1 for pulse to trigger
    # + 1 for reset
    # + 1 for EndMotion to transition to RUNNING
    # + 1 for EndMotion to observe True
    assert np.allclose(seconds * 2 + 0.4, clock.time())


def test_count_ticks():
    msc = MotionStatechart()
    msc.add_node(counter := CountControlCycles(control_cycles=3))
    msc.add_node(EndMotion.when_true(counter))
    kin_sim = Executor(MotionStatechartContext(world=World()))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()
    # ending tacks 4 ticks, one to turn EndMotion to true
    assert kin_sim.control_cycles == 3 + 1


class TestEndMotion:
    def test_end_motion_when_all_done1(self, tmp_path):
        msc = MotionStatechart()
        msc.add_nodes(
            [
                ConstTrueNode(),
                ConstTrueNode(),
            ]
        )
        end = EndMotion.when_all_true(msc.nodes)
        msc.add_node(end)

        kin_sim = Executor(
            MotionStatechartContext(
                world=World(),
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert end.life_cycle_state == LifeCycleValues.RUNNING

    def test_end_motion_when_all_done2(self, tmp_path):
        msc = MotionStatechart()
        msc.add_nodes(
            [
                ConstTrueNode(),
                ConstFalseNode(),
            ]
        )
        end = EndMotion.when_all_true(msc.nodes)
        msc.add_node(end)

        kin_sim = Executor(
            MotionStatechartContext(
                world=World(),
            )
        )
        kin_sim.compile(motion_statechart=msc)
        with pytest.raises(TimeoutError):
            kin_sim.tick_until_end()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED

    def test_end_motion_when_any_done1(self, tmp_path):
        msc = MotionStatechart()
        msc.add_nodes(
            [
                ConstTrueNode(),
                ConstFalseNode(),
            ]
        )
        end = EndMotion.when_any_true(msc.nodes)
        msc.add_node(end)

        kin_sim = Executor(
            MotionStatechartContext(
                world=World(),
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert end.life_cycle_state == LifeCycleValues.RUNNING

    def test_end_motion_when_any_done2(self, tmp_path):
        msc = MotionStatechart()
        msc.add_nodes(
            [
                ConstFalseNode(),
                ConstFalseNode(),
            ]
        )
        end = EndMotion.when_any_true(msc.nodes)
        msc.add_node(end)

        kin_sim = Executor(
            MotionStatechartContext(
                world=World(),
            )
        )
        kin_sim.compile(motion_statechart=msc)
        with pytest.raises(TimeoutError):
            kin_sim.tick_until_end()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert end.life_cycle_state == LifeCycleValues.NOT_STARTED

    def test_goals_cannot_have_end_motion(self):
        msc = MotionStatechart()
        msc.add_node(Sequence([ConstTrueNode(), EndMotion()]))
        with pytest.raises(EndMotionInGoalError):
            kin_sim = Executor(
                MotionStatechartContext(
                    world=World(),
                )
            )
            kin_sim.compile(motion_statechart=msc)


class TestTemplates:

    def test_sequence_goal(self, tmp_path):
        msc = MotionStatechart()
        node = Sequence(
            nodes=[
                ConstTrueNode(),
                ConstTrueNode(),
                ConstTrueNode(),
                ConstTrueNode(),
            ]
        )
        msc.add_node(node)
        msc.add_node(EndMotion.when_true(node))

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        msc.draw(str(tmp_path / "muh.pdf"))
        assert kin_sim.control_cycles == 6
        assert msc.nodes[1].life_cycle_state == LifeCycleValues.RUNNING
        assert msc.nodes[2].life_cycle_state == LifeCycleValues.DONE
        assert msc.nodes[3].life_cycle_state == LifeCycleValues.DONE
        assert msc.nodes[4].life_cycle_state == LifeCycleValues.DONE
        assert msc.nodes[5].life_cycle_state == LifeCycleValues.DONE

    def test_parallel(self):
        msc = MotionStatechart()
        msc.add_nodes(
            [
                parallel := Parallel(
                    [
                        CountControlCycles(control_cycles=3),
                        CountControlCycles(control_cycles=5),
                    ]
                ),
            ]
        )
        msc.add_node(EndMotion.when_true(parallel))

        kin_sim = Executor(
            MotionStatechartContext(
                world=World(),
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        # 5 (longest ticker) + 1 (for parallel to turn True) + 1 (for end to trigger)
        assert kin_sim.control_cycles == 7

    def test_parallel_with_tasks(self, pr2_world_state_reset: World):
        map = pr2_world_state_reset.root
        r_tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        msc = MotionStatechart()
        msc.add_node(
            parallel := Parallel(
                [
                    AlignPlanes(
                        root_link=map,
                        tip_link=r_tip,
                        tip_normal=Vector3.X(reference_frame=r_tip),
                        goal_normal=Vector3.X(reference_frame=map),
                    ),
                    AlignPlanes(
                        root_link=map,
                        tip_link=r_tip,
                        tip_normal=Vector3.Y(reference_frame=r_tip),
                        goal_normal=Vector3.Z(reference_frame=map),
                    ),
                ]
            )
        )
        msc.add_node(EndMotion.when_true(parallel))

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_state_reset,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

    def test_parallel_minimum_success(self):
        """Test that Parallel completes when minimum_success nodes are True"""
        msc = MotionStatechart()
        msc.add_nodes(
            [
                parallel := Parallel(
                    [
                        CountControlCycles(control_cycles=2),
                        CountControlCycles(control_cycles=4),
                        CountControlCycles(control_cycles=6),
                    ],
                    minimum_success=2,
                ),
            ]
        )
        msc.add_node(EndMotion.when_true(parallel))

        kin_sim = Executor(
            MotionStatechartContext(world=World()),
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        # 4 (second ticker completes) + 1 (for parallel to turn True) + 1 (for end to trigger)
        assert kin_sim.control_cycles == 6

    def test_parallel_minimum_success_zero(self):
        """Test that Parallel completes when no node is True"""
        msc = MotionStatechart()
        msc.add_nodes(
            [
                parallel := Parallel(
                    [
                        CountControlCycles(control_cycles=3),
                        CountControlCycles(control_cycles=5),
                        CountControlCycles(control_cycles=7),
                    ],
                    minimum_success=0,
                ),
            ]
        )
        msc.add_node(EndMotion.when_true(parallel))

        kin_sim = Executor(
            MotionStatechartContext(world=World()),
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        # 0 (no ticker completes) + 1 (for parallel to turn True) + 1 (for end to trigger)
        assert kin_sim.control_cycles == 2


class TestOpenClose:
    def test_open(self, pr2_world_copy, tmp_path):

        with pr2_world_copy.modify_world():
            door = Door.create_with_new_body_in_world(
                name=PrefixedName("door"),
                world=pr2_world_copy,
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=1.5, z=1, yaw=np.pi, reference_frame=pr2_world_copy.root
                ),
            )

            handle = Handle.create_with_new_body_in_world(
                name=PrefixedName("handle"),
                world=pr2_world_copy,
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=1.5,
                    y=0.45,
                    z=1,
                    yaw=np.pi,
                    reference_frame=pr2_world_copy.root,
                ),
            )

            lower_limits = DerivativeMap()
            lower_limits.position = -np.pi / 2
            lower_limits.velocity = -1
            upper_limits = DerivativeMap()
            upper_limits.position = np.pi / 2
            upper_limits.velocity = 1

            hinge = Hinge.create_with_new_body_in_world(
                name=PrefixedName("hinge"),
                world=pr2_world_copy,
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=1.5,
                    y=-0.5,
                    z=1,
                    yaw=np.pi,
                    reference_frame=pr2_world_copy.root,
                ),
                connection_limits=DegreeOfFreedomLimits(
                    lower=lower_limits, upper=upper_limits
                ),
                active_axis=Vector3.Z(),
            )

            door.add_handle(handle)
            door.add_hinge(hinge=hinge)

        root_C_hinge = door.hinge.root.parent_connection

        r_tip = pr2_world_copy.get_body_by_name("r_gripper_tool_frame")
        handle = pr2_world_copy.get_semantic_annotations_by_type(Handle)[0].root
        open_goal = 1
        close_goal = -1

        msc = MotionStatechart()
        msc.add_nodes(
            [
                Sequence(
                    [
                        CartesianPose(
                            root_link=pr2_world_copy.root,
                            tip_link=r_tip,
                            goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                                yaw=np.pi, reference_frame=handle
                            ),
                        ),
                        Parallel(
                            [
                                Open(
                                    tip_link=r_tip,
                                    environment_link=handle,
                                    goal_joint_state=open_goal,
                                ),
                                opened := JointPositionReached(
                                    connection=root_C_hinge,
                                    position=open_goal,
                                    name="opened",
                                ),
                            ]
                        ),
                        Parallel(
                            [
                                Close(
                                    tip_link=r_tip,
                                    environment_link=handle,
                                    goal_joint_state=close_goal,
                                ),
                                closed := JointPositionReached(
                                    connection=root_C_hinge,
                                    position=close_goal,
                                    name="closed",
                                ),
                            ]
                        ),
                    ]
                ),
            ]
        )
        msc.add_node(EndMotion.when_true(msc.nodes[0]))

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_world_copy,
            )
        )
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        msc.draw(str(tmp_path / "muh.pdf"))

        assert opened.observation_state == ObservationStateValues.TRUE
        assert closed.observation_state == ObservationStateValues.TRUE


class TestCollisionAvoidance:
    def test_external_collision_avoidance(self, cylinder_bot_world: World):
        robot = cylinder_bot_world.get_semantic_annotations_by_type(AbstractRobot)[0]
        tip = cylinder_bot_world.get_kinematic_structure_entity_by_name("bot")
        env1 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment")
        env2 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment2")

        msc = MotionStatechart()
        msc.add_nodes(
            [
                UpdateTemporaryCollisionRules(
                    temporary_rules=[
                        AvoidCollisionBetweenGroups(
                            buffer_zone_distance=0.05,
                            violated_distance=0.0,
                            body_group_a=[tip],
                            body_group_b=[env1],
                        )
                    ]
                ),
                distance_violated := ExternalCollisionDistanceMonitor(
                    body=robot.root, threshold=0.049
                ),
                CartesianPose(
                    root_link=cylinder_bot_world.root,
                    tip_link=tip,
                    goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=1, reference_frame=cylinder_bot_world.root
                    ),
                ),
                ExternalCollisionAvoidance(robot=robot),
                local_min := LocalMinimumReached(),
            ]
        )
        msc.add_node(EndMotion.when_true(local_min))
        msc.add_node(CancelMotion.when_true(distance_violated))

        json_data = msc.to_json()
        json_str = json.dumps(json_data)
        new_json_data = json.loads(json_str)

        tracker = WorldEntityWithIDKwargsTracker.from_world(cylinder_bot_world)
        kwargs = tracker.create_kwargs()
        msc_copy = MotionStatechart.from_json(new_json_data, **kwargs)

        kin_sim = Executor(
            MotionStatechartContext(world=cylinder_bot_world),
            pacer=SimulationPacer(real_time_factor=2),
        )
        kin_sim.compile(motion_statechart=msc_copy)

        kin_sim.tick_until_end(500)
        collisions = kin_sim.context.world.collision_manager.compute_collisions()
        assert len(collisions.contacts) == 1
        assert collisions.contacts[0].distance > 0.049
        assert len(cylinder_bot_world.collision_manager.collision_consumers) == 0

    def test_external_collision_avoidance_battle(self):
        strong_robot_world = World()
        with strong_robot_world.modify_world():
            strong = Body(
                name=PrefixedName("strong"),
                collision=ShapeCollection(
                    [Cylinder(width=0.1, height=0.1, color=Color(R=1, G=0, B=0, A=1))]
                ),
            )
            strong_robot_world.add_body(strong)
            strong_robot_sa = MinimalRobot.from_world(strong_robot_world)

        weak_robot_world = World()
        with weak_robot_world.modify_world():
            weak = Body(
                name=PrefixedName("weak"),
                collision=ShapeCollection(
                    [Cylinder(width=0.1, height=0.1, color=Color(R=0, G=0, B=1, A=1))]
                ),
            )
            weak_robot_world.add_body(weak)
            weak_robot_sa = MinimalRobot.from_world(weak_robot_world)

        world = World()

        with world.modify_world():
            wall = Body(
                name=PrefixedName("wall"),
                collision=ShapeCollection([Box(scale=Scale(x=0.1, y=10, z=0.1))]),
            )
            map = Body(name=PrefixedName("map"))
            world.add_connection(
                FixedConnection(
                    parent=map,
                    child=wall,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=0.5
                    ),
                )
            )
            strong_odom = Body(name=PrefixedName("strong_odom"))
            world.add_connection(
                FixedConnection(
                    parent=map,
                    child=strong_odom,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        y=0.5
                    ),
                )
            )
            weak_odom = Body(name=PrefixedName("weak_odom"))
            world.add_connection(
                FixedConnection(
                    parent=map,
                    child=weak_odom,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        y=-0.5
                    ),
                )
            )

            # %% attach robots
            world.merge_world(
                strong_robot_world,
                omni1 := OmniDrive.create_with_dofs(
                    world=world,
                    parent=strong_odom,
                    child=strong_robot_world.root,
                ),
            )
            world.merge_world(
                weak_robot_world,
                omni2 := OmniDrive.create_with_dofs(
                    world=world,
                    parent=weak_odom,
                    child=weak_robot_world.root,
                ),
            )
            omni1.has_hardware_interface = True
            omni2.has_hardware_interface = True

        msc = MotionStatechart()
        msc.add_nodes(
            [
                UpdateTemporaryCollisionRules(
                    temporary_rules=[
                        AvoidCollisionBetweenGroups(
                            buffer_zone_distance=0.05,
                            violated_distance=0.0,
                            body_group_a=[weak],
                            body_group_b=[wall],
                        ),
                        AvoidCollisionBetweenGroups(
                            buffer_zone_distance=0.05,
                            violated_distance=0.0,
                            body_group_a=[strong],
                            body_group_b=[wall],
                        ),
                    ]
                ),
                CartesianPose(
                    root_link=map,
                    tip_link=strong,
                    goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=1, reference_frame=strong
                    ),
                    weight=DefaultWeights.WEIGHT_COLLISION_AVOIDANCE,
                ),
                CartesianPose(
                    root_link=map,
                    tip_link=weak,
                    goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=1, reference_frame=weak
                    ),
                    weight=DefaultWeights.WEIGHT_BELOW_CA,
                ),
                ExternalCollisionAvoidance(robot=weak_robot_sa),
                ExternalCollisionAvoidance(robot=strong_robot_sa),
                local_min := LocalMinimumReached(),
            ]
        )
        msc.add_node(EndMotion.when_true(local_min))
        # msc.add_node(CancelMotion.when_true(distance_violated))

        kin_sim = Executor(
            MotionStatechartContext(world=world),
            pacer=SimulationPacer(real_time_factor=1),
        )
        kin_sim.compile(motion_statechart=msc)

        kin_sim.tick_until_end(500)

        assert (
            0.05
            > world.collision_manager.collision_detector.check_collision_between_bodies(
                body_a=strong, body_b=wall, distance=1
            ).distance
            > -0.01
        )

        assert (
            world.collision_manager.collision_detector.check_collision_between_bodies(
                body_a=weak, body_b=wall, distance=1
            ).distance
            > 0.049
        )

    def test_external_collision_avoidance_with_weight_above_ca(
        self, cylinder_bot_world: World
    ):
        robot = cylinder_bot_world.get_semantic_annotations_by_type(AbstractRobot)[0]
        tip = cylinder_bot_world.get_kinematic_structure_entity_by_name("bot")
        env1 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment")
        env2 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment2")

        msc = MotionStatechart()
        msc.add_nodes(
            [
                UpdateTemporaryCollisionRules(
                    temporary_rules=[
                        AvoidCollisionBetweenGroups(
                            buffer_zone_distance=0.05,
                            violated_distance=0.0,
                            body_group_a=[tip],
                            body_group_b=[env1],
                        )
                    ]
                ),
                distance_violated := ExternalCollisionDistanceMonitor(
                    body=robot.root, threshold=0.0
                ),
                CartesianPose(
                    root_link=cylinder_bot_world.root,
                    tip_link=tip,
                    goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=1, reference_frame=cylinder_bot_world.root
                    ),
                    weight=DefaultWeights.WEIGHT_COLLISION_AVOIDANCE,
                ),
                ExternalCollisionAvoidance(robot=robot),
                local_min := LocalMinimumReached(),
            ]
        )
        msc.add_node(EndMotion.when_true(local_min))
        msc.add_node(CancelMotion.when_true(distance_violated))

        json_data = msc.to_json()
        json_str = json.dumps(json_data)
        new_json_data = json.loads(json_str)

        tracker = WorldEntityWithIDKwargsTracker.from_world(cylinder_bot_world)
        kwargs = tracker.create_kwargs()
        msc_copy = MotionStatechart.from_json(new_json_data, **kwargs)

        kin_sim = Executor(
            MotionStatechartContext(world=cylinder_bot_world),
            pacer=SimulationPacer(real_time_factor=1),
        )
        kin_sim.compile(motion_statechart=msc_copy)

        kin_sim.tick_until_end(500)
        collisions = kin_sim.context.world.collision_manager.compute_collisions()
        assert len(collisions.contacts) == 1
        assert collisions.contacts[0].distance < 0.049
        assert collisions.contacts[0].distance > 0.0
        assert len(cylinder_bot_world.collision_manager.collision_consumers) == 0

    def test_update_collision_matrix_later(self, cylinder_bot_world: World):
        robot = cylinder_bot_world.get_semantic_annotations_by_type(AbstractRobot)[0]
        tip = cylinder_bot_world.get_kinematic_structure_entity_by_name("bot")
        env1 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment")
        env2 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment2")

        msc = MotionStatechart()
        msc.add_nodes(
            [
                UpdateTemporaryCollisionRules(
                    temporary_rules=[
                        AvoidCollisionBetweenGroups(
                            buffer_zone_distance=0.05,
                            violated_distance=0.0,
                            body_group_a=[tip],
                            body_group_b=[env1],
                        )
                    ]
                ),
                ExternalCollisionAvoidance(robot=robot),
                cart_goal_reached := CartesianPose(
                    root_link=cylinder_bot_world.root,
                    tip_link=tip,
                    goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=1, reference_frame=cylinder_bot_world.root
                    ),
                ),
                Sequence(
                    [
                        LocalMinimumReached(),
                        UpdateTemporaryCollisionRules(
                            temporary_rules=[AllowAllCollisions()]
                        ),
                    ]
                ),
            ]
        )
        msc.add_node(EndMotion.when_true(cart_goal_reached))

        kin_sim = Executor(
            MotionStatechartContext(world=cylinder_bot_world),
        )
        kin_sim.compile(motion_statechart=msc)

        kin_sim.tick_until_end(500)
        msc.draw("muh.pdf")
        cylinder_bot_world.collision_manager.clear_temporary_rules()
        cylinder_bot_world.collision_manager.add_temporary_rule(AvoidAllCollisions())
        cylinder_bot_world.collision_manager.update_collision_matrix()
        collisions = kin_sim.context.world.collision_manager.compute_collisions()
        assert len(collisions.contacts) == 1
        assert collisions.contacts[0].distance < 0.049
        assert len(cylinder_bot_world.collision_manager.collision_consumers) == 0

    def test_consumer_cleanup_after_cancel(self, cylinder_bot_world: World):
        robot = cylinder_bot_world.get_semantic_annotations_by_type(AbstractRobot)[0]
        tip = cylinder_bot_world.get_kinematic_structure_entity_by_name("bot")
        env1 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment")
        env2 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment2")

        msc = MotionStatechart()
        msc.add_nodes(
            [
                UpdateTemporaryCollisionRules(
                    temporary_rules=[
                        AvoidCollisionBetweenGroups(
                            buffer_zone_distance=0.05,
                            violated_distance=0.0,
                            body_group_a=[tip],
                            body_group_b=[env1],
                        ),
                    ]
                ),
                CartesianPose(
                    root_link=cylinder_bot_world.root,
                    tip_link=tip,
                    goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=1, reference_frame=cylinder_bot_world.root
                    ),
                ),
                ExternalCollisionAvoidance(robot=robot),
                local_min := LocalMinimumReached(),
            ]
        )
        msc.add_node(
            Sequence(
                [
                    CountControlCycles(control_cycles=5),
                    CancelMotion(exception=Exception("muh")),
                ]
            )
        )
        msc.add_node(EndMotion.when_true(local_min))

        kin_sim = Executor(MotionStatechartContext(world=cylinder_bot_world))
        kin_sim.compile(motion_statechart=msc)

        with pytest.raises(Exception, match="muh"):
            kin_sim.tick_until_end(500)
        assert len(cylinder_bot_world.collision_manager.collision_consumers) == 0

    def test_multiple_external_collision_avoidance_motions(
        self, cylinder_bot_world: World
    ):
        robot = cylinder_bot_world.get_semantic_annotations_by_type(AbstractRobot)[0]
        tip = cylinder_bot_world.get_kinematic_structure_entity_by_name("bot")
        env1 = cylinder_bot_world.get_kinematic_structure_entity_by_name("environment")

        def run_motion(goal_x):
            msc = MotionStatechart()
            msc.add_nodes(
                [
                    UpdateTemporaryCollisionRules(
                        temporary_rules=[
                            AvoidCollisionBetweenGroups(
                                buffer_zone_distance=0.05,
                                violated_distance=0.0,
                                body_group_a=[tip],
                                body_group_b=[env1],
                            ),
                        ]
                    ),
                    CartesianPose(
                        root_link=cylinder_bot_world.root,
                        tip_link=tip,
                        goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                            x=goal_x, reference_frame=cylinder_bot_world.root
                        ),
                    ),
                    ExternalCollisionAvoidance(robot=robot),
                    local_min := LocalMinimumReached(),
                ]
            )
            msc.add_node(EndMotion.when_true(local_min))

            kin_sim = Executor(MotionStatechartContext(world=cylinder_bot_world))
            kin_sim.compile(motion_statechart=msc)
            kin_sim.tick_until_end(500)
            return kin_sim

        # First motion
        run_motion(1.0)
        assert len(cylinder_bot_world.collision_manager.collision_consumers) == 0

        # Second motion
        run_motion(0.5)
        assert len(cylinder_bot_world.collision_manager.collision_consumers) == 0

    def test_self_collision_avoidance(self, self_collision_bot_world: World):

        robot = self_collision_bot_world.get_semantic_annotations_by_type(
            AbstractRobot
        )[0]
        l_tip = self_collision_bot_world.get_kinematic_structure_entity_by_name("l_tip")
        r_tip = self_collision_bot_world.get_kinematic_structure_entity_by_name("r_tip")
        l_thumb = self_collision_bot_world.get_kinematic_structure_entity_by_name(
            "l_thumb"
        )
        r_thumb = self_collision_bot_world.get_kinematic_structure_entity_by_name(
            "r_thumb"
        )

        msc = MotionStatechart()
        msc.add_nodes(
            [
                UpdateTemporaryCollisionRules(
                    temporary_rules=[
                        AvoidCollisionBetweenGroups(
                            buffer_zone_distance=0.25,
                            violated_distance=0.0,
                            body_group_a={l_thumb},
                            body_group_b={r_thumb},
                        ),
                    ]
                ),
                SelfCollisionAvoidance(robot=robot),
                local_min := LocalMinimumReached(),
            ]
        )
        msc.add_node(EndMotion.when_true(local_min))

        json_data = msc.to_json()
        json_str = json.dumps(json_data)
        new_json_data = json.loads(json_str)

        tracker = WorldEntityWithIDKwargsTracker.from_world(self_collision_bot_world)
        kwargs = tracker.create_kwargs()
        msc_copy = MotionStatechart.from_json(new_json_data, **kwargs)

        kin_sim = Executor(MotionStatechartContext(world=self_collision_bot_world))
        kin_sim.compile(motion_statechart=msc_copy)

        # 4 because of the base nodes + 20 that are added by self collision avoidance
        assert len(msc_copy.nodes) == 4 + 20

        kin_sim.tick_until_end(500)
        collisions = kin_sim.context.world.collision_manager.compute_collisions()
        assert len(collisions.contacts) == 1
        for contact in collisions.contacts:
            assert contact.distance > 0.249
        assert len(self_collision_bot_world.collision_manager.collision_consumers) == 0

    def test_avoid_collision_go_around_corner(self, pr2_with_box):
        r_tip = pr2_with_box.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        robot = pr2_with_box.get_semantic_annotations_by_type(AbstractRobot)[0]

        msc = MotionStatechart()
        msc.add_node(
            UpdateTemporaryCollisionRules(
                temporary_rules=[
                    AvoidExternalCollisions(
                        buffer_zone_distance=0.1, violated_distance=0.0, robot=robot
                    )
                ]
            )
        )
        msc.add_node(
            Sequence(
                [
                    SetSeedConfiguration(
                        seed_configuration=JointState.from_str_dict(
                            {
                                "r_elbow_flex_joint": -1.29610152504,
                                "r_forearm_roll_joint": -0.0301682323805,
                                "r_shoulder_lift_joint": 1.20324921318,
                                "r_shoulder_pan_joint": -0.73456435706,
                                "r_upper_arm_roll_joint": -0.70790051778,
                                "r_wrist_flex_joint": -0.10001,
                                "r_wrist_roll_joint": 0.258268529825,
                                "l_elbow_flex_joint": -1.29610152504,
                                "l_forearm_roll_joint": 0.0301682323805,
                                "l_shoulder_lift_joint": 1.20324921318,
                                "l_shoulder_pan_joint": 0.73456435706,
                                "l_upper_arm_roll_joint": 0.70790051778,
                                "l_wrist_flex_joint": -0.1001,
                                "l_wrist_roll_joint": -0.258268529825,
                                "torso_lift_joint": 0.2,
                                "head_pan_joint": 0,
                                "head_tilt_joint": 0,
                                "l_gripper_l_finger_joint": 0.55,
                                "r_gripper_l_finger_joint": 0.55,
                            },
                            world=pr2_with_box,
                        )
                    ),
                    Parallel(
                        [
                            CartesianPose(
                                root_link=pr2_with_box.root,
                                tip_link=r_tip,
                                goal_pose=HomogeneousTransformationMatrix.from_xyz_axis_angle(
                                    x=0.8,
                                    y=-0.38,
                                    z=0.84,
                                    axis=Vector3.Y(),
                                    angle=np.pi / 2.0,
                                    reference_frame=pr2_with_box.root,
                                ),
                                weight=DefaultWeights.WEIGHT_BELOW_CA,
                            ),
                            ExternalCollisionAvoidance(robot=robot),
                        ]
                    ),
                ]
            )
        )
        msc.add_node(
            distance_violated := ExternalCollisionDistanceMonitor(
                body=pr2_with_box.get_kinematic_structure_entity_by_name(
                    "r_gripper_palm_link"
                ),
                threshold=0.0,
            ),
        )
        msc.add_node(local_min := LocalMinimumReached())
        msc.add_node(EndMotion.when_true(local_min))
        msc.add_node(CancelMotion.when_true(distance_violated))

        kin_sim = Executor(MotionStatechartContext(world=pr2_with_box))
        kin_sim.compile(motion_statechart=msc)

        kin_sim.tick_until_end(500)

    def test_avoid_self_collision_with_l_arm(self, pr2_with_box):
        r_tip = pr2_with_box.get_kinematic_structure_entity_by_name(
            "r_gripper_tool_frame"
        )
        l_forearm_link = pr2_with_box.get_kinematic_structure_entity_by_name(
            "l_forearm_link"
        )
        r_palm_link = pr2_with_box.get_kinematic_structure_entity_by_name(
            "r_gripper_palm_link"
        )
        base_footprint = pr2_with_box.get_kinematic_structure_entity_by_name(
            "base_footprint"
        )
        robot = pr2_with_box.get_semantic_annotations_by_type(AbstractRobot)[0]

        msc = MotionStatechart()
        msc.add_node(
            Sequence(
                [
                    SetSeedConfiguration(
                        seed_configuration=JointState.from_str_dict(
                            {
                                "r_elbow_flex_joint": -1.43286344265,
                                "r_forearm_roll_joint": -1.26465060073,
                                "r_shoulder_lift_joint": 0.47990329056,
                                "r_shoulder_pan_joint": -0.281272240139,
                                "r_upper_arm_roll_joint": -0.528415402668,
                                "r_wrist_flex_joint": -1.18811419869,
                                "r_wrist_roll_joint": 2.26884630124,
                                "l_elbow_flex_joint": 0.0,
                                "l_forearm_roll_joint": 0.0,
                                "l_shoulder_lift_joint": 0.0,
                                "l_shoulder_pan_joint": 0.0,
                                "l_upper_arm_roll_joint": 0.0,
                                "l_wrist_flex_joint": 0.0,
                                "l_wrist_roll_joint": 0.0,
                            },
                            world=pr2_with_box,
                        )
                    ),
                    Parallel(
                        [
                            CartesianPose(
                                root_link=base_footprint,
                                tip_link=r_tip,
                                goal_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                                    0.2, reference_frame=r_tip
                                ),
                                weight=DefaultWeights.WEIGHT_ABOVE_CA,
                            ),
                            SelfCollisionAvoidance(robot=robot),
                        ],
                    ),
                ],
            ),
        )
        msc.add_node(
            contact := SelfCollisionDistanceMonitor(
                body_a=r_palm_link, body_b=l_forearm_link, threshold=0.01
            )
        )
        msc.add_node(local_min := LocalMinimumReached())
        msc.add_node(EndMotion.when_true(local_min))
        msc.add_node(CancelMotion.when_true(contact))

        kin_sim = Executor(
            MotionStatechartContext(
                world=pr2_with_box,
                qp_controller_config=QPControllerConfig(
                    target_frequency=100,
                    prediction_horizon=30,
                ),
            )
        )
        kin_sim.compile(motion_statechart=msc)

        assert len(msc.nodes) == 75

        kin_sim.tick_until_end(500)


def test_constraint_collection(pr2_world_state_reset: World):
    """
    Test the constraint collection naming behavior. Expected behavior is:
    - Not naming constraints should result in automatically generated unique names
    - Manually naming constraints the same name should result in an Exception
    - Merging constraint collections should handle duplicates via prefix if they are in different collections
    - Merge raises an Exception if a collection contains duplicates in itself
    """
    col = ConstraintCollection()
    tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
        "r_gripper_tool_frame"
    )
    root = pr2_world_state_reset.get_kinematic_structure_entity_by_name("odom_combined")

    expr = Vector3.X(tip).angle_between(Vector3.Y(root))

    col.add_point_goal_constraints(
        frame_P_current=Point3(0, 0, 0, reference_frame=tip),
        frame_P_goal=Point3(0, 0, 0, reference_frame=tip),
        reference_velocity=0.1,
        quadratic_weight=DefaultWeights.WEIGHT_BELOW_CA,
    )
    assert len(col.equality_constraints) >= 3

    for i in range(3):
        col.add_equality_constraint(
            reference_velocity=0.1 * i,
            equality_bound=0.0,
            quadratic_weight=DefaultWeights.WEIGHT_BELOW_CA,
            task_expression=expr,
        )

    col.add_inequality_constraint(
        name="same_name",
        reference_velocity=0.2,
        quadratic_weight=DefaultWeights.WEIGHT_BELOW_CA,
        task_expression=expr,
        lower_error=0.1,
        upper_error=0.2,
    )

    with pytest.raises(DuplicateNameException):
        col.add_equality_constraint(
            name="same_name",
            reference_velocity=0.2,
            equality_bound=0.0,
            quadratic_weight=DefaultWeights.WEIGHT_BELOW_CA,
            task_expression=expr,
        )

    col2 = ConstraintCollection()
    col2.add_equality_constraint(
        name="same_name",
        reference_velocity=0.2,
        equality_bound=0.0,
        quadratic_weight=DefaultWeights.WEIGHT_BELOW_CA,
        task_expression=expr,
    )

    col.merge("prefix", col2)
    assert any(c.name.startswith("prefix/") for c in col._constraints)

    with pytest.raises(DuplicateNameException):
        col.merge("", col2)

    col3 = ConstraintCollection()
    col3.add_equality_constraint(
        name="same_name",
        reference_velocity=0.2,
        equality_bound=0.0,
        quadratic_weight=DefaultWeights.WEIGHT_BELOW_CA,
        task_expression=expr,
    )
    constraint = EqualityConstraint(
        name="same_name",
        expression=expr,
        bound=0.0,
        normalization_factor=0.1,
        quadratic_weight=DefaultWeights.WEIGHT_BELOW_CA,
        lower_slack_limit=-float("inf"),
        upper_slack_limit=float("inf"),
        linear_weight=0,
    )
    col3._constraints.append(constraint)

    with pytest.raises(DuplicateNameException):
        col3._are_names_unique()

    with pytest.raises(DuplicateNameException):
        col2.merge("", col3)


class TestLifeCycleTransitions:
    """
    Tests the LifeCycle transitions of nodes in various edge cases and intended behavior.
    """

    def test_run_after_stop(self):
        """
        Test for node to run after the parent node already stopped.
        """
        msc = MotionStatechart()

        msc.add_node(
            sequence := Sequence(
                [
                    ConstTrueNode(),
                    TestRunAfterStop(),
                    CountControlCycles(name="delay endmotion", control_cycles=5),
                ]
            )
        )
        msc.add_node(EndMotion.when_true(sequence))

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert sequence.nodes[1].cancel.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert sequence.nodes[1].ticking1.life_cycle_state == LifeCycleValues.DONE
        assert sequence.nodes[1].ticking2.life_cycle_state == LifeCycleValues.DONE
        assert sequence.nodes[1].life_cycle_state == LifeCycleValues.DONE

    def test_run_after_stop_from_pause(self):
        """
        Test for node to run from paused while the parent node already stopped.
        """
        msc = MotionStatechart()

        msc.add_node(
            sequence := Sequence(
                [
                    ConstTrueNode(),
                    TestRunAfterStopFromPause(),
                    CountControlCycles(name="delay endmotion", control_cycles=5),
                ]
            )
        )
        msc.add_node(EndMotion.when_true(sequence))

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert sequence.nodes[1].cancel.life_cycle_state == LifeCycleValues.NOT_STARTED
        assert sequence.nodes[1].ticking1.life_cycle_state == LifeCycleValues.DONE
        assert sequence.nodes[1].ticking2.life_cycle_state == LifeCycleValues.DONE
        assert sequence.nodes[1].ticking3.life_cycle_state == LifeCycleValues.DONE
        assert sequence.nodes[1].pulse.life_cycle_state == LifeCycleValues.DONE
        assert sequence.nodes[1].life_cycle_state == LifeCycleValues.DONE

    def test_end_before_start(self):
        """
        Test for node to start even if it's end condition is met before start condition.
        Node3 should start and run for 1 tick before ending, instead of never starting.
        """
        msc = MotionStatechart()

        node1 = CountControlCycles(control_cycles=1)
        node2 = ConstTrueNode()
        node3 = ConstTrueNode()

        msc.add_nodes(nodes=[node1, node2, node3])
        msc.add_node(EndMotion.when_true(node3))

        node3.start_condition = node1.observation_variable
        node3.end_condition = node2.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick()

        assert node3.life_cycle_state == LifeCycleValues.RUNNING
        assert node3.observation_state == ObservationStateValues.UNKNOWN

        kin_sim.tick()

        assert node3.life_cycle_state == LifeCycleValues.DONE
        assert node3.observation_state == ObservationStateValues.TRUE

    def test_end_before_start_in_template(self):
        """
        Test for node to start even if it's end condition is met before start condition,
        when the nodes are inside a template.
        """
        msc = MotionStatechart()

        node = TestEndBeforeStart()
        msc.add_node(node)

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick()

        assert node.node3.life_cycle_state == LifeCycleValues.RUNNING
        assert node.node3.observation_state == ObservationStateValues.UNKNOWN

        kin_sim.tick()

        assert node.node3.life_cycle_state == LifeCycleValues.DONE
        assert node.node3.observation_state == ObservationStateValues.TRUE

    def test_intended_transitions(self):
        """
        Test for intended LifeCycle transitions of nodes.
        """
        msc = MotionStatechart()

        count_node1 = CountControlCycles(control_cycles=1, name="node1")
        count_node2 = CountControlCycles(control_cycles=2, name="node2")
        end_count_node1 = CountControlCycles(control_cycles=11, name="end_node1")
        pulse_node1 = Pulse(name="pulse1")
        pulse_node2 = Pulse(name="pulse2")

        msc.add_nodes(
            nodes=[
                count_node1,
                count_node2,
                end_count_node1,
                pulse_node1,
                pulse_node2,
            ]
        )
        msc.add_node(end_node := EndMotion.when_true(end_count_node1))

        pulse_node1.start_condition = count_node1.observation_variable
        pulse_node2.start_condition = count_node2.observation_variable
        count_node2.start_condition = pulse_node1.observation_variable

        count_node1.pause_condition = pulse_node1.observation_variable

        count_node1.end_condition = count_node2.observation_variable
        pulse_node1.end_condition = count_node2.observation_variable

        count_node1.reset_condition = pulse_node2.observation_variable
        count_node2.reset_condition = pulse_node2.observation_variable
        pulse_node1.reset_condition = pulse_node2.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert len(msc.history) == 14
        # %% count_node1 history
        assert msc.history.get_life_cycle_history_of_node(count_node1) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.PAUSED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.DONE,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.PAUSED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.DONE,
            LifeCycleValues.DONE,
            LifeCycleValues.DONE,
        ]
        assert msc.history.get_observation_history_of_node(count_node1) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]

        # %% count_node2 history
        assert msc.history.get_life_cycle_history_of_node(count_node2) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(count_node2) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.FALSE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.FALSE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]

        # %% end_count_node1 history
        assert msc.history.get_life_cycle_history_of_node(end_count_node1) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(end_count_node1) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
        ]

        # %% end_node history
        assert msc.history.get_life_cycle_history_of_node(end_node) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(end_node) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
        ]

        # %% pulse_node1 history
        assert msc.history.get_life_cycle_history_of_node(pulse_node1) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.DONE,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.DONE,
            LifeCycleValues.DONE,
            LifeCycleValues.DONE,
        ]
        assert msc.history.get_observation_history_of_node(pulse_node1) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
        ]

        # %% pulse_node2 history
        assert msc.history.get_life_cycle_history_of_node(pulse_node2) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
            LifeCycleValues.RUNNING,
        ]
        assert msc.history.get_observation_history_of_node(pulse_node2) == [
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.UNKNOWN,
            ObservationStateValues.TRUE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
            ObservationStateValues.FALSE,
        ]

    def test_unpause_unknown_from_parent_pause(self):
        """
        Test for child node to unpause when parent node unpauses.
        Child node pause condition is unknown.
        """

        msc = MotionStatechart()

        pulse = Pulse()
        unpause = TestUnpauseUnknownFromParentPause()

        msc.add_nodes(nodes=[pulse, unpause])
        msc.add_node(EndMotion.when_true(unpause))

        unpause.pause_condition = pulse.observation_variable

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()

        assert unpause.count_ticks1.life_cycle_state == LifeCycleValues.DONE
        assert unpause.cancel.life_cycle_state == LifeCycleValues.NOT_STARTED

        assert unpause.count_ticks1.observation_state == ObservationStateValues.TRUE
        assert unpause.observation_state == ObservationStateValues.TRUE

    def test_long_pause(self):
        msc = MotionStatechart()
        msc.add_nodes(
            [
                node1 := Parallel([ConstTrueNode(), ConstFalseNode()]),
                pulse := Pulse(length=5),
            ]
        )
        node1.pause_condition = pulse.observation_variable
        msc.add_node(EndMotion.when_false(pulse))

        kin_sim = Executor(MotionStatechartContext(world=World()))
        kin_sim.compile(motion_statechart=msc)
        kin_sim.tick_until_end()
        msc.plot_gantt_chart()

        assert len(msc.history) == 5


# ---------------------------------------------------------------------------
# Force/torque symbol node + admittance control
# ---------------------------------------------------------------------------


from geometry_msgs.msg import WrenchStamped

# Hunt-Crossley contact: stiffness ramps with tip speed, so a fast impact
# produces more force than a slow press at the same depth.
_CONTACT_STIFFNESS_BASE = 2000.0  # N / m at zero tip speed
_CONTACT_VELOCITY_GAIN = 4000.0  # extra N/m per m/s of tip speed


def _make_wrench(force_xyz=(0.0, 0.0, 0.0), torque_xyz=(0.0, 0.0, 0.0)) -> WrenchStamped:
    msg = WrenchStamped()
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force_xyz
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque_xyz
    return msg


def _add_wipe_table(world: World, name: str, top_z: float, thickness: float = 0.04) -> Body:
    """Add a 0.6 x 0.6 table whose top sits at ``top_z`` and return its body."""
    table_name = PrefixedName(name)
    if not any(b.name == table_name for b in world.bodies):
        with world.modify_world():
            table = Body(
                name=table_name,
                collision=ShapeCollection([Box(scale=Scale(0.6, 0.6, thickness))]),
                visual=ShapeCollection([Box(scale=Scale(0.6, 0.6, thickness))]),
            )
            world.add_connection(
                FixedConnection(
                    parent=world.root,
                    child=table,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=0.55, z=top_z - thickness / 2
                    ),
                )
            )
    return world.get_body_by_name(table_name)


def _hsr_wipe_collision_nodes(robot, gripper_bodies, table_body):
    """Avoid external collisions but allow the gripper to touch the table."""
    return [
        UpdateTemporaryCollisionRules(
            temporary_rules=[
                AvoidExternalCollisions(robot=robot),
                AllowCollisionBetweenGroups(
                    body_group_a=gripper_bodies, body_group_b=[table_body]
                ),
            ]
        ),
        ExternalCollisionAvoidance(robot=robot),
    ]


def _run_contact_wipe(
    kin_sim, world, msc, sensor, root, tip, table_top_z,
    fail_penetration, on_tick=None, max_ticks=100000,
):
    """Tick the executor while feeding back a Hunt-Crossley contact wrench.

    Penetration into the table is converted into an upward wrench each tick,
    rotated into the sensor frame and written straight into the sensor's live
    wrench (the simulation stand-in for the real-robot subscriber); a penetration
    past ``fail_penetration`` fails the test. ``on_tick`` (if given) receives the
    current tip z after the wrench is written. Returns ``(tip_z_history,
    peak_penetration)`` once the motion ends.
    """
    control_dt = kin_sim.context.qp_controller_config.control_dt
    prev_tip_z = None
    tip_z_history = []
    peak_penetration = 0.0
    end_reached = False
    for _ in range(max_ticks):
        sleep(0.05)
        tip_z = float(world.compute_forward_kinematics_np(root, tip)[2, 3])
        tip_z_history.append(tip_z)
        tip_z_speed = 0.0 if prev_tip_z is None else abs(tip_z - prev_tip_z) / control_dt
        prev_tip_z = tip_z

        penetration = max(0.0, table_top_z - tip_z)
        peak_penetration = max(peak_penetration, penetration)
        assert penetration < fail_penetration, (
            f"Penetration grew to {penetration * 100:.2f} cm; admittance failed to yield."
        )
        effective_stiffness = _CONTACT_STIFFNESS_BASE + _CONTACT_VELOCITY_GAIN * tip_z_speed
        root_R_sensor = world.compute_forward_kinematics_np(root, sensor.root)[:3, :3]
        force_in_sensor = root_R_sensor.T @ np.array(
            [0.0, 0.0, effective_stiffness * penetration]
        )
        sensor.write_wrench(force_in_sensor, np.zeros(3))

        if on_tick is not None:
            on_tick(tip_z)
        kin_sim.tick()
        if msc.is_end_motion():
            end_reached = True
            break

    assert end_reached, "Wipe did not finish within the tick budget."
    return tip_z_history, peak_penetration


@pytest.fixture
def prismatic_z_world() -> World:
    """A two-body world with a prismatic joint along Z, velocity-limited to ±1 m/s."""
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        tip = Body(name=PrefixedName("tip"))
        upper = DerivativeMap()
        upper.velocity = 1.0
        lower = DerivativeMap()
        lower.velocity = -1.0
        dof = DegreeOfFreedom(
            name=PrefixedName("dof", "z_prismatic"),
            limits=DegreeOfFreedomLimits(lower=lower, upper=upper),
        )
        world.add_degree_of_freedom(dof)
        connection = PrismaticConnection(
            parent=root, child=tip, axis=Vector3.Z(), dof_id=dof.id
        )
        world.add_connection(connection)
    return world


def _add_force_torque_sensor(world: World, frame) -> ForceTorqueSensor:
    """Attach a force/torque sensor annotation rooted at ``frame`` to ``world``."""
    with world.modify_world():
        return ForceTorqueSensor(
            name=PrefixedName("ft_sensor", "test"), root=frame, _world=world
        )


class TestAdmittanceCartesianPosition:

    def _build_task(self, world: World, **task_kwargs):
        """Construct context, FT sensor and admittance task; run build()."""
        context = MotionStatechartContext(world=world)
        sensor = _add_force_torque_sensor(world, world.root)
        defaults = dict(
            root_link=world.root,
            tip_link=world.get_kinematic_structure_entity_by_name("tip"),
            goal_point=Point3(0, 0, 0, reference_frame=world.root),
            mass=Vector3(x=1.0, y=1.0, z=1.0),
            damping=Vector3(x=20.0, y=20.0, z=20.0),
            stiffness=Vector3(),
        )
        defaults.update(task_kwargs)
        task = AdmittanceCartesianPosition(**defaults)
        task.build(context)
        return context, sensor, task

    def _apply_force(self, context, sensor, force_xyz):
        sensor.write_wrench(np.array(force_xyz), np.zeros(3))

    def _offset_seen_by_the_qp(self, task) -> np.ndarray:
        """The admittance offset as the QP reads it: out of the float-variable data,
        through the registered symbol."""
        return task._admittance_position.evaluate().flatten()[:3]

    def test_build_registers_admittance_state(self, prismatic_z_world):
        context, _, task = self._build_task(prismatic_z_world)
        assert isinstance(task._admittance_position, Vector3)
        start = context.float_variable_data.index_of(task._admittance_position)
        assert np.allclose(context.float_variable_data.data[start : start + 3], 0)
        assert np.allclose(task._state, 0), "offset and rate start at rest"

    def test_zero_force_keeps_state_at_zero(self, prismatic_z_world):
        context, sensor, task = self._build_task(prismatic_z_world)
        self._apply_force(context, sensor, (0.0, 0.0, 0.0))
        for _ in range(20):
            task.on_tick(context)
        assert np.allclose(task._state, 0)

    def test_z_force_drives_z_correction(self, prismatic_z_world):
        context, sensor, task = self._build_task(prismatic_z_world)
        self._apply_force(context, sensor, (0.0, 0.0, 10.0))
        for _ in range(10):
            task.on_tick(context)
        x_a = task._state[:3]
        assert x_a[0] == pytest.approx(0.0, abs=1e-9)
        assert x_a[1] == pytest.approx(0.0, abs=1e-9)
        assert x_a[2] > 0.01
        assert np.allclose(self._offset_seen_by_the_qp(task), x_a), (
            "the QP must see the offset the task integrated"
        )

    def test_stiff_damping_converges_to_terminal_velocity(self, prismatic_z_world):
        """With damping far beyond the bound an explicit damping term would survive
        (``control_dt * damping / mass`` > 2), a constant force error must settle at
        the terminal velocity ``force_error / damping`` instead of oscillating with
        growing amplitude. This is the "sponge bounces off the table" regression."""
        damping, desired_force = 100.0, 8.0
        context, sensor, task = self._build_task(
            prismatic_z_world,
            damping=Vector3(x=damping, y=damping, z=damping),
            desired_force=Vector3(z=desired_force),
        )
        self._apply_force(context, sensor, (0.0, 0.0, 0.0))
        velocities = []
        for _ in range(200):
            task.on_tick(context)
            velocities.append(task._state[5])

        assert velocities[-1] == pytest.approx(-desired_force / damping, abs=1e-6)
        assert all(
            later <= earlier + 1e-12
            for earlier, later in zip(velocities, velocities[1:])
        ), "Velocity must approach the terminal value monotonically, not oscillate."

    def test_press_against_contact_spring_settles_without_bouncing(
        self, prismatic_z_world
    ):
        """Pressing with a desired force against a stiff unilateral contact (the wipe
        scenario) must settle at the equilibrium penetration ``desired_force /
        contact_stiffness`` without diverging."""
        desired_force, contact_stiffness = 8.0, 2000.0
        context, sensor, task = self._build_task(
            prismatic_z_world,
            damping=Vector3(x=100.0, y=100.0, z=100.0),
            desired_force=Vector3(z=desired_force),
        )
        offsets = []
        for _ in range(2000):
            contact_force = contact_stiffness * max(0.0, -task._state[2])
            self._apply_force(context, sensor, (0.0, 0.0, contact_force))
            task.on_tick(context)
            offsets.append(task._state[2])

        equilibrium = -desired_force / contact_stiffness
        assert offsets[-1] == pytest.approx(equilibrium, abs=1e-4)
        assert min(offsets) > 10 * equilibrium, (
            "The press dove far past the equilibrium penetration; the coupled "
            "contact loop is unstable."
        )

    def test_steady_state_velocity_under_damping(self, prismatic_z_world):
        # stiffness=0, mass=1, damping=20, force=10
        # → steady-state velocity_z = force / damping = 0.5 m/s.
        context, sensor, task = self._build_task(prismatic_z_world)
        self._apply_force(context, sensor, (0.0, 0.0, 10.0))
        for _ in range(50):
            task.on_tick(context)
        xd_a = task._state[3:]
        assert xd_a[2] == pytest.approx(0.5, abs=0.05)

    def test_spring_pulls_correction_back_to_zero(self, prismatic_z_world):
        # stiffness>0 with no force pulls a non-zero offset back to zero
        # (damped spring).
        context, sensor, task = self._build_task(
            prismatic_z_world,
            mass=Vector3(x=1.0, y=1.0, z=1.0),
            damping=Vector3(x=5.0, y=5.0, z=5.0),
            stiffness=Vector3(x=50.0, y=50.0, z=50.0),
        )
        # Inject an offset and let the spring relax.
        task._state[:3] = [0.0, 0.0, 0.1]
        self._apply_force(context, sensor, (0.0, 0.0, 0.0))
        for _ in range(200):
            task.on_tick(context)
        assert abs(task._state[2]) < 0.005

    def test_reset_zeros_state(self, prismatic_z_world):
        context, sensor, task = self._build_task(prismatic_z_world)
        self._apply_force(context, sensor, (0.0, 0.0, 10.0))
        for _ in range(5):
            task.on_tick(context)
        assert task._state[2] != 0
        task.on_reset(context)
        assert np.allclose(task._state, 0)
        assert np.allclose(self._offset_seen_by_the_qp(task), 0)

    def test_invalid_mass_raises(self, prismatic_z_world):
        context = MotionStatechartContext(world=prismatic_z_world)
        _add_force_torque_sensor(prismatic_z_world, prismatic_z_world.root)
        task = AdmittanceCartesianPosition(
            root_link=prismatic_z_world.root,
            tip_link=prismatic_z_world.get_kinematic_structure_entity_by_name("tip"),
            goal_point=Point3(0, 0, 0, reference_frame=prismatic_z_world.root),
            mass=Vector3(x=1.0, y=1.0, z=0.0),
        )
        with pytest.raises(NonPositiveVirtualMassError):
            task.build(context)

    def test_full_executor_drift_in_force_direction(
        self, prismatic_z_world, rclpy_node
    ):
        """End-to-end: Ros2Executor + published wrench, fed into the sensor
        annotation by the updater node, drives tip motion."""
        VizMarkerPublisher(_world=prismatic_z_world, node=rclpy_node).with_tf_publisher()
        tip = prismatic_z_world.get_kinematic_structure_entity_by_name("tip")
        topic = "/test_admittance/wrench"
        _add_force_torque_sensor(prismatic_z_world, tip)
        wrench_source = ForceTorqueSensorUpdater(
            topic_name=topic,
            reference_frame=tip,
            name="ft",
        )
        admittance = AdmittanceCartesianPosition(
            root_link=prismatic_z_world.root,
            tip_link=tip,
            goal_point=Point3(0, 0, 0, reference_frame=prismatic_z_world.root),
            mass=Vector3(x=1.0, y=1.0, z=1.0),
            damping=Vector3(x=20.0, y=20.0, z=20.0),
            stiffness=Vector3(),
            threshold=100.0,
        )
        msc = MotionStatechart()
        msc.add_node(wrench_source)
        msc.add_node(admittance)

        kin_sim = Ros2Executor(
            context=MotionStatechartContext(world=prismatic_z_world),
            ros_node=rclpy_node,
        )
        kin_sim.compile(motion_statechart=msc)

        pub = rclpy_node.create_publisher(WrenchStamped, topic, 10)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and pub.get_subscription_count() == 0:
            time.sleep(0.02)
        assert pub.get_subscription_count() >= 1

        msg = _make_wrench(force_xyz=(0.0, 0.0, 10.0))
        for _ in range(60):
            pub.publish(msg)
            kin_sim.tick()
            time.sleep(0.01)

        tip_pose = prismatic_z_world.compute_forward_kinematics_np(
            prismatic_z_world.root, tip
        )
        assert tip_pose[2, 3] > 0.01


def test_hsr_admittance_wipe_waypoints(hsr_world_setup, rclpy_node):
    """HSR wipes waypoints set into a virtual table; admittance must yield to
    the contact wrench so the tip hovers at the surface instead of pressing
    through it."""
    VizMarkerPublisher(_world=hsr_world_setup, node=rclpy_node).with_tf_publisher()
    world = hsr_world_setup

    table_top_z = 0.6
    table_body = _add_wipe_table(world, "wipe_test_table", table_top_z)
    tip = world.get_kinematic_structure_entity_by_name("hand_gripper_tool_frame")
    root = world.root
    robot = world.get_semantic_annotations_by_type(HSRB)[0]
    gripper_bodies = [b for b in world.bodies_with_collision if "hand_" in str(b.name)]
    assert gripper_bodies, "No hand_* bodies with collision found on HSRB."

    # Waypoints sit 2-3 cm below the surface so admittance has to steer away.
    waypoint_coords = [
        (0.45, -0.25, table_top_z - 0.025),
        (0.7, 0.0, table_top_z - 0.03),
        (0.45, 0.25, table_top_z - 0.02),
    ]
    waypoints = [Point3(x=x, y=y, z=z, reference_frame=root) for x, y, z in waypoint_coords]

    force_torque_sensor = next(
        s for s in robot.sensors if isinstance(s, ForceTorqueSensor)
    )
    sequence = Sequence([
        AdmittanceCartesianPosition(
            name=f"wipe_step_{i}",
            root_link=root,
            tip_link=tip,
            goal_point=wp,
            mass=Vector3(x=1.0, y=1.0, z=1.0),
            damping=Vector3(x=20.0, y=20.0, z=20.0),
            stiffness=Vector3(x=80.0, y=80.0, z=80.0),
            threshold=0.08,
        )
        for i, wp in enumerate(waypoints)
    ])

    msc = MotionStatechart()
    msc.add_nodes(_hsr_wipe_collision_nodes(robot, gripper_bodies, table_body))
    msc.add_node(sequence)
    msc.add_node(EndMotion.when_true(sequence))

    kin_sim = Ros2Executor(
        context=MotionStatechartContext(world=world),
        ros_node=rclpy_node,
        pacer=SimulationPacer(0.01),
    )
    kin_sim.compile(motion_statechart=msc)

    tip_z_history, peak_penetration = _run_contact_wipe(
        kin_sim, world, msc, force_torque_sensor, root, tip, table_top_z,
        fail_penetration=0.01,
    )

    settled_min_z = min(tip_z_history[20:])
    max_penetration = max(0.0, table_top_z - settled_min_z)
    assert max_penetration < 0.05, (
        f"Tip dipped {max_penetration * 100:.1f} cm below table after settling."
    )

    final_xy = world.compute_forward_kinematics_np(root, tip)[:2, 3]
    target_xy = np.array(waypoint_coords[-1][:2])
    assert np.linalg.norm(final_xy - target_xy) < 0.1, (
        f"Final tip xy {final_xy} not within 10 cm of last waypoint {target_xy}."
    )
    assert peak_penetration < 0.01


def test_hsr_wipe_goal_full_pipeline(hsr_world_setup, rclpy_node):
    """End-to-end ``WipeGoal`` on the HSR: expands into
    approach -> lower-until-contact -> admittance wipes -> retract, declares
    contact only after descending, keeps the tip near the surface during the
    wipes, and lifts back above the table on retract."""
    VizMarkerPublisher(_world=hsr_world_setup, node=rclpy_node).with_tf_publisher()
    world = hsr_world_setup

    table_top_z = 0.6
    table_body = _add_wipe_table(world, "wipe_goal_test_table", table_top_z)
    tip = world.get_kinematic_structure_entity_by_name("hand_gripper_tool_frame")
    root = world.root
    robot = world.get_semantic_annotations_by_type(HSRB)[0]
    gripper_bodies = [b for b in world.bodies_with_collision if "hand_" in str(b.name)]
    assert gripper_bodies, "No hand_* bodies with collision found on HSRB."

    # Waypoints sit 2 cm below the surface so admittance has to yield upward.
    waypoint_coords = [
        (0.45, -0.20, table_top_z - 0.02),
        (0.55, 0.00, table_top_z - 0.02),
        (0.45, 0.20, table_top_z - 0.02),
    ]
    waypoints = [
        Point3(x=x, y=y, z=z, reference_frame=root) for x, y, z in waypoint_coords
    ]

    force_torque_sensor = next(
        s for s in robot.sensors if isinstance(s, ForceTorqueSensor)
    )
    wipe_goal = WipeGoal(
        name="wipe",
        segments=[waypoints],
        tip_link=tip,
        root_link=root,
    )

    msc = MotionStatechart()
    msc.add_nodes(_hsr_wipe_collision_nodes(robot, gripper_bodies, table_body))
    msc.add_node(wipe_goal)
    msc.add_node(EndMotion.when_true(wipe_goal))

    kin_sim = Ros2Executor(
        context=MotionStatechartContext(world=world),
        ros_node=rclpy_node,
        pacer=SimulationPacer(0.01),
    )
    kin_sim.compile(motion_statechart=msc)

    # No wrench source is added in simulation (the wrench is written directly),
    # so the goal's first child is the strokes sequence:
    # approach -> lower -> wipe-sequence -> retract, alongside the tip-down alignment.
    strokes_node, *_ = wipe_goal.nodes
    assert isinstance(strokes_node, Sequence)
    approach_node, lower_node, wipe_sequence_node, retract_node = strokes_node.nodes
    assert isinstance(approach_node, CartesianPosition)
    assert not isinstance(approach_node, LowerUntilContact)
    assert isinstance(lower_node, LowerUntilContact)
    assert isinstance(wipe_sequence_node, Sequence)
    assert len(wipe_sequence_node.nodes) == len(waypoints)
    assert all(
        isinstance(n, AdmittanceCartesianPosition) for n in wipe_sequence_node.nodes
    )
    assert isinstance(retract_node, CartesianPosition)
    assert not isinstance(retract_node, LowerUntilContact)

    lower_contact_z = []

    def capture_contact(tip_z):
        if not lower_contact_z and (
            lower_node.observation_state == ObservationStateValues.TRUE
        ):
            lower_contact_z.append(tip_z)

    tip_z_history, peak_penetration = _run_contact_wipe(
        kin_sim, world, msc, force_torque_sensor, root, tip, table_top_z,
        fail_penetration=0.015, on_tick=capture_contact,
    )

    assert lower_contact_z, "LowerUntilContact never reported contact during the run."
    assert lower_contact_z[0] < table_top_z + 0.05, (
        f"LowerUntilContact declared contact at z={lower_contact_z[0]:.3f}, "
        f"more than 5 cm above the table top ({table_top_z})."
    )

    settled_min_z = min(tip_z_history[20:])
    max_penetration = max(0.0, table_top_z - settled_min_z)
    assert max_penetration < 0.05, (
        f"Tip dipped {max_penetration * 100:.1f} cm below table after settling."
    )

    final_pose = world.compute_forward_kinematics_np(root, tip)
    final_z = float(final_pose[2, 3])
    final_xy = final_pose[:2, 3]
    assert final_z > table_top_z + 0.05, (
        f"After retract, tip z={final_z:.3f} is not at least 5 cm above the table top."
    )
    last_xy = np.array(waypoint_coords[-1][:2])
    assert np.linalg.norm(final_xy - last_xy) < 0.15, (
        f"Final tip xy {final_xy} not within 15 cm of last waypoint {last_xy}."
    )
    assert peak_penetration < 0.015


def test_wipe_goal_empty_segments_expands_to_nothing(hsr_world_setup):
    """Passing an empty segment list, or segments with no waypoints, must
    produce no child nodes. Guards against silently scheduling stray
    approach/lower/retract steps."""
    world = hsr_world_setup
    tip = world.get_kinematic_structure_entity_by_name("hand_gripper_tool_frame")
    root = world.root

    empty_goal = WipeGoal(
        name="wipe_empty",
        segments=[],
        tip_link=tip,
        root_link=root,
    )
    empty_goal.expand(MotionStatechartContext(world=world))
    assert empty_goal.nodes == []

    only_empty_waypoints = WipeGoal(
        name="wipe_only_empty_waypoints",
        segments=[[], []],
        tip_link=tip,
        root_link=root,
    )
    only_empty_waypoints.expand(MotionStatechartContext(world=world))
    assert only_empty_waypoints.nodes == []


def test_grasp_bar(pr2_world_state_reset: World, rclpy_node):
    tip = pr2_world_state_reset.get_kinematic_structure_entity_by_name(
        "r_gripper_tool_frame"
    )
    root = pr2_world_state_reset.get_kinematic_structure_entity_by_name("odom_combined")
    VizMarkerPublisher(node=rclpy_node, _world=pr2_world_state_reset).with_tf_publisher()

    bar_length = 0.4
    bar_center = Point3(0.6, -0.2, 1.0, reference_frame=root)
    bar_axis = Vector3.Z(reference_frame=root)
    tip_grasp_axis = Vector3.Z(reference_frame=tip)

    with pr2_world_state_reset.modify_world():
        bar_body = Body(
            name=PrefixedName("bar"),
            visual=ShapeCollection(shapes=[Cylinder(width=0.02, height=bar_length, color=Color(R=0.6, G=0.3, B=0.1, A=1))]),
        )
        pr2_world_state_reset.add_connection(
            FixedConnection(
                parent=root,
                child=bar_body,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=0.6, y=-0.2, z=1.0, reference_frame=root
                ),
            )
        )

    msc = MotionStatechart()
    grasp_bar = GraspBar(
        root_link=root,
        tip_link=tip,
        tip_grasp_axis=tip_grasp_axis,
        bar_center=bar_center,
        bar_axis=bar_axis,
        bar_length=bar_length,
    )
    msc.add_node(grasp_bar)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = grasp_bar.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    # Check axis alignment
    root_V_tip_grasp_axis = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=tip_grasp_axis
    )
    root_V_tip_grasp_axis.scale(1)
    root_V_bar_axis = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=bar_axis
    )
    root_V_bar_axis.scale(1)
    v_tip = root_V_tip_grasp_axis.to_np()[:3]
    v_bar = root_V_bar_axis.to_np()[:3]
    angle = angle_between_vector(v_tip, v_bar)
    assert angle <= grasp_bar.threshold, (
        f"GraspBar axis alignment failed: angle {angle:.6f} rad > threshold {grasp_bar.threshold:.6f} rad"
    )

    # Check tip position is within bar_length/2 of bar_center along bar_axis
    root_P_tip = (
        pr2_world_state_reset.compute_forward_kinematics(root, tip).to_position().to_np()[:3]
    )
    root_P_center = pr2_world_state_reset.transform(
        target_frame=root, spatial_object=bar_center
    ).to_np()[:3]
    projection = np.dot(root_P_tip - root_P_center, v_bar)
    projection = np.clip(projection, -grasp_bar.bar_length / 2, grasp_bar.bar_length / 2)
    nearest = root_P_center + projection * v_bar
    dist = np.linalg.norm(root_P_tip - nearest)
    assert dist <= grasp_bar.threshold, (
        f"GraspBar position failed: distance {dist:.6f} m > threshold {grasp_bar.threshold:.6f} m"
    )
