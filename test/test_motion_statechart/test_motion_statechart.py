import time
from dataclasses import dataclass

import numpy as np
import pytest

import semantic_digital_twin.spatial_types.spatial_types as cas
from giskardpy.motion_statechart.binding_policy import GoalBindingPolicy
from giskardpy.motion_statechart.data_types import (
    LifeCycleValues,
    ObservationStateValues,
)
from giskardpy.motion_statechart.graph_node import (
    EndMotion,
    CancelMotion,
    NodeArtifacts,
)
from giskardpy.motion_statechart.graph_node import Goal
from giskardpy.motion_statechart.graph_node import ThreadPayloadMonitor
from giskardpy.motion_statechart.monitors.monitors import TrueMonitor
from giskardpy.motion_statechart.monitors.overwrite_state_monitors import (
    SetSeedConfiguration,
    SetOdometry,
)
from giskardpy.motion_statechart.monitors.payload_monitors import Print, Pulse
from giskardpy.motion_statechart.motion_statechart import (
    MotionStatechart,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianOrientation,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.tasks.pointing import Pointing
from giskardpy.motion_statechart.test_nodes.test_nodes import ChangeStateOnEvents
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import TransformationMatrix
from semantic_digital_twin.spatial_types.derivatives import DerivativeMap
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    RevoluteConnection,
    ActiveConnection1DOF,
)
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom
from semantic_digital_twin.world_description.world_entity import Body


def test_condition_to_str():
    msc = MotionStatechart(World())
    node1 = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(node1)
    node2 = TrueMonitor(name=PrefixedName("muh2"))
    msc.add_node(node2)
    node3 = TrueMonitor(name=PrefixedName("muh3"))
    msc.add_node(node3)
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(end)

    end.start_condition = cas.trinary_logic_and(
        node1.observation_variable,
        cas.trinary_logic_or(
            node2.observation_variable,
            cas.trinary_logic_not(node3.observation_variable),
        ),
    )
    a = str(end._start_condition)
    assert a == '("muh" and ("muh2" or not "muh3"))'


def test_motion_statechart_to_dot():
    msc = MotionStatechart(World())
    node1 = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(node1)
    node2 = TrueMonitor(name=PrefixedName("muh2"))
    msc.add_node(node2)
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(end)
    node1.end_condition = node2.observation_variable
    end.start_condition = cas.trinary_logic_and(
        node1.observation_variable, node2.observation_variable
    )
    msc.draw("muh.pdf")


@pytest.mark.skip(reason="not implemented yet")
def test_self_start_condition():
    pass


@pytest.mark.skip(reason="not implemented yet")
def test_all_conditions_with_goals():
    pass


@pytest.mark.skip(reason="not implemented yet")
def test_all_conditions_with_nodes():
    pass


@pytest.mark.skip(reason="not implemented yet")
def test_arrange_in_sequence():
    pass


@pytest.mark.skip(reason="not implemented yet")
def test_transition_hooks():
    pass


@pytest.mark.skip(reason="not implemented yet")
def test_optionality_of_qp_controller_in_compile():
    pass


@pytest.mark.skip(reason="not implemented yet")
def test_state_deletion():
    pass


def test_motion_statechart():
    msc = MotionStatechart(World())

    node1 = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(node1)
    node2 = TrueMonitor(name=PrefixedName("muh2"))
    msc.add_node(node2)
    node3 = TrueMonitor(name=PrefixedName("muh3"))
    msc.add_node(node3)
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(end)

    node1.start_condition = cas.trinary_logic_or(
        node3.observation_variable, node2.observation_variable
    )
    end.start_condition = node1.observation_variable
    msc.compile()

    assert len(msc.nodes) == 4
    assert len(msc.edges) == 3
    msc.tick_until_end()

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


def test_duplicate_name():
    msc = MotionStatechart(World())

    with pytest.raises(ValueError):
        cas.FloatVariable(name=PrefixedName("muh"))
        msc.add_node(TrueMonitor(name=PrefixedName("muh")))
        msc.add_node(TrueMonitor(name=PrefixedName("muh")))


def test_print():
    msc = MotionStatechart(World())
    print_node1 = Print(name=PrefixedName("cow"), message="muh")
    msc.add_node(print_node1)
    print_node2 = Print(name=PrefixedName("cow2"), message="muh")
    msc.add_node(print_node2)

    node1 = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(node1)
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(end)

    node1.start_condition = print_node1.observation_variable
    print_node2.start_condition = node1.observation_variable
    end.start_condition = print_node2.observation_variable
    msc.compile()

    assert len(msc.nodes) == 4
    assert len(msc.edges) == 3

    assert print_node1.observation_state == ObservationStateValues.UNKNOWN
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert print_node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert print_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert print_node1.observation_state == ObservationStateValues.UNKNOWN
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert print_node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert print_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert print_node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.TRUE
    assert print_node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.TRUE
    assert print_node2.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert not msc.is_end_motion()

    msc.tick()
    assert print_node1.observation_state == ObservationStateValues.TRUE
    assert node1.observation_state == ObservationStateValues.TRUE
    assert print_node2.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.TRUE

    assert print_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert print_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert msc.is_end_motion()


def test_cancel_motion():
    msc = MotionStatechart(World())
    node1 = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(node1)
    cancel = CancelMotion(name=PrefixedName("done"), exception=Exception("test"))
    msc.add_node(cancel)
    cancel.start_condition = node1.observation_variable

    msc.compile()
    msc.tick()  # first tick, cancel motion node1 turns true
    msc.tick()  # second tick, cancel goes into running
    with pytest.raises(Exception):
        msc.tick()  # third tick, cancel goes true and triggers
    msc.draw("muh.pdf")


def test_joint_goal():
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
            name=PrefixedName("dof", "a"), lower_limits=ll, upper_limits=ul
        )
        world.add_degree_of_freedom(dof)
        root_C_tip = RevoluteConnection(
            parent=root, child=tip, axis=cas.Vector3.Z(), dof_name=dof.name
        )
        world.add_connection(root_C_tip)

        dof = DegreeOfFreedom(
            name=PrefixedName("dof", "b"), lower_limits=ll, upper_limits=ul
        )
        world.add_degree_of_freedom(dof)
        root_C_tip2 = RevoluteConnection(
            parent=root, child=tip2, axis=cas.Vector3.Z(), dof_name=dof.name
        )
        world.add_connection(root_C_tip2)

    msc = MotionStatechart(world)

    task1 = JointPositionList(
        name=PrefixedName("task1"), goal_state=JointState({root_C_tip: 1})
    )
    always_true = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(always_true)
    msc.add_node(task1)
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(end)

    task1.start_condition = always_true.observation_variable
    end.start_condition = cas.trinary_logic_and(
        task1.observation_variable, always_true.observation_variable
    )

    msc.compile(QPControllerConfig.create_default_with_50hz())

    assert task1.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert task1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    msc.draw("muh.pdf")
    msc.tick_until_end()
    msc.draw("muh.pdf")
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
        msc.history.get_life_cycle_history_of_node(task1)[-1] == LifeCycleValues.RUNNING
    )
    assert (
        msc.history.get_life_cycle_history_of_node(end)[-1] == LifeCycleValues.RUNNING
    )


def test_reset():
    msc = MotionStatechart(World())
    node1 = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(node1)
    node2 = TrueMonitor(name=PrefixedName("muh2"))
    msc.add_node(node2)
    node3 = TrueMonitor(name=PrefixedName("muh3"))
    msc.add_node(node3)
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(end)
    node1.reset_condition = node2.observation_variable
    node2.start_condition = node1.observation_variable
    node3.start_condition = node2.observation_variable
    node2.end_condition = node2.observation_variable
    end.start_condition = cas.trinary_logic_and(
        node1.observation_variable,
        node2.observation_variable,
        node3.observation_variable,
    )

    msc.compile()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert node2.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node2.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert node2.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert node2.life_cycle_state == LifeCycleValues.DONE
    assert node3.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert node2.observation_state == ObservationStateValues.TRUE
    assert node3.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node2.life_cycle_state == LifeCycleValues.DONE
    assert node3.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert node2.observation_state == ObservationStateValues.TRUE
    assert node3.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert node2.life_cycle_state == LifeCycleValues.DONE
    assert node3.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert node2.observation_state == ObservationStateValues.TRUE
    assert node3.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.TRUE
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert node2.life_cycle_state == LifeCycleValues.DONE
    assert node3.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert msc.is_end_motion()


def test_nested_goals():
    msc = MotionStatechart(World())

    node1 = TrueMonitor(name=PrefixedName("start"))
    msc.add_node(node1)

    # inner goal with two sub-nodes
    inner = Goal(name=PrefixedName("inner"))
    sub_node1 = TrueMonitor(name=PrefixedName("inner sub 1"))
    msc.add_node(sub_node1)
    sub_node2 = TrueMonitor(name=PrefixedName("inner sub 2"))
    msc.add_node(sub_node2)
    inner.add_node(sub_node1)
    inner.add_node(sub_node2)
    sub_node1.end_condition = sub_node1.observation_variable
    sub_node2.start_condition = sub_node1.observation_variable
    inner.build = lambda context: NodeArtifacts(
        observation=sub_node2.observation_variable
    )

    # outer goal that contains the inner goal as a node
    outer = Goal(name=PrefixedName("outer"))
    msc.add_node(outer)
    outer.add_node(inner)
    outer.build = lambda context: NodeArtifacts(observation=inner.observation_variable)
    outer.start_condition = node1.observation_variable

    end = EndMotion(name=PrefixedName("done nested"))
    msc.add_node(end)
    end.start_condition = outer.observation_variable

    # compile and check initial states
    msc.compile()
    msc.draw("muh.pdf")
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert inner.observation_state == ObservationStateValues.UNKNOWN
    assert outer.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert sub_node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert inner.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert outer.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    # tick 1: start trigger begins running
    msc.tick()
    msc.draw("muh.pdf")
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert inner.observation_state == ObservationStateValues.UNKNOWN
    assert outer.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert inner.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert outer.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    # tick 2: start trigger turns true; inner sub_node1 already resolves to True and sub_node2 starts
    msc.tick()
    msc.draw("muh.pdf")
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert inner.observation_state == ObservationStateValues.UNKNOWN
    assert outer.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert inner.life_cycle_state == LifeCycleValues.RUNNING
    assert outer.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    # tick 3: inner sub_node2 turns true (inner goal still evaluating)
    msc.tick()
    msc.draw("muh.pdf")
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert inner.observation_state == ObservationStateValues.UNKNOWN
    assert outer.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert inner.life_cycle_state == LifeCycleValues.RUNNING
    assert outer.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    # tick 4: inner sub_node2 turns true
    msc.tick()
    msc.draw("muh.pdf")
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.TRUE
    assert inner.observation_state == ObservationStateValues.UNKNOWN
    assert outer.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert inner.life_cycle_state == LifeCycleValues.RUNNING
    assert outer.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    # tick 5: inner goal becomes true, outer still running, end starts running
    msc.tick()
    msc.draw("muh.pdf")
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.TRUE
    assert inner.observation_state == ObservationStateValues.TRUE
    assert outer.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert inner.life_cycle_state == LifeCycleValues.RUNNING
    assert outer.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    # tick 6: outer goal becomes true; end still running
    msc.tick()
    msc.draw("muh.pdf")
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.TRUE
    assert inner.observation_state == ObservationStateValues.TRUE
    assert outer.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.UNKNOWN

    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert inner.life_cycle_state == LifeCycleValues.RUNNING
    assert outer.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert not msc.is_end_motion()

    # tick 7: end motion becomes true
    msc.tick()
    msc.draw("muh.pdf")
    assert end.observation_state == ObservationStateValues.TRUE
    assert msc.is_end_motion()


@dataclass(eq=False, repr=False)
class _TestThreadMonitor(ThreadPayloadMonitor):
    delay: float = 0.05
    return_value: float = ObservationStateValues.TRUE

    def _compute_observation(self):
        time.sleep(self.delay)
        return self.return_value


def test_thread_payload_monitor_non_blocking_and_caching():
    msc = MotionStatechart(World())
    mon = _TestThreadMonitor(
        name=PrefixedName("thread_mon"),
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


@pytest.mark.skip(reason="Not working yet")
def test_thread_payload_monitor_integration():
    msc = MotionStatechart(World())
    mon = _TestThreadMonitor(
        name=PrefixedName("thread_mon2"),
        delay=0.03,
        return_value=ObservationStateValues.TRUE,
    )
    msc.add_node(mon)
    end = EndMotion(name=PrefixedName("done thread"))
    msc.add_node(end)
    end.start_condition = mon.observation_variable

    msc.compile()

    # tick 1: monitor not started yet becomes RUNNING; end not started
    msc.tick()
    assert mon.observation_state == ObservationStateValues.UNKNOWN
    assert mon.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED

    # tick 2: compute_observation is triggered asynchronously; still Unknown immediately
    msc.tick()
    assert mon.observation_state == ObservationStateValues.UNKNOWN
    assert mon.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED

    # allow background to finish and propagate on next tick
    time.sleep(mon.delay * 2)
    msc.tick()
    assert mon.observation_state == ObservationStateValues.TRUE
    assert end.life_cycle_state == LifeCycleValues.RUNNING

    # next tick the EndMotion should turn true
    msc.tick()
    assert end.observation_state == ObservationStateValues.TRUE


def test_goal():
    msc = MotionStatechart(World())

    node1 = TrueMonitor(name=PrefixedName("muh"))
    msc.add_node(node1)

    goal = Goal(name=PrefixedName("goal"))
    msc.add_node(goal)
    sub_node1 = TrueMonitor(name=PrefixedName("sub muh1"))
    goal.add_node(sub_node1)
    sub_node2 = TrueMonitor(name=PrefixedName("sub muh2"))
    goal.add_node(sub_node2)
    goal.add_node(sub_node1)
    goal.add_node(sub_node2)
    sub_node1.end_condition = sub_node1.observation_variable
    sub_node2.start_condition = sub_node1.observation_variable
    goal.build = lambda context: NodeArtifacts(
        observation=sub_node2.observation_variable
    )
    goal.start_condition = node1.observation_variable

    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(end)
    end.start_condition = goal.observation_variable

    msc.compile()
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert goal.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert sub_node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert goal.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert goal.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert goal.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.UNKNOWN
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert goal.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node2.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert goal.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.UNKNOWN
    assert goal.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert goal.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.TRUE
    assert goal.observation_state == ObservationStateValues.UNKNOWN
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert goal.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.TRUE
    assert goal.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.UNKNOWN
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert goal.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert not msc.is_end_motion()

    msc.tick()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert sub_node1.observation_state == ObservationStateValues.TRUE
    assert sub_node2.observation_state == ObservationStateValues.TRUE
    assert goal.observation_state == ObservationStateValues.TRUE
    assert end.observation_state == ObservationStateValues.TRUE
    assert node1.life_cycle_state == LifeCycleValues.RUNNING
    assert sub_node1.life_cycle_state == LifeCycleValues.DONE
    assert sub_node2.life_cycle_state == LifeCycleValues.RUNNING
    assert goal.life_cycle_state == LifeCycleValues.RUNNING
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert msc.is_end_motion()
    msc.draw("muh.pdf")


def test_set_seed_configuration(pr2_world):
    msc = MotionStatechart(pr2_world)
    goal = 0.1

    connection: ActiveConnection1DOF = pr2_world.get_connection_by_name(
        "torso_lift_joint"
    )

    node1 = SetSeedConfiguration(
        name=PrefixedName("muh"), seed_configuration=JointState({connection: goal})
    )
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(node1)
    msc.add_node(end)
    node1.end_condition = node1.observation_variable
    end.start_condition = node1.observation_variable

    msc.compile()

    msc.tick_until_end()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert node1.life_cycle_state == LifeCycleValues.DONE
    assert end.observation_state == ObservationStateValues.TRUE
    assert end.life_cycle_state == LifeCycleValues.RUNNING

    assert np.isclose(connection.position, goal)


def test_set_seed_odometry(pr2_world):
    msc = MotionStatechart(pr2_world)

    goal = TransformationMatrix.from_xyz_rpy(
        x=1, y=-1, z=1, roll=1, pitch=1, yaw=1, reference_frame=pr2_world.root
    )
    expected = TransformationMatrix.from_xyz_rpy(
        x=1, y=-1, yaw=1, reference_frame=pr2_world.root
    )

    node1 = SetOdometry(
        name=PrefixedName("muh"),
        base_pose=goal,
    )
    end = EndMotion(name=PrefixedName("done"))
    msc.add_node(node1)
    msc.add_node(end)
    node1.end_condition = node1.observation_variable
    end.start_condition = node1.observation_variable

    msc.compile()

    msc.tick_until_end()
    assert node1.observation_state == ObservationStateValues.TRUE
    assert node1.life_cycle_state == LifeCycleValues.DONE
    assert end.observation_state == ObservationStateValues.TRUE
    assert end.life_cycle_state == LifeCycleValues.RUNNING

    assert np.allclose(
        expected.to_np(),
        pr2_world.compute_forward_kinematics_np(
            pr2_world.root, node1.odom_connection.child
        ),
    )


def test_continuous_joint(pr2_world):
    msc = MotionStatechart(pr2_world)
    joint_goal = JointPositionList(
        name=PrefixedName("joint_goal"),
        goal_state=JointState.from_str_dict(
            {
                "r_wrist_roll_joint": -np.pi,
                "l_wrist_roll_joint": -2.1 * np.pi,
            },
            world=pr2_world,
        ),
    )
    msc.add_node(joint_goal)
    end = EndMotion(name=PrefixedName("end"))
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable
    msc.compile(QPControllerConfig.create_default_with_50hz())
    msc.tick_until_end()


def test_revolute_joint(pr2_world):
    msc = MotionStatechart(pr2_world)
    joint_goal = JointPositionList(
        name=PrefixedName("joint_goal"),
        goal_state=JointState.from_str_dict(
            {
                "head_pan_joint": 0.041880780651479044,
                "head_tilt_joint": -0.37,
            },
            world=pr2_world,
        ),
    )
    msc.add_node(joint_goal)
    end = EndMotion(name=PrefixedName("end"))
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable
    msc.compile(QPControllerConfig.create_default_with_50hz())
    msc.tick_until_end()


def test_cart_goal_1eef(pr2_world: World):
    tip = pr2_world.get_kinematic_structure_entity_by_name("r_gripper_tool_frame")
    root = pr2_world.get_kinematic_structure_entity_by_name("base_footprint")
    tip_goal = TransformationMatrix.from_xyz_quaternion(pos_x=-0.2, reference_frame=tip)

    msc = MotionStatechart(pr2_world)
    cart_goal = CartesianPose(
        name=PrefixedName("cart_goal"),
        root_link=root,
        tip_link=tip,
        goal_pose=tip_goal,
    )
    msc.add_node(cart_goal)
    end = EndMotion(name=PrefixedName("end"))
    msc.add_node(end)
    end.start_condition = cart_goal.observation_variable

    msc.compile(QPControllerConfig.create_default_with_50hz())
    msc.tick_until_end()


def test_cart_goal_sequence_at_build(pr2_world: World):
    tip = pr2_world.get_kinematic_structure_entity_by_name("base_footprint")
    root = pr2_world.get_kinematic_structure_entity_by_name("odom_combined")

    tip_goal1 = TransformationMatrix.from_xyz_quaternion(
        pos_x=-0.2, reference_frame=tip
    )
    tip_goal2 = TransformationMatrix.from_xyz_quaternion(pos_x=0.2, reference_frame=tip)

    msc = MotionStatechart(pr2_world)
    cart_goal1 = CartesianPose(
        name=PrefixedName("cart_goal1"),
        root_link=root,
        tip_link=tip,
        goal_pose=tip_goal1,
    )
    msc.add_node(cart_goal1)

    cart_goal2 = CartesianPose(
        name=PrefixedName("cart_goal2"),
        root_link=root,
        tip_link=tip,
        goal_pose=tip_goal2,
        binding_policy=GoalBindingPolicy.Bind_at_build,
    )
    msc.add_node(cart_goal2)

    cart_goal1.end_condition = cart_goal1.observation_variable
    cart_goal2.start_condition = cart_goal1.observation_variable

    end = EndMotion(name=PrefixedName("end"))
    msc.add_node(end)
    end.start_condition = cas.trinary_logic_and(
        cart_goal1.observation_variable, cart_goal2.observation_variable
    )
    msc.compile(QPControllerConfig.create_default_with_50hz())
    msc.tick_until_end()

    fk = pr2_world.compute_forward_kinematics_np(root, tip)
    assert np.allclose(fk, tip_goal2.to_np(), atol=cart_goal2.threshold)


def test_cart_goal_sequence_on_start(pr2_world: World):
    tip = pr2_world.get_kinematic_structure_entity_by_name("base_footprint")
    root = pr2_world.get_kinematic_structure_entity_by_name("odom_combined")

    tip_goal1 = TransformationMatrix.from_xyz_quaternion(
        pos_x=-0.2, reference_frame=tip
    )
    tip_goal2 = TransformationMatrix.from_xyz_quaternion(pos_x=0.2, reference_frame=tip)

    msc = MotionStatechart(pr2_world)
    cart_goal1 = CartesianPose(
        name=PrefixedName("cart_goal1"),
        root_link=root,
        tip_link=tip,
        goal_pose=tip_goal1,
    )
    msc.add_node(cart_goal1)

    cart_goal2 = CartesianPose(
        name=PrefixedName("cart_goal2"),
        root_link=root,
        tip_link=tip,
        goal_pose=tip_goal2,
    )
    msc.add_node(cart_goal2)

    cart_goal1.end_condition = cart_goal1.observation_variable
    cart_goal2.start_condition = cart_goal1.observation_variable

    end = EndMotion(name=PrefixedName("end"))
    msc.add_node(end)
    end.start_condition = cas.trinary_logic_and(
        cart_goal1.observation_variable, cart_goal2.observation_variable
    )
    msc.compile(QPControllerConfig.create_default_with_50hz())
    msc.tick_until_end()

    fk = pr2_world.compute_forward_kinematics_np(root, tip)
    expected = np.eye(4)
    assert np.allclose(fk, expected, atol=cart_goal2.threshold)


def test_CartesianOrientation(pr2_world: World):
    tip = pr2_world.get_kinematic_structure_entity_by_name("base_footprint")
    root = pr2_world.get_kinematic_structure_entity_by_name("odom_combined")

    tip_goal = cas.RotationMatrix.from_axis_angle(
        cas.Vector3.Z(), 4.0, reference_frame=tip
    )

    msc = MotionStatechart(pr2_world)
    cart_goal = CartesianOrientation(
        name=PrefixedName("cart_goal"),
        root_link=root,
        tip_link=tip,
        goal_orientation=tip_goal,
    )
    msc.add_node(cart_goal)
    end = EndMotion(name=PrefixedName("end"))
    msc.add_node(end)
    end.start_condition = cart_goal.observation_variable
    msc.compile(QPControllerConfig.create_default_with_50hz())
    msc.tick_until_end()

    fk = pr2_world.compute_forward_kinematics_np(root, tip)
    assert np.allclose(fk, tip_goal.to_np(), atol=cart_goal.threshold)


def test_pointing(pr2_world: World):
    tip = pr2_world.get_kinematic_structure_entity_by_name("r_gripper_tool_frame")
    root = pr2_world.get_kinematic_structure_entity_by_name("odom_combined")

    msc = MotionStatechart(pr2_world)

    goal_point = cas.Point3(2, 0, 0, reference_frame=root)
    pointing_axis = cas.Vector3.X(reference_frame=tip)

    pointing = Pointing(
        name=PrefixedName("pointing"),
        root_link=root,
        tip_link=tip,
        goal_point=goal_point,
        pointing_axis=pointing_axis,
    )
    msc.add_node(pointing)
    end = EndMotion(name=PrefixedName("end"))
    msc.add_node(end)
    end.start_condition = pointing.observation_variable
    msc.compile(QPControllerConfig.create_default_with_50hz())
    msc.tick_until_end()


def test_transition_triggers():
    msc = MotionStatechart(World())

    changer = ChangeStateOnEvents(name=PrefixedName("changer"))
    msc.add_node(changer)

    node1 = Pulse(name=PrefixedName("node1"))
    msc.add_node(node1)

    node2 = Pulse(name=PrefixedName("node2"))
    msc.add_node(node2)
    node2.start_condition = node1.observation_variable

    node3 = Pulse(name=PrefixedName("node3"))
    msc.add_node(node3)
    node3.start_condition = cas.trinary_logic_and(
        cas.trinary_logic_not(node1.observation_variable),
        cas.trinary_logic_not(node2.observation_variable),
    )

    node4 = Pulse(name=PrefixedName("node4"))
    msc.add_node(node4)
    node4.start_condition = node3.observation_variable

    changer.start_condition = node1.observation_variable
    changer.pause_condition = node2.observation_variable
    changer.end_condition = node3.observation_variable
    changer.reset_condition = node4.observation_variable

    msc.compile()

    assert changer.state is None

    msc.tick()
    msc.draw("muh.pdf")
    msc.tick()
    msc.draw("muh.pdf")
    assert changer.life_cycle_state == LifeCycleValues.RUNNING
    assert changer.state == "on_start"

    msc.tick()
    msc.draw("muh.pdf")
    assert changer.life_cycle_state == LifeCycleValues.PAUSED
    assert changer.state == "on_pause"

    msc.tick()
    msc.draw("muh.pdf")
    assert changer.life_cycle_state == LifeCycleValues.RUNNING
    assert changer.state == "on_unpause"

    msc.tick()
    msc.draw("muh.pdf")
    assert changer.life_cycle_state == LifeCycleValues.DONE
    assert changer.state == "on_end"

    msc.tick()
    msc.draw("muh.pdf")
    assert changer.life_cycle_state == LifeCycleValues.NOT_STARTED
    assert changer.state == "on_reset"
