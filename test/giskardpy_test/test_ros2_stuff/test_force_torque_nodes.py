import json
from copy import deepcopy

import numpy as np
from geometry_msgs.msg import WrenchStamped

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceImpactMonitor,
    ForceTorqueSensorUpdater,
)
from giskardpy.motion_statechart.ros2_nodes.topic_monitor import (
    PublishOnStart,
    WaitForMessage,
)
from giskardpy.motion_statechart.ros_context import RosContextExtension
from giskardpy.ros_executor import Ros2Executor
from semantic_digital_twin.robots.abstract_robot import ForceTorqueSensor
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world import World


def test_force_torque_sensor_updater_writes_wrench_into_annotation(
    hsr_world_setup, rclpy_node
):
    """The updater resolves the robot's force/torque sensor from its frame and,
    on tick, writes each received wrench into that sensor's live value."""
    world = deepcopy(hsr_world_setup)
    hsrb = HSRB.from_world(world)
    sensor = next(s for s in hsrb.sensors if isinstance(s, ForceTorqueSensor))

    updater = ForceTorqueSensorUpdater(
        topic_name="/test_updater/wrench",
        reference_frame=sensor.root,
        name="ft_updater",
    )
    context = MotionStatechartContext(world=world)
    context.add_extension(RosContextExtension(rclpy_node))
    updater.build(context)

    message = WrenchStamped()
    message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = (
        1.0,
        2.0,
        3.0,
    )
    message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = (
        4.0,
        5.0,
        6.0,
    )
    setattr(updater, "_TopicSubscriberNode__last_msg", message)

    assert not sensor.has_received_wrench
    updater.on_tick(context)

    assert sensor.has_received_wrench
    np.testing.assert_array_equal(sensor.force.evaluate().flatten()[:3], [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(
        sensor.torque.evaluate().flatten()[:3], [4.0, 5.0, 6.0]
    )


def test_force_impact_node(rclpy_node):
    topic_name = "force_torque_topic"

    msg_below = WrenchStamped()

    msg_above = WrenchStamped()
    msg_above.wrench.force.x = 20.0

    msc = MotionStatechart()
    msc.add_node(
        parallel := Parallel(
            [
                ForceImpactMonitor(topic_name=topic_name, threshold=10),
                Sequence(
                    nodes=[
                        PublishOnStart(topic_name=topic_name, msg=msg_below),
                        WaitForMessage(topic_name=topic_name, msg_type=WrenchStamped),
                        PublishOnStart(topic_name=topic_name, msg=msg_above),
                    ]
                ),
            ]
        )
    )
    msc.add_node(EndMotion.when_true(parallel))

    json_data = msc.to_json()
    json_str = json.dumps(json_data)
    new_json_data = json.loads(json_str)
    msc_copy = MotionStatechart.from_json(new_json_data)

    kin_sim = Ros2Executor(
        context=MotionStatechartContext(world=World()), ros_node=rclpy_node
    )
    kin_sim.compile(motion_statechart=msc_copy)

    ft_node = msc_copy.nodes[0].nodes[0]

    kin_sim.tick_until_end(timeout=5_000)
    msc_copy.draw("muh.pdf")
    assert (
        msc_copy.history.get_observation_history_of_node(ft_node)[0]
        == ObservationStateValues.UNKNOWN
    )
    assert (
        ObservationStateValues.FALSE
        in msc_copy.history.get_observation_history_of_node(ft_node)
    )
    assert (
        msc_copy.history.get_observation_history_of_node(ft_node)[-1]
        == ObservationStateValues.TRUE
    )
