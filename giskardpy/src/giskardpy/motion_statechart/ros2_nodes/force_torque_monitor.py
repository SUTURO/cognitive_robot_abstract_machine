from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from geometry_msgs.msg import WrenchStamped

from semantic_digital_twin.robots.abstract_robot import ForceTorqueSensor
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

from giskardpy.motion_statechart.ros2_nodes.topic_monitor import TopicSubscriberNode
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.graph_node import NodeArtifacts


@dataclass(eq=False, repr=False)
class ForceTorqueNode(TopicSubscriberNode[WrenchStamped]):
    """
    Superclass for all nodes that subscribe to a ROS topic that contains force and torque data.
    """

    msg_type: WrenchStamped = field(init=False, default=WrenchStamped)

    def force_as_np(self) -> np.ndarray:
        return np.array(
            [
                self.current_msg.wrench.force.x,
                self.current_msg.wrench.force.y,
                self.current_msg.wrench.force.z,
            ]
        )

    def force_magnitude(self) -> float:
        return float(np.linalg.norm(self.force_as_np()))

    def torque_as_np(self) -> np.ndarray:
        return np.array(
            [
                self.current_msg.wrench.torque.x,
                self.current_msg.wrench.torque.y,
                self.current_msg.wrench.torque.z,
            ]
        )

    def torque_magnitude(self) -> float:
        return float(np.linalg.norm(self.torque_as_np()))


@dataclass(eq=False, repr=False)
class ForceImpactMonitor(ForceTorqueNode):
    """
    This node checks if the force magnitude is above a threshold.
    """

    threshold: float = field(kw_only=True)
    """Force magnitude in N above which the observation turns true."""

    def on_tick(
        self, context: MotionStatechartContext
    ) -> Optional[ObservationStateValues]:
        super().on_tick(context)
        if not self.has_msg():
            return ObservationStateValues.UNKNOWN
        if self.force_magnitude() > self.threshold:
            return ObservationStateValues.TRUE
        return ObservationStateValues.FALSE


@dataclass(eq=False, repr=False)
class ForceTorqueSensorUpdater(ForceTorqueNode):
    """
    Feeds a robot's :class:`ForceTorqueSensor` annotation from a wrench topic.

    On every tick it writes the latest wrench into ``world.sensor_inputs`` through
    the annotation, so admittance and contact tasks read a live, fixed-size sensor
    input rather than a per-goal symbol. This is the motion-statechart feeder used
    where the giskard server ticks the serialized chart; in simulation the wrench is
    written directly and this node stays inert (no publisher on the topic).

    .. todo:: Remove once giskard lives inside cram. The node only exists because the
        separate giskard server has to update its own world model from the chart;
        :class:`~semantic_digital_twin.adapters.ros.force_torque_sensor_subscriber.ForceTorqueSensorSubscriber`
        then becomes the single feeder, independent of any ticking motion.
    """

    reference_frame: KinematicStructureEntity = field(kw_only=True)
    """Root frame of the force/torque sensor annotation the wrench is written to."""

    _sensor: ForceTorqueSensor = field(init=False, default=None)
    """Annotation resolved from ``reference_frame`` against the executed world."""

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        self._sensor = ForceTorqueSensor.with_root(context.world, self.reference_frame)
        return artifacts

    def on_tick(
        self, context: MotionStatechartContext
    ) -> Optional[ObservationStateValues]:
        super().on_tick(context)
        if not self.has_msg():
            return ObservationStateValues.UNKNOWN
        self._sensor.write_wrench(self.force_as_np(), self.torque_as_np())
        return ObservationStateValues.UNKNOWN
