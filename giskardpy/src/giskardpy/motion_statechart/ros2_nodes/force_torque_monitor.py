from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from geometry_msgs.msg import WrenchStamped

from semantic_digital_twin.spatial_types import Vector3
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
class ForceTorqueSymbolNode(ForceTorqueNode):
    """
    Subscribes to a wrench topic and exposes the latest force and torque as
    symbolic ``Vector3`` expressions.

    The data flow follows the ``float_variable_data`` pattern (analogous to
    collision avoidance): each tick writes the latest wrench into
    ``context.float_variable_data``; the QP solver picks up the new values
    via the registered symbols on the next solve. Downstream tasks reference
    force and torque directly in their constraint expressions.

    The symbolic ``Vector3`` instances are created in ``__post_init__`` (not
    in ``build``) so consumers may reference them during their own ``build``
    phase regardless of node-add order.
    """

    reference_frame: KinematicStructureEntity = field(kw_only=True)
    """The kinematic frame the wrench is measured in (e.g. the FT sensor link)."""

    force: Vector3 = field(init=False, default=None)
    """Symbolic ``Vector3`` carrying the latest force [fx, fy, fz]."""

    torque: Vector3 = field(init=False, default=None)
    """Symbolic ``Vector3`` carrying the latest torque [tx, ty, tz]."""

    def __post_init__(self):
        super().__post_init__()
        self.force = Vector3.create_with_variables(f"{self.name}/force")
        self.force.reference_frame = self.reference_frame
        self.torque = Vector3.create_with_variables(f"{self.name}/torque")
        self.torque.reference_frame = self.reference_frame

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        context.float_variable_data.register_expression(self.force)
        context.float_variable_data.register_expression(self.torque)
        return artifacts

    def on_tick(
        self, context: MotionStatechartContext
    ) -> Optional[ObservationStateValues]:
        super().on_tick(context)
        print(f"[ft] has_msg={self.has_msg()} force={self.force_as_np() if self.has_msg() else None}")
        if not self.has_msg():
            return ObservationStateValues.UNKNOWN
        context.float_variable_data.set_value(self.force, self.force_as_np())
        context.float_variable_data.set_value(self.torque, self.torque_as_np())
        return ObservationStateValues.UNKNOWN
