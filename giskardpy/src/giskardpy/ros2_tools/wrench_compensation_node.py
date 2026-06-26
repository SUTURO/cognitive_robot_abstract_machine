#!/usr/bin/env python3
"""Gravity/bias compensation for the HSR wrist force-torque sensor.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

GRAVITY = 9.81

RAW_WRENCH_TOPIC = "/wrist_wrench/raw"
COMPENSATED_WRENCH_TOPIC = "/wrist_wrench/compensated"


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from an (x, y, z, w) quaternion."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass
class FTCompensationParams:
    """Static gravity-compensation parameters for one tool/sensor, all in the
    sensor frame. The identified model of a resting reading is

        force_raw  = force_offset  + mass * g_sensor
        torque_raw = torque_offset + first_moment x g_sensor

    where ``g_sensor`` is gravity expressed in the sensor frame. This sensor
    reports the gravity force as ``+mass * g_sensor`` (sign verified against a
    z-down sample).

    Defaults are the HSR wrist values identified on ``tf_ft_bag_BA`` (2026-06-19,
    6549 settled samples, RMS residual 0.494 N / 0.0265 Nm).
    """

    mass: float = 0.5945
    """Sensor-distal load mass in kg (weight ~5.83 N)."""

    first_moment: np.ndarray = field(
        default_factory=lambda: np.array([-0.0002, -0.0001, -0.0246])
    )
    """First mass moment ``mass * com`` in kg*m; COM ~4 cm down the sensor z-axis."""

    force_offset: np.ndarray = field(
        default_factory=lambda: np.array([-8.378, 13.673, -67.203])
    )
    """Constant sensor force bias in N."""

    torque_offset: np.ndarray = field(
        default_factory=lambda: np.array([0.0526, -0.0081, 0.0526])
    )
    """Constant sensor torque bias in Nm."""

    def __post_init__(self):
        self.first_moment = np.asarray(self.first_moment, dtype=float).reshape(3)
        self.force_offset = np.asarray(self.force_offset, dtype=float).reshape(3)
        self.torque_offset = np.asarray(self.torque_offset, dtype=float).reshape(3)


@dataclass
class WrenchGravityCompensator:
    """Removes the static gravity load, constant sensor bias, and a per-episode
    re-tare bias from a raw wrench, leaving the external contact wrench. Pure
    numpy, no ROS.

    ``base_R_sensor`` rotates sensor-frame vectors into a gravity-aligned base
    frame (its columns are the sensor axes in the base frame).
    """

    params: FTCompensationParams = field(default_factory=FTCompensationParams)
    gravity: float = GRAVITY
    bias_force: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bias_torque: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        self.bias_force = np.asarray(self.bias_force, dtype=float).reshape(3)
        self.bias_torque = np.asarray(self.bias_torque, dtype=float).reshape(3)

    def gravity_in_sensor(self, base_R_sensor: np.ndarray) -> np.ndarray:
        g_base = np.array([0.0, 0.0, -self.gravity])
        return np.asarray(base_R_sensor, dtype=float).reshape(3, 3).T @ g_base

    def gravity_compensated(
        self,
        force_raw: np.ndarray,
        torque_raw: np.ndarray,
        base_R_sensor: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Gravity and constant offset removed; the per-episode bias is not yet
        applied. This is the residual a re-tare averages."""
        g_sensor = self.gravity_in_sensor(base_R_sensor)
        force = (
            np.asarray(force_raw, dtype=float).reshape(3)
            - self.params.force_offset
            - self.params.mass * g_sensor
        )
        torque = (
            np.asarray(torque_raw, dtype=float).reshape(3)
            - self.params.torque_offset
            - np.cross(self.params.first_moment, g_sensor)
        )
        return force, torque

    def compensate(
        self,
        force_raw: np.ndarray,
        torque_raw: np.ndarray,
        base_R_sensor: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """External contact wrench ``(force, torque)`` in the sensor frame."""
        force, torque = self.gravity_compensated(force_raw, torque_raw, base_R_sensor)
        return force - self.bias_force, torque - self.bias_torque


@dataclass
class BiasEstimator:
    """Averages gravity-compensated residuals collected while the arm is held
    still, producing a per-episode bias to subtract. Rejects the window if the
    force varies beyond ``stationary_force_std``, which means the arm moved.
    """

    required_samples: int = 100
    stationary_force_std: float = 0.5

    _force: List[np.ndarray] = field(default_factory=list, init=False)
    _torque: List[np.ndarray] = field(default_factory=list, init=False)

    def reset(self) -> None:
        self._force.clear()
        self._torque.clear()

    def add(self, force: np.ndarray, torque: np.ndarray) -> bool:
        """Add one residual; returns True once the window is full."""
        self._force.append(np.asarray(force, dtype=float).reshape(3))
        self._torque.append(np.asarray(torque, dtype=float).reshape(3))
        return len(self._force) >= self.required_samples

    def result(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Mean ``(bias_force, bias_torque)``, or None if the window is not full
        or the arm was not stationary."""
        if len(self._force) < self.required_samples:
            return None
        force = np.array(self._force)
        if force.std(axis=0).max() > self.stationary_force_std:
            return None
        return force.mean(axis=0), np.array(self._torque).mean(axis=0)


# --- ROS 2 shell -----------------------------------------------------------

try:
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:  # numpy core stays importable without a ROS install
    Node = object


@dataclass(eq=False)
class WrenchCompensationNode(Node):
    """ROS 2 node: subscribe a raw ``WrenchStamped``, subtract the identified
    gravity load and sensor bias using the live sensor orientation from tf, and
    republish the external contact wrench.

    The output stays in the sensor frame (``header.frame_id`` unchanged), so a
    downstream consumer that rotates it itself (e.g. the admittance task's
    ``goal_T_sensor``) keeps working unchanged. Point ``ForceTorqueSymbolNode``
    at ``topic_out`` and its ``reference_frame`` at the sensor link.
    """

    topic_in: str = RAW_WRENCH_TOPIC
    topic_out: str = COMPENSATED_WRENCH_TOPIC
    gravity_frame: str = "base_footprint"
    """Gravity-aligned frame; its z is assumed up."""
    sensor_frame: str = "wrist_ft_sensor_frame"
    """Fallback sensor frame when the incoming message has no ``frame_id``."""
    params: FTCompensationParams = field(default_factory=FTCompensationParams)
    qos_depth: int = 1
    """Keep the queue shallow so a stalled consumer can't build up latency."""
    retare_samples: int = 100
    retare_timeout: float = 5.0
    """Seconds the re-tare service waits for its sample window to fill."""

    def __post_init__(self) -> None:
        super().__init__("wrench_compensation")
        self.compensator = WrenchGravityCompensator(self.params)
        self.estimator = BiasEstimator(required_samples=self.retare_samples)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._lock = threading.Lock()
        self._collecting = False
        self._retare_done = threading.Event()
        self._retare_success = False
        self._retare_message = ""

        # A reliable publisher is accepted by reliable and best-effort
        # subscribers; subscribe best-effort to accept either kind of sensor.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
        )
        # One reentrant group lets the subscription keep delivering while the
        # blocking re-tare service waits for its window (MultiThreadedExecutor).
        group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            WrenchStamped, self.topic_in, self._on_msg, sub_qos, callback_group=group
        )
        self.pub = self.create_publisher(WrenchStamped, self.topic_out, pub_qos)
        self.retare_service = self.create_service(
            Trigger, "~/retare", self._on_retare, callback_group=group
        )
        self.get_logger().info(
            f"WrenchCompensationNode: {self.topic_in} -> {self.topic_out}"
        )

    def _base_R_sensor(self, sensor_frame: str) -> np.ndarray | None:
        # Latest available transform: the wrist reorients slowly relative to
        # 100 Hz, and Time() avoids extrapolation throws while the arm moves.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.gravity_frame, sensor_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"tf {self.gravity_frame} <- {sensor_frame} unavailable: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        q = tf.transform.rotation
        return quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

    def _on_msg(self, msg: WrenchStamped) -> None:
        sensor_frame = msg.header.frame_id or self.sensor_frame
        base_R_sensor = self._base_R_sensor(sensor_frame)
        if base_R_sensor is None:
            return
        f_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        t_raw = np.array(
            [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        )
        force, torque = self.compensator.gravity_compensated(f_raw, t_raw, base_R_sensor)

        with self._lock:
            if self._collecting and self.estimator.add(force, torque):
                self._finalize_retare()

        out = WrenchStamped()
        out.header = msg.header
        contact_force = force - self.compensator.bias_force
        contact_torque = torque - self.compensator.bias_torque
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = map(
            float, contact_force
        )
        out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = map(
            float, contact_torque
        )
        self.pub.publish(out)

    def _on_retare(self, request, response):
        """Zero the residual for this episode: hold the arm still in free space,
        then call this. Blocks until the window fills or times out."""
        with self._lock:
            self.estimator.reset()
            self._retare_done.clear()
            self._collecting = True
        if not self._retare_done.wait(self.retare_timeout):
            with self._lock:
                self._collecting = False
            response.success = False
            response.message = "re-tare timed out waiting for samples"
            return response
        response.success = self._retare_success
        response.message = self._retare_message
        return response

    def _finalize_retare(self) -> None:
        # Called while holding self._lock.
        self._collecting = False
        result = self.estimator.result()
        if result is None:
            self._retare_success = False
            self._retare_message = "arm not stationary; bias unchanged"
        else:
            self.compensator.bias_force, self.compensator.bias_torque = result
            self._retare_success = True
            self._retare_message = (
                f"bias force={self.compensator.bias_force.round(3).tolist()} "
                f"torque={self.compensator.bias_torque.round(4).tolist()}"
            )
        self.get_logger().info(f"re-tare: {self._retare_message}")
        self._retare_done.set()


def main():
    rclpy.init()
    node = WrenchCompensationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
