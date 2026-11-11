"""
A ROS node whose executor spins on a background thread.

Kept apart from the demonstrations so that tools which only talk to ROS, such as the
scene report command, do not load the plan executors and their controller dependencies.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.world import World

# %% ros session

WORLD_FETCH_TIMEOUT_SECONDS = 300
"""
How long to wait for a controller to serve its world.

Matches giskardpy's own client (``giskardpy/middleware/ros2/giskard_client_executor.py``), which
waits this long for the same race: the world-fetcher server is still parsing the URDF and
starting up when a shorter budget would expire.
"""

SPIN_THREAD_JOIN_TIMEOUT_SECONDS = 5.0
"""
How long to wait for the executor's spin thread to exit after :meth:`Executor.shutdown`.

A daemon thread still running rclpy's native code when the interpreter starts finalizing
gets torn down mid-call, which crashes the process rather than raising a catchable
exception. Joining it here ensures it has actually left that code first.
"""


@dataclass
class RobotDemonstrationRosSession:
    """
    A ROS node whose executor is spun on a background thread.

    ..note:: Service responses are delivered by the executor, so a node that is not spun
        blocks every call until its timeout.
    """

    node: Node
    """
    The node that service calls and synchronizers are attached to.
    """

    executor: MultiThreadedExecutor
    """
    Executor delivering this node's callbacks, those of different callback groups at the
    same time.
    """

    owns_context: bool
    """
    Whether this session initialized rclpy and must therefore shut it down again.
    """

    spin_thread: threading.Thread = field(init=False)
    """
    Thread running :attr:`executor`'s spin loop, joined again in :meth:`stop`.
    """

    @classmethod
    def start(cls, node_name: str) -> RobotDemonstrationRosSession:
        """
        Create a node and spin its executor, initializing rclpy if nothing else has.

        :param node_name: Name to register the node under.
        """
        owns_context = not rclpy.ok()
        if owns_context:
            rclpy.init()
        node = rclpy.create_node(node_name)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        session = cls(node=node, executor=executor, owns_context=owns_context)
        session.spin_thread = threading.Thread(
            target=executor.spin, daemon=True, name=f"{node_name}-executor"
        )
        session.spin_thread.start()
        return session

    def fetch_world(
        self, timeout_seconds: float = WORLD_FETCH_TIMEOUT_SECONDS
    ) -> World:
        """
        Fetch the world that a controller is serving.

        :param timeout_seconds: How long to wait for the service to answer.
        """
        return fetch_world_from_service(node=self.node, timeout_seconds=timeout_seconds)

    def stop(self) -> None:
        """
        Destroy the node, and release the ROS context if this session created it.
        """
        self.executor.shutdown()
        self.spin_thread.join(timeout=SPIN_THREAD_JOIN_TIMEOUT_SECONDS)
        self.node.destroy_node()
        if self.owns_context and rclpy.ok():
            rclpy.shutdown()

    def __enter__(self) -> RobotDemonstrationRosSession:
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.stop()
