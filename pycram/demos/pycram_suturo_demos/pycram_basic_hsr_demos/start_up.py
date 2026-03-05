import logging
from typing_extensions import Tuple, Any

from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world import World

logger = logging.getLogger(__name__)


import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor


def setup_hsrb_context(
    node_name: str = "pycram_node",
) -> Tuple[Any, World, HSRB, Context]:
    """
    Initializes rclpy, starts a SingleThreadedExecutor in a background thread,
    synchronizes the world model, and returns all relevant objects.

    Returns:
        dict containing:
            - node
            - world
            - robot_view
            - context
    """

    # Initialize ROS 2
    rclpy.init()

    # Create node
    rclpy_node = rclpy.create_node(node_name)

    # Create executor
    executor = SingleThreadedExecutor()
    executor.add_node(rclpy_node)

    # Start executor in background thread
    thread = threading.Thread(
        target=executor.spin,
        daemon=True,
        name="rclpy-executor",
    )
    thread.start()

    # Fetch world
    world: World = fetch_world_from_service(rclpy_node)

    # Synchronizers
    model_sync = ModelSynchronizer(world=world, node=rclpy_node)
    state_sync = StateSynchronizer(world=world, node=rclpy_node)

    # Optional TF publisher
    # TFPublisher(world=world, node=rclpy_node)

    # env_world = load_environment()
    # with world.modify_world():
    #     world.merge_world(env_world)

    # Visualization
    # VizMarkerPublisher(world=world, node=rclpy_node)

    # Robot semantic view
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]

    # Context
    context = Context(
        world,
        robot_view,
        ros_node=rclpy_node,
    )

    return rclpy_node, world, robot_view, context
