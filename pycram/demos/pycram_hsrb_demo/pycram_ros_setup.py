import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)


def setup_ros_node():
    rclpy.init()
    node = rclpy.create_node("pycram_node")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Start executor in a separate thread
    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)

    hsrb_world = fetch_world_from_service(node)
    model_sync = ModelSynchronizer(world=hsrb_world, node=node)
    state_sync = StateSynchronizer(world=hsrb_world, node=node)

    return node, hsrb_world
