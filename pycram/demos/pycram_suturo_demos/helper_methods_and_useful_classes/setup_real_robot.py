import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor

from semantic_digital_twin.adapters.ros.messages import LoadModel
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
    ModelReloadSynchronizer,
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

    # This loads toya from the database - it is needed if u do not want to restart giskard constantly
    mrs = ModelReloadSynchronizer(node=node, world=None, session=None)
    message = LoadModel(primary_key=1, meta_data=mrs.meta_data)
    mrs.publish(message)

    hsrb_world = fetch_world_from_service(node)
    model_sync = ModelSynchronizer(world=hsrb_world, node=node)
    state_sync = StateSynchronizer(world=hsrb_world, node=node)

    return node, hsrb_world
