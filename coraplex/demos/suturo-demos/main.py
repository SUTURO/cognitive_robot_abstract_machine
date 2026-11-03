import rclpy

from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment

world = KitchenEnvironment().create_world()
print(world.bodies)
rclpy.init()
node = rclpy.create_node("tracy_demo_node")
VizMarkerPublisher(_world=world, node=node)
