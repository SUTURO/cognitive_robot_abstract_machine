"""
The door to sofa demo in kinematic simulation, shown in RViz.

Run it from this folder, naming the seat places a person sits on::

    python rviz_demo.py RIGHT

Watch it with the fixed frame ``root`` and a ``MarkerArray`` display on
``/semworld/viz_marker`` whose durability is transient local. The world after the run
stays shown, its state published every second so an RViz started later finds the robot,
until Ctrl+C.
"""

from __future__ import annotations

import time

import rclpy

from coraplex.plans.executors import KinematicallySimulatedPlanExecutor
from demo import main, occupied_seats_from_command_line
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)

# %% running it

if __name__ == "__main__":
    world = main(
        executor_type=KinematicallySimulatedPlanExecutor,
        occupied_seats=occupied_seats_from_command_line(),
    )
    rclpy.init()
    node = rclpy.create_node("hsr_door_to_sofa_demo_result")
    TFPublisher(node=node, _world=world)
    VizMarkerPublisher(_world=world, node=node)
    print("Shown in RViz; press Ctrl+C to quit.")
    try:
        while True:
            world.notify_state_change()
            time.sleep(1)
    except KeyboardInterrupt:
        rclpy.try_shutdown()
