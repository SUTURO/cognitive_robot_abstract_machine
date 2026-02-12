import time
from time import sleep

import rclpy

from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import (
    TorsoState,
    Arms,
    VerticalAlignment,
    ApproachDirection,
)
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import (
    MoveTorsoActionDescription,
    ParkArmsActionDescription,
    PickUpActionDescription,
    TransportActionDescription,
    NavigateActionDescription,
)
from simulation_setup import setup_hsrb_in_environment

rclpy.init()

result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
world, robot_view, context, viz = (
    result.world,
    result.robot_view,
    result.context,
    result.viz,
)

milk = world.get_body_by_name("milk.stl")
manipulator = robot_view.arms[0].manipulator
print(manipulator.tool_frame)
grasp = GraspDescription(
    ApproachDirection.FRONT, VerticalAlignment.NoAlignment, manipulator=manipulator
)

# nav = NavigateActionDescription(PoseStamped.from_matrix(
#     [robot_view.root.global_pose.x.to_list()[0],robot_view.root.global_pose.y.to_list()[0]+20,robot_view.root.global_pose.z.to_list()[0]]
#     ))

plan = SequentialPlan(
    context,
    # ParkArmsActionDescription(Arms.BOTH),
    # MoveTorsoActionDescription(TorsoState.HIGH),
    PickUpActionDescription(
        object_designator=milk, arm=Arms.LEFT, grasp_description=grasp
    ),
    # TransportActionDescription(object_designator=milk, target_location=PoseStamped.from_list([4.9, 3.3, 0.8]),
    #                          arm=Arms.LEFT),
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
    MoveTorsoActionDescription(TorsoState.LOW),
    MoveTorsoActionDescription(TorsoState.HIGH),
)

with simulated_robot:
    print(plan.current_plan)

    plan.perform()
