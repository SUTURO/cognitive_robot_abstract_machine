import time

import rclpy
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import Arms, VerticalAlignment, ApproachDirection
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.datastructures.definitions import TorsoState
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, ParkArmsActionDescription, PickUpActionDescription, \
    NavigateActionDescription
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
grasp = GraspDescription(manipulator=next(iter(robot_view.manipulators)),approach_direction=ApproachDirection.FRONT,
                         vertical_alignment=VerticalAlignment.NoAlignment)
new_robot_pose = PoseStamped.from_list([1,1,1],[1,1,1,1])

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
    PickUpActionDescription(object_designator=milk,grasp_description=grasp,arm=Arms.LEFT),
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.LOW),
    # NavigateActionDescription(new_robot_pose, keep_joint_states=True),
)

with simulated_robot:
    plan.perform()