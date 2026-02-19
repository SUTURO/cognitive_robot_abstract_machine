import time

import rclpy
import pycram.external_interfaces.nav2_move as nav
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import Arms, VerticalAlignment, ApproachDirection
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.ros import sleep
from semantic_digital_twin.datastructures.definitions import TorsoState
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import (
    MoveTorsoActionDescription,
    ParkArmsActionDescription,
    PickUpActionDescription,
    NavigateActionDescription,
    PlaceActionDescription,
    MoveGripperMotion,
)
from simulation_setup import setup_hsrb_in_environment
from semantic_digital_twin.datastructures.definitions import GripperState

rclpy.init()
result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
world, robot_view, context, viz = (
    result.world,
    result.robot_view,
    result.context,
    result.viz,
)

milk = world.get_body_by_name("milk.stl")
grasp = GraspDescription(
    manipulator=next(iter(robot_view.manipulators)),
    approach_direction=ApproachDirection.FRONT,
    vertical_alignment=VerticalAlignment.NoAlignment,
)

# print(world.get_body_by_name("sofa_body").inertial)
print(world.bodies)

world_root = getattr(world, "root")
new_robot_pose = PoseStamped.from_list(
    [1.6, 2.0, 0], [0, 0, 0.7071068, 0.7071068], frame=world_root
)
sofa_body = getattr(world, "get_body_by_name")("sofa_body")
sofa_pose_pycram = PoseStamped.from_matrix(
    sofa_body.global_pose.to_np(), frame=world_root
)
print(sofa_pose_pycram)

place_pose = PoseStamped.from_list([1.9, 3.3, 1], [0, 0, 1, 0.1], frame=world_root)

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
    PickUpActionDescription(
        object_designator=milk, grasp_description=grasp, arm=Arms.LEFT
    ),
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.LOW),
    NavigateActionDescription(target_location=new_robot_pose, keep_joint_states=True),
    PlaceActionDescription(
        object_designator=milk,
        target_location=place_pose,
        arm=Arms.LEFT,
    ),
    # nav.start_nav_to_pose(new_robot_pose),
)

with real_robot:
    plan.perform()
