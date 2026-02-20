import logging

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription
from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from semantic_digital_twin.adapters.ros import (
    HomogeneousTransformationMatrixToRos2Converter,
)

logger = logging.getLogger(__name__)

rclpy_node, world, robot_view, context = setup_hsrb_context()

plan = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))
print(
    HomogeneousTransformationMatrixToRos2Converter.convert(robot_view.root.global_pose)
)
print(PoseStamped.from_spatial_type(robot_view.root.global_pose))

with real_robot:
    plan.perform()
