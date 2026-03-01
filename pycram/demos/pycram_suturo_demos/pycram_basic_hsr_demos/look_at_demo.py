import logging

import semantic_digital_twin
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    LookAtActionDescription,
    MoveTorsoActionDescription,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from semantic_digital_twin.adapters.ros import (
    HomogeneousTransformationMatrixToRos2Converter,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.spatial_types import *

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


rclpy_node, world, robot_view, context = setup_hsrb_context()


look_at_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
    x=1, y=0, z=0.5, reference_frame=robot_view.root
)
look_at_pose_in_map = world.transform(look_at_pose, world.root)
plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
    LookAtActionDescription([look_at_pose_in_map]),
)
print(look_at_pose_in_map.to_pose())

with real_robot:
    plan.perform()
