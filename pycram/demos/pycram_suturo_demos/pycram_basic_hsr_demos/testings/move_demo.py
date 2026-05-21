import logging
from enum import Enum

import semantic_digital_twin
from pycram import alternative_motion_mappings
from pycram.alternative_motion_mappings.hsrb_motion_mapping import HSRBMoveMotion
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.robot_plans import (
    NavigateActionDescription,
    ParkArmsActionDescription,
    LookAtActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.setup import setup_context
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)
logger = logging.getLogger(__name__)

context, execution_type = setup_context(simulated=False)

# Making sure alternative mapping is imported
alternative_motion_mappings = alternative_motion_mappings

hsrb_move = HSRBMoveMotion

_TABLE_POSE = PoseStamped.from_list(
    [
        1.0,
        5.1,
        0.0,
    ],
    [0.0, 0.0, 0.728, 0.684],
    frame=context.world.root,
)


class Direction(Enum):
    LEFT = [0.1, 1, 1]
    RIGHT = [0.1, -1, 1]
    BACK = [-1, 0, 1]
    FRONT = [1, 0, 1]
    FRONT_SOFA = [1, 0, 0.65]
    FRONT_DOWN = [1, 0, 0.5]


def look_in_direction(direction: Direction):
    look_at_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=direction.value[0],
        y=direction.value[1],
        z=direction.value[2],
        reference_frame=context.robot.root,
    )
    look_at_pose_in_map = context.world.transform(look_at_pose, context.world.root)
    SequentialPlan(
        context,
        LookAtActionDescription([look_at_pose_in_map.to_pose()]),
    ).perform()


with execution_type:
    SequentialPlan(
        context,
        # ParkArmsActionDescription(Arms.BOTH),
        NavigateActionDescription(target_location=_TABLE_POSE),
    ).perform()

    # look_in_direction(Direction.LEFT)
    # look_in_direction(Direction.RIGHT)
    # look_in_direction(Direction.FRONT)
