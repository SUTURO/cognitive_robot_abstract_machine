import logging
import time
from typing import Optional

import rclpy

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.nav2_move import min_distance_2_human
from pycram.external_interfaces.robokudo import query_waving_human
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, simulated_robot
from pycram.robot_plans import ParkArmsActionDescription, NavigateActionDescription
from pycram.robot_plans.motions import MoveJointsMotion

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.WARN)

SIMULATED: bool = True
MIN_DISTANCE_M: float = 0.5

HEAD_PAN_JOINT = "head_pan_joint"
PAN_START = -0.5
PAN_END = 1.0
INCREMENT = 0.5


if SIMULATED:
    from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
        robot_setup,
    )

    setup_result = robot_setup(simulation=True, with_objects=False)
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )
else:
    from demos.pycram_suturo_demos.pycram_basic_hsr_demos.start_up import (
        setup_hsrb_context,
    )

    _node, world, robot_view, context = setup_hsrb_context()


if not rclpy.ok():
    rclpy.init()


def look_around(
    increase: float = INCREMENT,
    timeout: Optional[float] = None,
) -> Optional[PoseStamped]:

    deadline = time.monotonic() + timeout if timeout is not None else None

    pan = PAN_START
    direction = 1

    while True:
        logger.info("Panning head to %.2f rad", pan)
        SequentialPlan(
            context,
            MoveJointsMotion(names=[HEAD_PAN_JOINT], positions=[pan]),
        ).perform()

        h_pose: Optional[PoseStamped] = query_waving_human()

        if h_pose is not None:
            logger.info("Waving human found at pan=%.2f – %s", pan, h_pose)
            return h_pose

        logger.info("No waving human found, continuing scan …")

        if deadline is not None and time.monotonic() >= deadline:
            logger.warning("Timed out after %.1f s", timeout)
            return None

        pan += increase * direction
        if pan >= PAN_END:
            pan = PAN_END
            direction = -1
        elif pan <= PAN_START:
            pan = PAN_START
            direction = 1


def get_robot_pose() -> PoseStamped:
    return PoseStamped.from_spatial_type(robot_view.root.global_pose)


executor = simulated_robot if SIMULATED else real_robot

with executor:
    human_pose = look_around(increase=INCREMENT)

if human_pose is None:
    logger.error("No waving human detected – aborting.")
    raise SystemExit(1)

logger.info("Waving human detected at %s", human_pose)

robot_pose = get_robot_pose()

human_pose_clean = PoseStamped.from_list(
    position=human_pose.position.to_list(),
    orientation=human_pose.orientation.to_list(),
    frame=world.root,
)

target_ros = min_distance_2_human(
    human_pose=human_pose_clean.ros_message(),
    robot_pose=robot_pose.ros_message(),
    min_distance=MIN_DISTANCE_M,
)

nav_target = PoseStamped.from_list(
    position=[
        target_ros.pose.position.x,
        target_ros.pose.position.y,
        target_ros.pose.position.z,
    ],
    orientation=[
        target_ros.pose.orientation.x,
        target_ros.pose.orientation.y,
        target_ros.pose.orientation.z,
        target_ros.pose.orientation.w,
    ],
    frame=world.root,
)

logger.info(
    "Nav target: x=%.3f  y=%.3f",
    float(nav_target.position.x),
    float(nav_target.position.y),
)

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    NavigateActionDescription(target_location=nav_target),
)

with executor:
    plan.perform()
