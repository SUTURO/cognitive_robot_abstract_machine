"""
driving_with_buffer.py
======================
Detects a waving human (via RoboKudo or dummy data) and drives the
simulated HSR-B robot toward that human while keeping a configurable
stand-off distance (buffer) using :func:`nav2_move.buffer_to_pose`.

Pipeline
--------
1. Set up the simulation world via ``robot_setup``.
2. Poll RoboKudo for a waving human (falls back to dummy pose).
3. Apply ``buffer_to_pose`` so the robot stops *BUFFER_M* metres away.
4. Execute ``NavigateActionDescription`` inside a ``SequentialPlan``
   using ``simulated_robot``.

Run
---
    python3 driving_with_buffer.py
"""

import logging
import time
from typing import Optional

import rclpy
from rclpy.logging import get_logger

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.nav2_move import buffer_to_pose
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import NavigateActionDescription, ParkArmsActionDescription
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)

try:
    from pycram.external_interfaces.robokudo import send_query

    _ROBOKUDO_AVAILABLE = True
except ModuleNotFoundError:
    logging.warning(
        "robokudo_msgs not found – waving-human queries will use dummy data."
    )
    send_query = None
    _ROBOKUDO_AVAILABLE = False


logger = get_logger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
BUFFER_M: float = 0.5  # stand-off distance to the human [m]
SIMULATED: bool = True

# ---------------------------------------------------------------------------
# RoboKudo helper
# ---------------------------------------------------------------------------


def _extract_pose_fields(result) -> Optional[dict]:
    """Extract raw pose numbers from a RoboKudo Query.Result."""
    try:
        ros_ps = result.res[0].pose[0]
    except (AttributeError, IndexError, TypeError):
        return None
    return {
        "frame_id": ros_ps.header.frame_id,
        "position": [
            ros_ps.pose.position.x,
            ros_ps.pose.position.y,
            ros_ps.pose.position.z,
        ],
        "orientation": [
            ros_ps.pose.orientation.x,
            ros_ps.pose.orientation.y,
            ros_ps.pose.orientation.z,
            ros_ps.pose.orientation.w,
        ],
    }


def wait_for_waving_human(
    retry_interval: float = 1.0,
    timeout: Optional[float] = None,
) -> dict:
    """Poll RoboKudo until a waving human is found.

    Falls back to a fixed dummy pose when ``robokudo_msgs`` is not installed
    or the action server is unreachable.
    """
    _DUMMY = {
        "frame_id": "map",
        "position": [3.8683114051818848, 5.459158897399902, 0.0],
        "orientation": [0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    }

    if not _ROBOKUDO_AVAILABLE:
        logger.warning("RoboKudo not available – using dummy waving-human pose.")
        return _DUMMY

    deadline = time.monotonic() + timeout if timeout is not None else None
    attempt = 0
    while True:
        attempt += 1
        logger.info(f"Waving query attempt {attempt} …")
        result = send_query(obj_type="human", attributes=["waving"])
        if result is None:
            logger.warning(
                "RoboKudo action server not available – falling back to dummy data."
            )
            return _DUMMY
        data = _extract_pose_fields(result)
        if data is not None:
            logger.info(f"Waving human found after {attempt} attempt(s).")
            return data
        logger.warning("RoboKudo result has no pose – retrying …")

        if deadline is not None and time.monotonic() >= deadline:
            logger.error("Timed out waiting for waving human.")
            return _DUMMY
        time.sleep(retry_interval)


# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

if not rclpy.ok():
    rclpy.init()

result = robot_setup(SIMULATED, with_objects=False)

hsrb_world, robot_view, context = (
    result.world,
    result.robot_view,
    result.context,
)

# ---------------------------------------------------------------------------
# Detect waving human
# ---------------------------------------------------------------------------

logger.info("Waiting for waving human …")
human_data = wait_for_waving_human(retry_interval=1.0)

pos = human_data["position"]
ori = human_data["orientation"]
frame = human_data["frame_id"]
print("\n=== Waving human detected ===")
print(f"  frame_id   : {frame}")
print(f"  position   : x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}")
print(f"  orientation: x={ori[0]:.4f}  y={ori[1]:.4f}  z={ori[2]:.4f}  w={ori[3]:.4f}")

# ---------------------------------------------------------------------------
# Apply buffer and build nav target
# ---------------------------------------------------------------------------

human_pose = PoseStamped.from_list(
    position=pos,
    orientation=ori,
    frame=hsrb_world.root,
)

buffered_ros = buffer_to_pose(human_pose.ros_message(), BUFFER_M)
nav_target = PoseStamped.from_ros_message(buffered_ros)
nav_target.frame_id = hsrb_world.root

print(
    f"\n  Nav target (with {BUFFER_M}m buffer): "
    f"x={float(nav_target.position.x):.3f}  "
    f"y={float(nav_target.position.y):.3f}"
)

# ---------------------------------------------------------------------------
# Planning & Execution
# ---------------------------------------------------------------------------

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    NavigateActionDescription(target_location=nav_target),
)

try:
    logger.info("Executing navigation plan in simulation …")
    with simulated_robot:
        plan.perform()
    logger.info("Done – robot reached the buffered nav target.")
finally:
    if rclpy.ok():
        rclpy.shutdown()
