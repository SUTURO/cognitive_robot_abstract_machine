"""
waving_and_driving_sim.py
==========================
Queries RoboKudo continuously until a waving human is detected, then
drives the simulated HSR-B robot toward that human and visualises the
motion in RViz via VizMarkerPublisher / TFPublisher.

Pipeline
--------
1. Boot the simulation world (HSRB URDF + suturo environment map).
2. Poll RoboKudo until a waving human is found.
3. Navigate directly to the detected human pose (no stand-off distance).
4. Execute ``NavigateActionDescription`` inside a ``SequentialPlan``
   using ``simulated_robot`` so the motion is visible in RViz.

Run
---
    python3 move_without_buffer_sim.py

Make sure RViz is open and subscribed to:
    - /visualization_marker_array  (VizMarkerPublisher)
    - /tf                          (TFPublisher)
"""

import logging
import os
import sys
import time
from typing import Optional

import rclpy
from suturo_resources.suturo_map import load_environment

# ---------------------------------------------------------------------------
# Make sibling packages importable when the script is run directly.
# ---------------------------------------------------------------------------
_WAVING_HUMAN_DIR = os.path.dirname(os.path.abspath(__file__))
_DEMOS_ROOT = os.path.abspath(os.path.join(_WAVING_HUMAN_DIR, ".."))
_SIM_SETUP_DIR = os.path.abspath(
    os.path.join(_DEMOS_ROOT, "helper_methods_and_useful_classes")
)
for _p in (_DEMOS_ROOT, _SIM_SETUP_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import NavigateActionDescription

try:
    from pycram.external_interfaces.robokudo import send_query

    _ROBOKUDO_AVAILABLE = True
except ModuleNotFoundError:
    logging.warning(
        "robokudo_msgs not found – waving-human queries will use dummy data."
    )
    send_query = None
    _ROBOKUDO_AVAILABLE = False

# noinspection PyUnresolvedReferences
from simulation_setup import setup_hsrb_in_environment


logger = logging.getLogger(__name__)


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
    retry_interval: float = 0,
    timeout: Optional[float] = None,
) -> dict:
    """Poll RoboKudo until a waving human is found.

    Falls back to dummy data immediately when the action server is unavailable
    or when robokudo_msgs is not installed.

    :param retry_interval: Seconds between failed queries.
    :param timeout: Give up after this many seconds (None = forever).
    :return: Plain dict with frame_id / position / orientation.
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
        logger.info("Waving query attempt %d …", attempt)
        result = send_query(obj_type="human", attributes=["waving"])
        if result is None:
            logger.warning(
                "RoboKudo action server not available – falling back to dummy data."
            )
            return _DUMMY
        data = _extract_pose_fields(result)
        if data is not None:
            logger.info("Waving human found after %d attempt(s).", attempt)
            return data
        logger.warning("RoboKudo result has no pose – retrying …")

        if deadline is not None and time.monotonic() >= deadline:
            logger.error("Timed out waiting for waving human.")
            return _DUMMY
        time.sleep(retry_interval)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:

    # 1. Init ROS 2
    if not rclpy.ok():
        rclpy.init()

    # 2. Build simulation world (HSRB + environment, with RViz publishers)
    logger.info("Setting up simulation world …")
    result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
    world = result.world
    context = result.context

    # 3. Detect waving human via RoboKudo
    logger.info("Waiting for waving human …")
    human_data = wait_for_waving_human(retry_interval=1.0)

    pos = human_data["position"]
    ori = human_data["orientation"]
    frame = human_data["frame_id"]
    print("\n=== Waving human detected ===")
    print(f"  frame_id   : {frame}")
    print(f"  position   : x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}")
    print(
        f"  orientation: x={ori[0]:.4f}  y={ori[1]:.4f}  z={ori[2]:.4f}  w={ori[3]:.4f}"
    )

    # 4. Navigate directly to the human pose (no stand-off distance)
    nav_target = PoseStamped.from_list(
        position=pos,
        orientation=ori,
        frame=world.root,
    )
    print(
        f"\n  Nav target: x={float(nav_target.position.x):.3f}  y={float(nav_target.position.y):.3f}"
    )

    # 5. Execute in simulation (visible in RViz)
    plan = SequentialPlan(
        context,
        NavigateActionDescription(target_location=nav_target),
    )

    logger.info("Executing navigation plan in simulation …")
    with simulated_robot:
        plan.perform()

    logger.info("Done – robot reached the nav target.")


if __name__ == "__main__":
    main()
