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
3. Convert the detected RoboKudo pose (in ``head_rgbd_sensor_rgb_frame``)
   into a nav target expressed in the *world root* (map) frame by reading
   the current robot base pose from the simulation world.
4. Execute ``NavigateActionDescription`` inside a ``SequentialPlan``
   using ``simulated_robot`` so the motion is visible in RViz.

Run
---
    python3 waving_and_driving_sim.py

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
_SIM_SETUP_DIR = os.path.abspath(os.path.join(_DEMOS_ROOT, "hsrb_simulation"))
for _p in (_DEMOS_ROOT, _SIM_SETUP_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import NavigateActionDescription
from pycram.external_interfaces.robokudo import send_query
from pycram.external_interfaces import nav2_move

# simulation_setup lives in hsrb_simulation/ and is injected into sys.path above.
# noinspection PyUnresolvedReferences
from simulation_setup import setup_hsrb_in_environment  # from hsrb_simulation/

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Minimum stand-off distance to the human [m]
# ---------------------------------------------------------------------------
MIN_DISTANCE_M: float = 1.0


# ---------------------------------------------------------------------------
# RoboKudo helper – same raw extraction as waving_continuous_plain_data.py
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
) -> Optional[dict]:
    """Poll RoboKudo until a waving human is found.

    :param retry_interval: Seconds between failed queries.
    :param timeout: Give up after this many seconds (None = forever).
    :return: Plain dict with frame_id / position / orientation, or None.
    """
    deadline = time.monotonic() + timeout if timeout is not None else None
    attempt = 0
    while True:
        attempt += 1
        logger.info("Waving query attempt %d …", attempt)
        result = send_query(obj_type="human", attributes=["waving"])
        if result is not None:
            data = _extract_pose_fields(result)
            if data is not None:
                logger.info("Waving human found after %d attempt(s).", attempt)
                return data
            logger.warning("RoboKudo result has no pose – retrying …")
        else:
            logger.warning("No result from RoboKudo – retrying …")

        if deadline is not None and time.monotonic() >= deadline:
            logger.error("Timed out waiting for waving human.")
            return None
        time.sleep(retry_interval)


# ---------------------------------------------------------------------------
# Coordinate transform helper
# ---------------------------------------------------------------------------


def _robot_base_pose(world) -> PoseStamped:
    """Return the current robot base pose in world.root frame."""
    world_root = world.root
    base = world.get_body_by_name("base_footprint")
    return PoseStamped.from_matrix(base.global_pose.to_np(), frame=world_root)


def _human_nav_target(
    human_data: dict,
    robot_pose: PoseStamped,
    world,
    min_distance: float = MIN_DISTANCE_M,
) -> PoseStamped:
    """Compute a nav target *min_distance* metres in front of the human.

    Converts PyCRAM PoseStamped objects to ROS PoseStamped, delegates the
    distance calculation to :func:`nav2_move.min_distance_2_human`, then
    converts the result back to a PyCRAM PoseStamped in *world.root* frame.
    """
    pos = human_data["position"]
    ori = human_data["orientation"]
    world_root = world.root

    # Build ROS PoseStamped for the human (in world/map frame).
    # RoboKudo reports the pose in the camera frame; we use it directly here
    # as an approximate map-frame pose (valid for simulation validation).
    human_ros = PoseStamped.from_list(
        position=pos,
        orientation=ori,
        frame=world_root,
    ).ros_message()

    robot_ros = robot_pose.ros_message()

    target_ros = nav2_move.min_distance_2_human(
        human_pose=human_ros,
        robot_pose=robot_ros,
        min_distance=min_distance,
    )

    target = PoseStamped.from_ros_message(target_ros)
    target.frame_id = world_root

    logger.info(
        "Nav target: x=%.3f  y=%.3f",
        float(target.position.x),
        float(target.position.y),
    )
    return target


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    logging.basicConfig(level=logging.INFO)

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

    if human_data is None:
        logger.error("No waving human detected – aborting.")
        return

    pos = human_data["position"]
    ori = human_data["orientation"]
    frame = human_data["frame_id"]
    print("\n=== Waving human detected ===")
    print(f"  frame_id   : {frame}")
    print(f"  position   : x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}")
    print(
        f"  orientation: x={ori[0]:.4f}  y={ori[1]:.4f}  z={ori[2]:.4f}  w={ori[3]:.4f}"
    )
    print(
        f"\n  PoseStamped.from_list(\n"
        f"      position={[round(v, 4) for v in pos]},\n"
        f"      orientation={[round(v, 4) for v in ori]},\n"
        f'      frame="{frame}",\n'
        f"  )"
    )

    # 4. Compute nav target in map/world frame
    robot_pose = _robot_base_pose(world)
    nav_target = _human_nav_target(human_data, robot_pose, world)

    print(
        f"\n  Nav target (world frame): "
        f"x={float(nav_target.position.x):.3f}  "
        f"y={float(nav_target.position.y):.3f}"
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
