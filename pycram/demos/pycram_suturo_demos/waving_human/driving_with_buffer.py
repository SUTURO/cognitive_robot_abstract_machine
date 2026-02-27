import os
import sys
import signal
import logging
import threading

import rclpy
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import NavigateActionDescription
from suturo_resources.suturo_map import load_environment

logging.getLogger("semantic_digital_twin.world").setLevel(logging.WARN)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Make sibling packages importable when the script is run directly.
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_DEMOS_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
_SIM_SETUP_DIR = os.path.join(_DEMOS_ROOT, "helper_methods_and_useful_classes")
_REAL_SETUP_DIR = os.path.join(_DEMOS_ROOT, "pycram_basic_hsr_demos")
for _p in (_DEMOS_ROOT, _SIM_SETUP_DIR, _REAL_SETUP_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

USE_REAL_ROBOT = False


def robot_move(target_pose: PoseStamped, context):
    """
    Sends a navigation goal to Nav2 (real robot) or executes it in simulation.
    """
    buffered_pose = nav2_move.buffer_to_pose(target_pose, 0.5)

    if USE_REAL_ROBOT:
        os.environ["ROS_PYTHON_CHECK_FIELDS"] = "1"
        logger.info(f"[REAL] Moving to {buffered_pose}")
        nav2_move.start_nav_to_pose(buffered_pose)
    else:
        logger.info(f"[SIM] Moving to {buffered_pose}")
        plan = SequentialPlan(
            context,
            NavigateActionDescription(target_location=buffered_pose),
        )
        with simulated_robot:
            plan.perform()


def main():
    if not rclpy.ok():
        rclpy.init()

    if USE_REAL_ROBOT:
        # noinspection PyUnresolvedReferences
        from start_up import setup_hsrb_context

        rclpy_node, world, robot_view, context = setup_hsrb_context()
    else:
        # noinspection PyUnresolvedReferences
        from simulation_setup import setup_hsrb_in_environment

        result = setup_hsrb_in_environment(
            load_environment=load_environment, with_viz=True
        )
        world, robot_view, context = result.world, result.robot_view, result.context

    cabinet = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
        frame=world.root,
    )
    popcorn_table = PoseStamped.from_list(
        position=[1.3, 5.3, 0.0],
        orientation=[0.0, 0.0, 0.72, 0.64],
        frame=world.root,
    )

    robot_move(cabinet, context)
    logger.info(
        "Navigation done – RViz will keep showing the result. Press Ctrl+C or stop the process to exit."
    )

    # Block the main thread cheaply until killed (SIGTERM from IDE or Ctrl+C).
    _stop = threading.Event()

    def _set_stop(signum, frame):  # noqa: ANN001
        _stop.set()

    signal.signal(signal.SIGTERM, _set_stop)
    signal.signal(signal.SIGINT, _set_stop)
    _stop.wait()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
