import os
import sys
import time
import logging
from typing import Optional

# Add pycram/demos to sys.path so pycram_suturo_demos is importable
# when this script is run directly (same pattern as move_demo.py).
_DEMOS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _DEMOS_ROOT not in sys.path:
    sys.path.insert(0, _DEMOS_ROOT)

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.robokudo import query_waving_human
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)


logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = robot_setup()


class ContinuousWavingDetector:
    """Continuously queries RoboKudo for a waving human until one is found.

    Sends a ``human + waving`` query in a loop and returns the first detected
    pose. Optionally stops after a configurable timeout.

    Usage::

        detector = ContinuousWavingDetector(retry_interval=1.0)

        # Block forever until a waving human appears:
        pose = detector.wait_for_waving_human()

        # Or with a 60 s timeout:
        pose = detector.wait_for_waving_human(timeout=60.0)
        if pose is None:
            logger.error("Timed out – no waving human found")
    """

    def __init__(
        self,
        retry_interval: float = 1.0,
    ) -> None:
        """
        :param retry_interval: Seconds to wait between failed queries.
        """
        self.retry_interval = retry_interval

    def wait_for_waving_human(
        self,
        timeout: Optional[float] = None,
    ) -> Optional[PoseStamped]:
        """Block until a waving human is detected and return the pose.

        Calls :func:`pycram.external_interfaces.robokudo.query_waving_human`
        repeatedly until a valid ``PoseStamped`` is returned.

        :param timeout: Maximum time to wait in seconds.
                        ``None`` (default) waits indefinitely.
        :return: ``PoseStamped`` of the waving human in the frame reported by
                 RoboKudo, or ``None`` if *timeout* elapsed without a detection.
        """
        deadline = time.monotonic() + timeout if timeout is not None else None
        attempt = 0

        while True:
            attempt += 1
            logger.info("ContinuousWavingDetector: attempt %d …", attempt)

            pose: Optional[PoseStamped] = query_waving_human()

            if pose is not None:
                logger.info(
                    "ContinuousWavingDetector: waving human found after %d attempt(s) – pose: %s",
                    attempt,
                    pose,
                )
                return pose

            if deadline is not None and time.monotonic() >= deadline:
                logger.warning(
                    "ContinuousWavingDetector: timed out after %.1f s (%d attempts)",
                    timeout,
                    attempt,
                )
                return None

            logger.debug(
                "ContinuousWavingDetector: no waving human yet, retrying in %.1f s …",
                self.retry_interval,
            )
            time.sleep(self.retry_interval)


def main() -> None:
    detector = ContinuousWavingDetector(retry_interval=1.0)

    logger.info("Waiting for a waving human …")
    raw_pose = detector.wait_for_waving_human()

    if raw_pose is None:
        logger.error("No waving human detected – aborting")
        return

    human_pose = raw_pose

    print("=== Waving human detected ===")
    print(f"  frame_id   : {str(human_pose.header.frame_id)}")
    print(
        f"  position   : x={human_pose.position.x:.4f}  "
        f"y={human_pose.position.y:.4f}  "
        f"z={human_pose.position.z:.4f}"
    )
    print(
        f"  orientation: x={human_pose.orientation.x:.4f}  "
        f"y={human_pose.orientation.y:.4f}  "
        f"z={human_pose.orientation.z:.4f}  "
        f"w={human_pose.orientation.w:.4f}"
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
