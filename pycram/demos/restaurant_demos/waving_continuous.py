import os
import sys
import time
import logging
from typing import Optional

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.robokudo import query_waving_human

try:
    from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
except ImportError:
    _here = os.path.abspath(os.path.dirname(__file__))
    _demos_root = os.path.abspath(os.path.join(_here, ".."))
    if _demos_root not in sys.path:
        sys.path.insert(0, _demos_root)
    from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context

logger = logging.getLogger(__name__)

rclpy_node, world, robot_view, context = setup_hsrb_context()


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
    human_pose = detector.wait_for_waving_human()

    if human_pose is None:
        logger.error("No waving human detected – aborting")
        return

    print("=== Waving human detected ===")
    print(f"  frame_id  : {human_pose.header.frame_id}")
    print(
        f"  position  : x={human_pose.position.x:.4f}  y={human_pose.position.y:.4f}  z={human_pose.position.z:.4f}"
    )
    print(
        f"  orientation: x={human_pose.orientation.x:.4f}  y={human_pose.orientation.y:.4f}"
        f"  z={human_pose.orientation.z:.4f}  w={human_pose.orientation.w:.4f}"
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
