import time
import logging
from typing import Optional

from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.robokudo import query_waving_human


logger = logging.getLogger(__name__)


class ContinuousWavingDetector:

    def __init__(
        self,
        retry_interval: float = 1.0,
    ) -> None:
        self.retry_interval = retry_interval

    def wait_for_waving_human(
        self,
        timeout: Optional[float] = None,
    ) -> Optional[PoseStamped]:
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
    from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
        robot_setup,
    )

    setup_result = robot_setup()
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
