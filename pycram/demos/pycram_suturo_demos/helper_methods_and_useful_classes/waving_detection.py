# Tell_waving_person_where_to_sit.py
import logging
import time
from typing import Optional

import semantic_digital_twin
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces.robokudo import query_waving_human

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


class ContinuousWavingDetector:
    """A detector that continuously queries for a waving human until one is found or a timeout occurs."""

    def __init__(
        self,
        retry_interval: float = 1.0,
    ) -> None:
        """Initializes the ContinuousWavingDetector.

        Args:
            retry_interval (float): The interval in seconds between attempts to find a waving human.
        """
        self.retry_interval = retry_interval

    def wait_for_waving_human(
        self,
        timeout: Optional[float] = None,
    ) -> Optional[PoseStamped]:
        """Waits for a waving human to be detected.

        This method repeatedly queries for a waving human at a fixed interval
        until one is found or the optional timeout is reached.

        Args:
            timeout (Optional[float]): The maximum time in seconds to wait for a waving human.
                If None, it will wait indefinitely.

        Returns:
            Optional[PoseStamped]: The pose of the waving human if found, otherwise None.
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
