"""
waving_continuous_plain_data.py
================================
Continuously queries RoboKudo for a waving human and prints the detected
pose data as a PoseStamped.from_list(…) construct.

NO nav2 / move commands are used – this script only validates that the
pose data arrives correctly from the perception pipeline.

Run with:
    python3 waving_continuous_plain_data.py
"""

import os
import sys
import time
import logging
from typing import Optional

# ---------------------------------------------------------------------------
# Make pycram_suturo_demos importable when run directly
# ---------------------------------------------------------------------------
_DEMOS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _DEMOS_ROOT not in sys.path:
    sys.path.insert(0, _DEMOS_ROOT)

from pycram.external_interfaces.robokudo import send_query

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helper: extract pose fields from a raw RoboKudo result
# ---------------------------------------------------------------------------


def _extract_pose_fields(result) -> Optional[dict]:
    """Return a plain dict with pose data, or None if unavailable."""
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


# ---------------------------------------------------------------------------
# Continuous detector
# ---------------------------------------------------------------------------


class ContinuousWavingDetectorPlain:
    """Poll RoboKudo until a waving human is detected.

    Returns a plain dict so there is no dependency on PoseStamped.__repr__
    or on a Body-typed frame_id.
    """

    def __init__(self, retry_interval: float = 1.0) -> None:
        self.retry_interval = retry_interval

    def wait_for_waving_human(self, timeout: Optional[float] = None) -> Optional[dict]:
        deadline = time.monotonic() + timeout if timeout is not None else None
        attempt = 0

        while True:
            attempt += 1
            logger.info("Attempt %d – querying RoboKudo for waving human …", attempt)

            result = send_query(obj_type="human", attributes=["waving"])
            if result is not None:
                pose_fields = _extract_pose_fields(result)
                if pose_fields is not None:
                    logger.info("Waving human found after %d attempt(s).", attempt)
                    return pose_fields
                else:
                    logger.warning("Result received but no pose data found: %s", result)
            else:
                logger.warning("No result from RoboKudo.")

            if deadline is not None and time.monotonic() >= deadline:
                logger.warning(
                    "Timed out after %.1f s (%d attempts).", timeout, attempt
                )
                return None

            logger.debug("Retrying in %.1f s …", self.retry_interval)
            time.sleep(self.retry_interval)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------


def main() -> None:
    detector = ContinuousWavingDetectorPlain(retry_interval=1.0)

    logger.info("Starting waving-human detection loop …")
    data = detector.wait_for_waving_human()

    if data is None:
        logger.error("No waving human detected – aborting.")
        return

    # ------------------------------------------------------------------
    # Build a PoseStamped.from_list(…) style construct and print it
    # ------------------------------------------------------------------
    pos = data["position"]
    ori = data["orientation"]
    frame = data["frame_id"]

    print("\n=== Waving human detected ===")
    print(f"  frame_id   : {frame}")
    print(f"  position   : x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}")
    print(
        f"  orientation: x={ori[0]:.4f}  y={ori[1]:.4f}  z={ori[2]:.4f}  w={ori[3]:.4f}"
    )

    print("\n--- as PoseStamped.from_list(…) construct ---")
    print(
        f"HUMAN_POSE = PoseStamped.from_list(\n"
        f"    position={[round(v, 4) for v in pos]},\n"
        f"    orientation={[round(v, 4) for v in ori]},\n"
        f'    frame="{frame}",\n'
        f")"
    )


if __name__ == "__main__":
    main()
