"""Small helper script to inspect what RoboKudo returns for the waving-human query.

Usage (in a ROS2 environment where RoboKudo is running):

- One-shot:
    python3 test_waving.py

- Retry for up to 30s (prints first successful pose):
    python3 test_waving.py --retry --timeout 30

- Retry without head scan, poll every 0.5s:
    python3 test_waving.py --retry --poll 0.5 --no-scan

Notes:
- This script prints the raw PoseStamped fields (frame_id, position, orientation).
- If the query returns None, RoboKudo didn't provide a pose result yet.
"""

from __future__ import annotations

import argparse
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from pycram.external_interfaces import robokudo


def _sleep_ros(node, seconds: float) -> None:
    """ROS2-friendly sleep that keeps callbacks/executors responsive."""
    rclpy.spin_once(node, timeout_sec=float(seconds))


def _format_pose(pose) -> str:
    # pose is expected to be pycram.datastructures.pose.PoseStamped
    try:
        frame = getattr(pose.header, "frame_id", None)
        pos = pose.position
        ori = pose.orientation
        return (
            f"PoseStamped(\n"
            f"  frame_id={frame!r},\n"
            f"  position=[{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}],\n"
            f"  orientation=[{ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}, {ori.w:.4f}]\n"
            f")"
        )
    except Exception as e:
        return f"<could not format pose: {e!r}>\nraw={pose!r}"


def query_once(node) -> Optional[object]:
    """Run a single waving-human query and print the result."""
    print("Querying RoboKudo for waving human…")
    try:
        res = robokudo.query_waving_human()
    except Exception as e:
        print(f"Query raised exception: {e!r}")
        return None

    if res is None:
        print("Result: None (no waving human detected / no pose returned)")
        return None

    print("Result: Pose found")
    print(_format_pose(res))
    return res


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--retry", action="store_true", help="keep querying until found or timeout"
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0, help="retry timeout in seconds"
    )
    parser.add_argument(
        "--poll", type=float, default=1.0, help="poll interval in seconds"
    )
    parser.add_argument(
        "--no-scan",
        action="store_true",
        help="do not move head; only query (head scan isn't needed for raw outcome inspection)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("test_waving_query")

    # Keep executor alive so action clients / callbacks can progress.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Optionally do a tiny head-pan movement sequence. This matches the observation from restaurant demos
    # that some perception pipelines behave better when head joints are commanded.
    MoveJointsMotion = None
    if not args.no_scan:
        try:
            from pycram.robot_plans import MoveJointsMotion as _MoveJointsMotion

            MoveJointsMotion = _MoveJointsMotion
        except Exception:
            MoveJointsMotion = None

    try:
        if not args.retry:
            query_once(node)
            return 0

        deadline = time.monotonic() + float(args.timeout)
        head_positions = [-0.5, 0.0, 0.5, 1.0, 0.5, 0.0]
        idx = 0

        print(f"Retry mode: timeout={args.timeout:.1f}s poll={args.poll:.2f}s")
        while rclpy.ok() and time.monotonic() < deadline:
            if MoveJointsMotion is not None:
                try:
                    MoveJointsMotion(
                        ["head_pan_joint"], [head_positions[idx % len(head_positions)]]
                    ).perform()
                    idx += 1
                except Exception as e:
                    print(f"Head motion failed (continuing): {e!r}")

            res = query_once(node)
            if res is not None:
                return 0

            _sleep_ros(node, float(args.poll))

        print("Timeout reached without a pose.")
        return 2
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
