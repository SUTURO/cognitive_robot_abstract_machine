"""
The errors of the kitchen to sofa demo.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from pathlib import Path

from krrood.exceptions import DataclassException
from krrood_spatial_math.spatial_types import Pose

# %% replaying a recording


@dataclass
class RecordingNotPlaying(DataclassException):
    """
    A recording of the HSR's head camera did not deliver its images in time.
    """

    recording: Path
    """
    The folder of the recording that was replayed.
    """

    timeout: timedelta
    """
    How long the images were waited for.
    """

    def error_message(self) -> str:
        return (
            f"The recording {self.recording} delivered no camera images within "
            f"{self.timeout.total_seconds()} s."
        )

    def suggest_correction(self) -> str:
        return (
            "Check that the folder is a ros2 bag recording of the HSR's head camera and "
            "that `ros2 bag play` runs with ROS sourced."
        )


# %% driving with Pumas


@dataclass
class NavigationGoalNotHeard(DataclassException):
    """
    Nothing listens for navigation goals, so the Pumas navigation is not running.
    """

    topic: str
    """
    The topic the goal would have been published on.
    """

    def error_message(self) -> str:
        return f"Nothing subscribes to {self.topic}."

    def suggest_correction(self) -> str:
        return (
            "Start the Pumas navigation (~/start_pumas_live.sh) and run the demo in the "
            "environment of ~/ros_clean_env.sh, so both use the same RMW."
        )


@dataclass
class NavigationTimedOut(DataclassException):
    """
    The base did not reach the goal it was sent to in time.
    """

    goal: Pose
    """
    Where the base was sent to, in the map frame.
    """

    timeout: timedelta
    """
    How long the base was given.
    """

    def error_message(self) -> str:
        return (
            f"The base did not reach ({float(self.goal.x):.2f}, {float(self.goal.y):.2f}) within "
            f"{self.timeout.total_seconds()} s."
        )

    def suggest_correction(self) -> str:
        return (
            "Check that the path is free (the safety limiter stops for anything closer "
            "than 0.6 m), that AMCL is localized and that the robot clock is synced."
        )
