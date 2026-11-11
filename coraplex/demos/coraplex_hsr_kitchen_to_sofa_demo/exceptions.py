"""
The errors of the kitchen to sofa demo.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from pathlib import Path

from krrood.exceptions import DataclassException

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
