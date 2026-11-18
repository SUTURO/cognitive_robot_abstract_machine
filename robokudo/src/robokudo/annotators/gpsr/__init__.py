"""
Annotators of the General Purpose Service Robot task, and the ROS package their trained
models come from.
"""

from __future__ import annotations

# %% Imports
from enum import StrEnum

# %% ROS packages


class GpsrRosPackage(StrEnum):
    """
    ROS packages that GPSR reads files from.
    """

    TRAINED_MODELS = "robokudo_suturo"
    """
    Holds the trained detectors of GPSR and the class names they report.
    """
