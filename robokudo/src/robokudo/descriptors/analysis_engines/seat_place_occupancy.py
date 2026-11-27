"""
Analysis engine answering queries for the seat places of the shared world in view of the
HSR head camera and whether a sitting human occupies them.

Unlike :mod:`~robokudo.descriptors.analysis_engines.free_seat_detection`, it detects no
chairs, so it reports only seat places the world already holds, such as those of a sofa.
"""

# %% imports

from __future__ import annotations

from typing_extensions import TYPE_CHECKING

from robokudo.descriptors.analysis_engines import free_seat_detection

if TYPE_CHECKING:
    from py_trees.behaviour import Behaviour

# %% analysis engine


class AnalysisEngine(free_seat_detection.AnalysisEngine):
    """
    The free seat detection without chair detection.
    """

    def name(self) -> str:
        """
        :return: The name identifier of this analysis engine
        """
        return "seat_place_occupancy"

    def chair_detection(self) -> list[Behaviour]:
        """
        :return: No behaviours, as no chairs are detected.
        """
        return []
