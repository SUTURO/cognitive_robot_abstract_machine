from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, List

import numpy as np

from pycram.datastructures.pose import PoseStamped
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.world_modification import WorldModelModificationBlock


@dataclass
class ExecutionData:
    """
    A dataclass for storing the information of an execution that is used for creating a robot description for that
    execution. An execution is a Robot with a virtual mobile base that can be used to move the robot in the environment.
    """

    execution_start_pose: PoseStamped
    """
    Start of the robot at the start of execution of an action designator
    """

    execution_start_world_state: np.ndarray
    """
    The world state at the start of execution of an action designator
    """

    execution_end_pose: Optional[PoseStamped] = None
    """
    The pose of the robot at the end of executing an action designator
    """

    execution_end_world_state: Optional[np.ndarray] = None
    """
    The world state at the end of executing an action designator
    """

    added_world_modifications: List[WorldModelModificationBlock] = field(
        default_factory=list
    )
    """
    A list of World modification blocks that were added during the execution of the action designator
    """

