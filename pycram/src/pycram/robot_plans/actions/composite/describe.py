from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
from typing import List

import numpy as np
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.semantic_annotations.semantic_annotations import Drawer
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Optional, Type, Any, Iterable

from .facing import FaceAtActionDescription
from ..core import (
    ParkArmsActionDescription,
    NavigateActionDescription,
    PickUpActionDescription,
    PlaceActionDescription,
    OpenActionDescription,
)
from ....config.action_conf import ActionConfig
from ....datastructures.enums import Arms, Grasp, VerticalAlignment
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....designators.location_designator import ProbabilisticCostmapLocation
from ....designators.object_designator import BelieveObject
from ....failures import ObjectUnfetchable, ConfigurationNotReached
from ....has_parameters import has_parameters
from ....language import SequentialPlan
from ....robot_description import RobotDescription
from ....robot_plans.actions.base import ActionDescription
from ....external_interfaces.nav2_move import start_nav_to_pose

import percieving

'''
 If there would be such class it would need parameters for:
    :param = goals: goals that need to be visited
    :param = finalGoal: end goal
'''
@has_parameters
@dataclass
class Describe(ActionDescription):
    def execute(self) -> None:
        '''
        Eine Describe action besteht aus einer Navigation und einer detail analysis,
        Perception:
            - Color
            - Pose
            - Type
            - Size
        and then based on the info build a description.
        And talk back
        '''

        raise NotImplementedError("This feature is not implemented yet")

    def getCurrentDescribtion(self):
        raise NotImplementedError("This feature is not implemented yet")

    def descriptionBuilder(self, entity)-> str:
        # Query something to perception and then see if it is sure its the percieved val.
        # build to be spoken text, thats the return

        descriptions = self.percieveResult(entity)    # Should return what can be recieved and then filter and use the most important things
        describtion = self.sentenceBuilder(descriptions)
        return describtion

    '''
    Describes a Object of class x in place y and talks back to referee.
    
        :param Object
        :param Location, where it is found
    
    '''
    def sentenceBuilder(self, describtion : Optional[str]="" )-> str:
        sentence = ""

        if describtion == "":
            sentence = "There was no description found, for the Object"
        else:
            for e in describtion:
                sentence = sentence + "The object is" + e
            sentence = sentence + "Those are the attributes of the object"
        return sentence


