from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import timedelta

from semantic_digital_twin.world_description.world_entity import SemanticAnnotation
from typing_extensions import Union, Optional, Type, Any, Iterable

from ..core.misc import DetectActionDescription
from ..core.navigation import LookAtActionDescription, NavigateActionDescription
from ....datastructures.enums import DetectionTechnique
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....designators.location_designator import CostmapLocation
from ....failures import PerceptionObjectNotFound
from ....has_parameters import has_parameters
from ....language import TryInOrderPlan, SequentialPlan
from ....robot_plans.actions.base import ActionDescription

@has_parameters
@dataclass
class CountAction(ActionDescription):
    raise NotImplementedError("This feature is not implemented yet")

@has_parameters
@dataclass
class DescribingAction(ActionDescription):

    def queryParser(self, percievedQuery):
        # depends on data-type
        raise NotImplementedError("There hasnt been an implementation.")

    def perceiveQuery(self, query):
        # Here should be a query of perception, for what the object or person is
        raise NotImplementedError("This feature is not implemented yet")

    def filterByEntity(self, entity, unfilteredQuery : list[str] = "") -> list[str]:
        raise NotImplementedError("This feature is not implemented yet")

    def perceiveResult(self, entity) -> list[str]:
        someQuery = ""
        percievedQuery = self.perceiveQuery(someQuery)              # its just raw percieved data
        parsedQuery = self.queryParser(percievedQuery)              # Parses the query and creates a better handleable data structure
        filteredQuery = self.filterByEntity(entity, parsedQuery)    # filters the instances of the entity we are intrested in

        # Here should be the processing of the Parsed query into an arra<
        resultArray = []        # Later should be replaced with the processing or assigned properly
        return resultArray

    raise NotImplementedError("This feature is not implemented yet")

class GuidingAction(ActionDescription):

    @has_parameters
    @dataclass
    def blabla(self):
        pass