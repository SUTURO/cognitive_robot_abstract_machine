from __future__ import annotations

import abc
import logging
from abc import ABC
from dataclasses import dataclass, field

from typing_extensions import Any, Optional, TYPE_CHECKING

from pycram.failures import PlanFailure
from semantic_digital_twin.world import World

if TYPE_CHECKING:
    from pycram.plans.plan_node import ActionNode, PlanNode
    from pycram.plans.plan import Plan

logger = logging.getLogger(__name__)


@dataclass
class ActionDescription(ABC):
    """
    Abstract base class for all actions.
    Actions are like builders for plans.
    An action has a set of parameters (its fields) from which it builds a symbolic plan and hence can be viewed as
    an easy abstraction of concrete low-level behavior that makes sense in certain contexts.
    """

    action_node: Optional[ActionNode] = field(default=None, kw_only=True)
    """
    The action node in the plan that executes this action.
    """

    @property
    def plan(self) -> Optional[Plan]:
        return self.action_node.plan if self.action_node else None

    @property
    def world(self) -> Optional[World]:
        return self.plan.world if self.plan else None

    def perform(self) -> Any:
        """
        Perform the entire action including precondition and postcondition validation.
        """
        logger.info(f"Performing action {self.__class__.__name__}")

        self.validate_precondition()

        result = None
        try:
            result = self.execute()
        except PlanFailure as e:
            raise e
        finally:
            self.validate_postcondition(result)

        return result

    @abc.abstractmethod
    def execute(self) -> Any:
        """
        Create the symbolic plan for this action.
        This method should only use Motions or Actions and mount them under itself, such that the plan can manage the
        entire execution.
        """
        pass

    def validate_precondition(self):
        """
        Symbolic/world state precondition validation.
        """
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        """
        Symbolic/world state postcondition validation.
        """
        pass

    def add_subplan(self, plan: PlanNode) -> PlanNode:
        subplan_root = self.plan._migrate_nodes_from_plan(plan)
        self.plan.add_edge(self.action_node, subplan_root)
