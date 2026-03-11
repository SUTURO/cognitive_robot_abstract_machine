from typing_extensions import List, assert_never, Union, Optional

from krrood.entity_query_language.query.match import Match
from pycram.datastructures.dataclasses import Context
from pycram.language import SequentialNode
from pycram.plans.plan import Plan
from pycram.plans.plan_node import (
    PlanNode,
    UnderspecifiedActionNode,
    ActionNode,
    MotionNode,
)
from pycram.robot_plans import ActionDescription, BaseMotion

ActionLike = Union[Match, ActionDescription, PlanNode, BaseMotion]


def execute_single(
    action_like: ActionLike, context: Optional[Context] = None
) -> PlanNode:
    plan = Plan(context=context)
    plan.add_node(make_node(action_like))
    return make_node(action_like)


def sequential(
    children: List[ActionLike],
    context: Optional[Context] = None,
) -> SequentialNode:
    result = SequentialNode()
    plan = Plan(context=context)
    plan.add_node(result)
    for action_like in children:
        plan.add_edge(result, make_node(action_like))
    return result


def make_node(action_like: ActionLike) -> PlanNode:

    if isinstance(action_like, PlanNode):
        return action_like
    elif isinstance(action_like, Match):
        underspecified_action = UnderspecifiedActionNode(
            underspecified_action=action_like
        )
        return underspecified_action
    elif isinstance(action_like, ActionDescription):
        return ActionNode(action=action_like)
    elif isinstance(action_like, BaseMotion):
        return MotionNode(motion=action_like)
    else:
        assert_never(action_like)
