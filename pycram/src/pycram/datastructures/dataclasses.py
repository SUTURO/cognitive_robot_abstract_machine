from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import (
    Optional,
    Any,
)

from pycram.plans.plan import Plan, PlanEntity
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.world import World


@dataclass
class Context(PlanEntity):
    """
    A dataclass for storing the context of a plan
    """

    world: World
    """
    The world in which the plan is executed
    """

    robot: AbstractRobot
    """
    The semantic robot annotation which should execute the plan
    """

    ros_node: Optional[Any] = field(default=None)
    """
    A ROS node that should be used for communication in this plan
    """

    @classmethod
    def from_world(cls, world: World, plan: Plan = None):
        """
        Create a context from a world by getting the first robot in the world. There is no super plan in this case.

        :param world: The world for which to create the context
        :param plan: The plan that manages this context
        :return: A context with the first robot in the world and no super plan
        """
        result =  cls(
            world=world,
            robot=world.get_semantic_annotations_by_type(AbstractRobot)[0],
        )
        if plan:
            plan.add_plan_entity(result)
        return result



