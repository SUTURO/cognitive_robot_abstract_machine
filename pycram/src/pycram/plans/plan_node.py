from __future__ import annotations

import logging
from abc import abstractmethod, ABC
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, Any, List, Type, TYPE_CHECKING

import rustworkx as rx

from giskardpy.motion_statechart.graph_node import Task
from krrood.entity_query_language.query.match import Match

from pycram.datastructures.enums import TaskStatus
from pycram.datastructures.pose import PoseStamped
from pycram.failures import PlanFailure
from pycram.motion_executor import MotionExecutor

from pycram.robot_plans import ActionDescription, BaseMotion
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.world_modification import (
    WorldModelModificationBlock,
)
from pycram.plans.plan import PlanEntity
from pycram.datastructures.execution_data import ExecutionData

if TYPE_CHECKING:
    from pycram.plans.plan import Plan

logger = logging.getLogger(__name__)


@dataclass(eq=False)
class PlanNode(PlanEntity):
    """
    A node in the plan.
    """

    status: TaskStatus = TaskStatus.CREATED
    """
    The status of the node from the TaskStatus enum.
    """

    start_time: Optional[datetime] = field(default_factory=datetime.now)
    """
    The starting time of the function, optional
    """

    end_time: Optional[datetime] = None
    """
    The ending time of the function, optional
    """

    reason: Optional[PlanFailure] = None
    """
    The reason of failure if the action failed.
    """

    result: Optional[Any] = None
    """
    Result from the execution of this node
    """

    index: Optional[int] = field(default=None, init=False, repr=False)
    """
    The index of this node in `self.plan.plan_graph`.
    """

    layer_index: int = field(default=0, init=False, repr=False)
    """
    The position of this node in the plan graph, as tuple of layer and index in layer
    """

    @property
    def parent(self) -> Optional[PlanNode]:
        """
        The parent node of this node, None if this is the root node

        :return: The parent node
        """
        return (
            self.plan.plan_graph.predecessors(self.index)[0]
            if self.plan.plan_graph.predecessors(self.index)
            else None
        )

    @property
    def children(self) -> List[PlanNode]:
        """
        All children nodes of this node

        :return:  A list of child nodes
        """
        children = self.plan.plan_graph.successors(self.index)
        return sorted(children, key=lambda node: node.layer_index)

    @property
    def recursive_children(self) -> List[PlanNode]:
        """
        Recursively lists all children and their children.

        :return: A list of all nodes below this node
        """
        rec_children = []
        for child in self.children:
            rec_children.append(child)
            rec_children.extend(child.recursive_children)

        return rec_children

    @property
    def subtree(self) -> Plan:
        """
        Creates a new plan with this node as the new root

        :return: A new plan
        """
        graph = self.plan.plan_graph.subgraph(
            [self.index] + [child.index for child in self.recursive_children]
        )
        plan = Plan(root=self, context=self.plan.context)
        plan.plan_graph = graph
        return plan

    @property
    def all_parents(self) -> List[PlanNode]:
        """
        Returns all nodes above this node until the root node. The order is from this node to the root node.

        :return: A list of all nodes above this
        """

        paths = rx.all_shortest_paths(
            self.plan.plan_graph, self.index, self.plan.root.index, as_undirected=True
        )
        return [self.plan.plan_graph[i] for i in paths[0][1:]] if len(paths) > 0 else []

    @property
    def is_leaf(self) -> bool:
        """
        Returns True if this node is a leaf node

        :return: True if this node is a leaf node
        """
        return self.children == []

    @property
    def layer(self) -> List[PlanNode]:
        return self.plan.get_layer_by_node(self)

    @property
    def left_neighbour(self) -> Optional[PlanNode]:
        left_node = [
            node
            for node in self.layer
            if node.layer_index[1] == self.layer_index[1] - 1
        ]
        return left_node[0] if left_node else None

    @property
    def right_neighbour(self) -> Optional[PlanNode]:
        right_node = [
            node
            for node in self.layer
            if node.layer_index[1] == self.layer_index[1] + 1
        ]
        return right_node[0] if right_node else None

    def __hash__(self):
        return id(self)

    def __repr__(self, *args, **kwargs):
        return f"{type(self)}"

    def interrupt(self):
        """
        Interrupts the execution of this node and all nodes below
        """
        self.status = TaskStatus.INTERRUPTED
        logger.info(f"Interrupted node: {str(self)}")
        # TODO: cancel giskard execution

    def resume(self):
        """
        Resumes the execution of this node and all nodes below
        """
        self.status = TaskStatus.RUNNING

    def pause(self):
        """
        Suspends the execution of this node and all nodes below.
        """
        self.status = TaskStatus.SLEEPING

    @abstractmethod
    def perform(self):
        """
        Perform the node.
        """

    def mount_at_index(self, child: PlanNode, index: int = -1):
        """
        Mount a plan node and all its children at a specific index.

        :param child:
        :param index:
        :return:
        """


@dataclass(eq=False)
class UnderspecifiedActionNode(PlanNode):
    """
    An action that is described by an `underspecified(...)` statement.
    This node is used to generate fully specified actions.
    """

    underspecified_action: Match = field(kw_only=True)

    @property
    def designator_type(self) -> Type:
        return self.underspecified_action.type

    def perform(self, *args, **kwargs):
        raise NotImplemented


@dataclass(eq=False)
class ActionNode(PlanNode):
    """
    A node representing a fully specified action.
    """

    action: ActionDescription = field(kw_only=True)
    """
    The fully specified action that will be executed.
    """

    execution_data: ExecutionData = None
    """
    Additional data that  is collected before and after the execution of the action.
    """

    motion_executor: MotionExecutor = None
    """
    Instance of the MotionExecutor used to execute the motion chart of the sub-motions of this action.
    """

    _world_modification_block_length_pre_perform: Optional[int] = None
    """
    The last model modification block before the execution of this node. 
    Used to check if the model has changed during execution.
    """

    def collect_motions(self) -> List[Task]:
        """
        Collects all child motions of this action. A motion is considered if it is a direct child of this action node,
        i.e. there is no other action node between this action node and the motion.
        """
        motion_desigs = list(
            filter(
                lambda x: x.is_leaf and x.parent_action_node == self,
                self.recursive_children,
            )
        )
        return [m.designator_ref.motion_chart for m in motion_desigs]

    def construct_motion_state_chart(self):
        """
        Builds a giskard Motion State Chart from the collected motions of this action node.
        """
        self.motion_executor = MotionExecutor(
            self.collect_motions(), self.plan.world, ros_node=self.plan.context.ros_node
        )
        self.motion_executor.construct_msc()

    def execute_motion_state_chart(self):
        """
        Executes the constructed Motion State Chart of this action node.
        """
        self.construct_motion_state_chart()
        self.motion_executor.execute()

    def create_execution_data_pre_perform(self):
        """
        Create the ExecutionData and logs additional information about the execution of this node.
        """
        robot_pose = PoseStamped.from_spatial_type(self.plan.robot.root.global_pose)
        exec_data = ExecutionData(robot_pose, self.plan.world.state.data)
        self.execution_data = exec_data
        self._last_world_modification_block_pre_perform_index = len(
            self.plan.world._model_manager.model_modification_blocks
        )

    def update_execution_data_post_perform(self):
        """
        Update the ExecutionData with additional information to the ExecutionData object after performing this node.
        """
        self.execution_data.execution_end_pose = PoseStamped.from_spatial_type(
            self.plan.robot.root.global_pose
        )
        self.execution_data.execution_end_world_state = self.plan.world.state.data
        self.execution_data.added_world_modifications = (
            self.plan.world._model_manager.model_modification_blocks[
                self._last_world_modification_block_pre_perform_index :
            ]
        )

    def perform(self):
        self.create_execution_data_pre_perform()

        result = self.action.perform()

        self.execute_motion_state_chart()

        self.update_execution_data_post_perform()

        return result


@dataclass(eq=False)
class MotionNode(PlanNode):
    """
    A node in the plan representing a fully specified motion
    """

    motion: BaseMotion = field(kw_only=True)

    def perform(self):
        """
        Performs this node by performing the respective MotionDesignator. Additionally, checks if one of the parents has
        the status INTERRUPTED and aborts the perform if that is the case.

        :return: The return value of the Motion Designator
        """
        return self.motion.perform()

    @property
    def parent_action_node(self):
        """
        Returns the next resolved action node in the plan above this motion node.
        """
        return list(filter(lambda x: isinstance(x, ActionNode), self.all_parents))[0]
