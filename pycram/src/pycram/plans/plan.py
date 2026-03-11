from __future__ import annotations

import logging
from copy import deepcopy
from dataclasses import field, dataclass
from enum import IntEnum
from itertools import chain

import numpy as np
import rustworkx as rx
import rustworkx.visualization
from typing_extensions import (
    Optional,
    Any,
    Dict,
    List,
    Iterable,
    TYPE_CHECKING,
    Type,
    Tuple,
    TypeVar,
)


from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.world import World

if TYPE_CHECKING:
    from pycram.plans.plan_callbacks import PlanCallback
    from pycram.datastructures.dataclasses import Context
    from pycram.plans.plan_node import (
        PlanNode,
        ActionNode,
    )


logger = logging.getLogger(__name__)


class PlotAlignment(IntEnum):
    HORIZONTAL = 0
    VERTICAL = 1


T = TypeVar("T")


@dataclass
class Plan:
    """
    Represents a plan structure, typically a tree, which can be changed at any point in time. Performing the plan will
    traverse the plan structure in depth first order and perform each PlanNode
    """

    context: Context
    """
    The context where the plan can extract information from.
    """

    initial_world: Optional[World] = field(repr=False, kw_only=True, default=None)
    """
    A deepcopy of the world before the first perform was called.
    """

    node_callbacks: List[PlanCallback] = field(default_factory=list)
    """
    A list of callbacks that are called when a node is started or ended.
    """

    plan_graph: rx.PyDiGraph[PlanNode] = field(
        default_factory=rx.PyDiGraph, init=False, repr=False
    )
    """
    A directed graph representation of the plan structure.
    """

    def __post_init__(self):
        self.add_plan_entity(self.context)

    @property
    def root(self):
        [result] = [node for node in self.all_nodes if node.parent is None]
        return result

    @property
    def world(self) -> World:
        return self.context.world

    @property
    def robot(self) -> AbstractRobot:
        return self.context.robot

    @property
    def nodes(self) -> List[PlanNode]:
        """
        All nodes of the plan in depth first order.

        .. info::
            This will only return nodes that have a path from the root node. Nodes that are part of the plan but do not
            have a path from the root node will not be returned. In that case use all_nodes

        :return: All nodes under the root node in depth first order
        """
        return [self.root] + self.root.recursive_children

    @property
    def all_nodes(self) -> List[PlanNode]:
        """
        All nodes that are part of this plan
        """
        return self.plan_graph.nodes()

    @property
    def edges(self):
        return self.plan_graph.edges()

    def add_plan_entity(self, entity: PlanEntity):
        entity.plan = self

    def mount(self, other: Plan, mount_node: PlanNode = None):
        """
        Mounts another plan to this plan. The other plan will be added as a child of the mount_node.

        :param other: The plan to be mounted
        :param mount_node: A node of this plan to which the other plan will be mounted. If None, the root of this plan will be used.
        """
        mount_node = mount_node or self.root
        self.add_edge(mount_node, other.root)
        self.add_edges_from(other.edges)
        for node in self.nodes:
            node.execute = self
            node.world = self.world

    def merge_nodes(self, node1: PlanNode, node2: PlanNode):
        """
        Merges two nodes into one. The node2 will be removed and all its children will be added to node1.

        :param node1: Node which will remain in the plan
        :param node2: Node which will be removed from the plan
        """
        for node in node2.children:
            self.add_edge(node1, node)
        self.remove_node(node2)

    def remove_node(self, node_for_removal: PlanNode):
        """
        Removes a node from the plan. If the node is not in the plan, it will be ignored.

        :param node_for_removal: Node to be removed
        """
        if node_for_removal in self.nodes:
            self.plan_graph.remove_node(node_for_removal.index)
            node_for_removal.index = -1
            node_for_removal.plan = None
            node_for_removal.world = None

    def add_node(self, node: PlanNode):
        """
        Adds a node to the plan. The node will not be connected to any other node of the plan.

        :param node: Node to be added
        """
        if node.plan is self:
            return
        self.add_plan_entity(node)
        node.index = self.plan_graph.add_node(node)

    def add_edge(self, source: PlanNode, target: PlanNode, **attr):
        """
        Adds an edge to the plan. If one or both nodes are not in the plan, they will be added to the plan.

        :param source: Origin node of the edge
        :param target: Target node of the edge
        """
        self.add_node(source)
        self.add_node(target)
        # self._set_layer_indices(source, target)

        self.plan_graph.add_edge(
            source.index,
            target.index,
            (source, target),
        )

    def _set_layer_indices(
        self,
        parent_node: PlanNode,
        child_node: PlanNode,
        node_to_insert_after: PlanNode = None,
        node_to_insert_before: PlanNode = None,
    ):
        """
        Shifts the layer indices of nodes in the layer such that the index for the child node is free and does not collide
        with another index.
        If a node_to_insert_after is given the index of all nodes after the given node will be shifted by one.
        if an node_to_insert_before is given the index of all nodes after the given node will be shifted by one plus the
        index of the node_to_insert_before.
        If none is given the child node will be inserted after the last child of the parent node and all indices will
        be shifter accordingly.

        :param parent_node: The parent node under which the new node will be inserted.
        :param child_node: The node that will be inserted.
        :param node_to_insert_after: The node after which the new node will be inserted.
        :param node_to_insert_before: The node before which the new node will be inserted.
        """
        if node_to_insert_after:
            child_node.layer_index = node_to_insert_after.layer_index + 1
            for node in self.get_following_nodes(node_to_insert_after, on_layer=True):
                node.layer_index += 1
        elif node_to_insert_before:
            child_node.layer_index = node_to_insert_before.layer_index
            for node in self.get_following_nodes(
                node_to_insert_before, on_layer=True
            ) + [node_to_insert_before]:
                node.layer_index += 1
        else:
            new_position, nodes_to_shift = self._find_nodes_to_shift_index(parent_node)
            child_node.layer_index = new_position
            for node in nodes_to_shift:
                node.layer_index += 1

    def _find_nodes_to_shift_index(
        self, parent_node: PlanNode
    ) -> Tuple[int, List[PlanNode]]:

        parent_prev_nodes = self.get_previous_nodes(parent_node, on_layer=True)
        parent_follow_nodes = self.get_following_nodes(parent_node, on_layer=True)

        prev_nodes_child_layer = (
            list(chain(*[p.children for p in parent_prev_nodes])) + parent_node.children
        )
        follow_nodes_child_layer = list(
            chain(*[p.children for p in parent_follow_nodes])
        )

        return (
            max([n.layer_index for n in prev_nodes_child_layer] + [-1]) + 1,
            follow_nodes_child_layer,
        )

    def add_edges_from(
        self,
        edges: Iterable[Tuple[PlanNode, PlanNode]],
    ):
        """
        Adds edges to the plan from an iterable of tuples. If one or both nodes are not in the plan, they will be added to the plan.

        :param edges: Iterable of tuples of nodes to be added
        """
        for u, v in edges:
            self.add_edge(u, v)

    def add_nodes_from(self, nodes_for_adding: Iterable[PlanNode]):
        """
        Adds nodes from an Iterable of nodes.

        :param nodes_for_adding: The iterable of nodes
        """
        for node in nodes_for_adding:
            self.add_node(node)

    def insert_below(self, insert_node: PlanNode, insert_below: PlanNode):
        """
        Inserts a node below the given node.

        :param insert_node: The node to be inserted
        :param insert_below: A node of the plan below which the given node should be added
        """
        self.add_edge(insert_below, insert_node)

    def perform(self) -> Any:
        """
        Performs the root node of this plan.

        :return: The return value of the root node
        """
        previous_plan = Plan.current_plan
        Plan.current_plan = self
        self.initial_world = deepcopy(self.context.world)
        result = self.root.perform()
        Plan.current_plan = previous_plan
        return result

    def re_perform(self):
        for child in self.root.recursive_children:
            if child.is_leaf:
                child.perform()

    @property
    def actions(self) -> List[ActionNode]:
        return [node for node in self.nodes if isinstance(node, ActionNode)]

    @property
    def layers(self) -> List[List[PlanNode]]:
        """
        Returns the nodes of the plan layer by layer starting from the root node.

        :return: A list of lists where each list represents a layer
        """
        layer = rx.layers(self.plan_graph, [self.root.index], index_output=False)
        return [sorted(l, key=lambda x: x.layer_index) for l in layer]

    def get_layer_by_node(self, node: PlanNode) -> List[PlanNode]:
        """
        Returns the layer this node is on

        :param node: The node to get layer for
        :return: The layer as a list of nodes
        """
        return [l for l in self.layers if node in l][0]

    def get_previous_nodes(
        self, node: PlanNode, on_layer: bool = False
    ) -> List[PlanNode]:
        """
        Gets the previous nodes to the given node. Previous meaning the nodes that are before the given one in
        depth first order of nodes.

        :param node: The node to get previous nodes for
        :param on_layer: Returns the previous nodes from the same layer as the given node
        :return: The previous nodes as a list of nodes
        """
        search_space = self.get_layer_by_node(node) if on_layer else self.nodes
        previous_nodes = []
        for search_node in search_space:
            if search_node == node:
                break
            previous_nodes.append(search_node)
        return previous_nodes

    def get_following_nodes(self, node: PlanNode, on_layer: bool = False):
        """
        Gets the nodes that come after the given node. Following meaning the nodes that are after the given node
        for all nodes in depth first order of nodes.

        :param node: The node to get following nodes for
        :param on_layer: Returns the following nodes from the same layer as the given node
        :return: The following nodes as a list of nodes
        """
        search_space = self.get_layer_by_node(node) if on_layer else self.nodes
        for i, search_node in enumerate(search_space):
            if search_node == node:
                return search_space[i + 1 :]
        return []

    def get_previous_node_by_type(
        self, origin_node: PlanNode, node_type: Type[T], on_layer: bool = False
    ) -> T:
        """
        Returns the Plan Node that precedes the given node on the same level

        :param origin_node: The node to be preceded, also determines the layer of the plan
        :param node_type: The type of the plan node
        :param on_layer: Whether the returned node should be on the same layer as the given one
        :return: The Plan Node that precedes the given node
        """
        search_space = self.get_previous_nodes(origin_node, on_layer)
        search_space.reverse()

        return [node for node in search_space if type(node) == node_type]

    def get_nodes_by_type(self, node_type: Type[T]) -> List[T]:
        """
        Returns a list of nodes that match the given type.

        :param node_type: The type of the node that should be returned
        :return: A list of nodes that match the given type
        """
        return [node for node in self.nodes if type(node) is node_type]

    def _migrate_nodes_from_plan(self, other: Plan) -> PlanNode:
        """
        Steal all nodes from another plan and add them to this plan.
        After this the other plan will be empty.
        :param other: The plan to steal nodes from
        """
        other_plans_edge = other.edges
        root_ref = other.root
        other.plan_graph.clear()

        for edge in other_plans_edge:
            self.add_edge(edge[0], edge[1])

        return root_ref

    # %% Plotting functions

    def bfs_layout(
        self, scale: float = 1.0, align: PlotAlignment = PlotAlignment.VERTICAL
    ) -> Dict[int, np.array]:
        """
        Generate a bfs layout for this circuit.

        :return: A dict mapping the node indices to 2d coordinates.
        """
        layers = self.layers

        pos = None
        nodes = []
        width = len(layers)
        for i, layer in enumerate(layers):
            height = len(layer)
            xs = np.repeat(i, height)
            ys = np.arange(0, height, dtype=float)
            offset = ((width - 1) / 2, (height - 1) / 2)
            layer_pos = np.column_stack([xs, ys]) - offset
            if pos is None:
                pos = layer_pos
            else:
                pos = np.concatenate([pos, layer_pos])
            nodes.extend(layer)

        # Find max length over all dimensions
        pos -= pos.mean(axis=0)
        lim = np.abs(pos).max()  # max coordinate for all axes
        # rescale to (-scale, scale) in all directions, preserves aspect
        if lim > 0:
            pos *= scale / lim

        if align == PlotAlignment.HORIZONTAL:
            pos = pos[:, ::-1]  # swap x and y coords

        pos = dict(zip([node.index for node in nodes], pos))
        return pos

    def plot_plan_structure(
        self, scale: float = 1.0, align: PlotAlignment = PlotAlignment.HORIZONTAL
    ) -> None:
        """
        Plots the kinematic structure of the world.
        The plot shows bodies as nodes and connections as edges in a directed graph.
        """
        import matplotlib.pyplot as plt

        # Create a new figure
        plt.figure(figsize=(15, 8))

        pos = self.bfs_layout(scale=scale, align=align)

        rx.visualization.mpl_draw(
            self.plan_graph, pos=pos, labels=lambda node: str(node), with_labels=True
        )

        plt.title("Plan Graph")
        plt.axis("off")  # Hide axes
        plt.gca().invert_yaxis()
        plt.gca().invert_xaxis()
        plt.show()


@dataclass
class PlanEntity:
    """
    A base class for entities that are managed by a plan.
    """

    plan: Optional[Plan] = field(kw_only=True, default=None)
