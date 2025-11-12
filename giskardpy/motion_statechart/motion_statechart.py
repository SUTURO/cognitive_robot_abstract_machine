from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, InitVar
from typing import Dict, Any

import numpy as np
import rustworkx as rx
from krrood.adapters.json_serializer import SubclassJSONSerializer
from typing_extensions import List, MutableMapping, ClassVar, Self, Type, Optional

import semantic_digital_twin.spatial_types.spatial_types as cas
from giskardpy.data_types.exceptions import EmptyProblemException
from giskardpy.motion_statechart.auxilary_variable_manager import (
    AuxiliaryVariableManager,
)
from giskardpy.motion_statechart.context import BuildContext, ExecutionContext
from giskardpy.motion_statechart.data_types import (
    LifeCycleValues,
    ObservationStateValues,
)
from giskardpy.motion_statechart.graph_node import (
    MotionStatechartNode,
    TrinaryCondition,
    Goal,
    EndMotion,
    CancelMotion,
    GenericMotionStatechartNode,
    ObservationVariable,
    LifeCycleVariable,
)
from giskardpy.motion_statechart.plotters.graphviz import MotionStatechartGraphviz
from giskardpy.qp.constraint_collection import ConstraintCollection
from giskardpy.qp.qp_controller import QPController
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World


@dataclass(repr=False, eq=False)
class State(MutableMapping[MotionStatechartNode, float], SubclassJSONSerializer):
    motion_statechart: MotionStatechart
    default_value: ClassVar[float] = field(init=False)
    data: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.float64))

    def grow(self) -> None:
        self.data = np.append(self.data, self.default_value)

    def life_cycle_symbols(self) -> List[LifeCycleVariable]:
        return [node.life_cycle_variable for node in self.motion_statechart.nodes]

    def observation_symbols(self) -> List[ObservationVariable]:
        return [node.observation_variable for node in self.motion_statechart.nodes]

    def __getitem__(self, node: MotionStatechartNode) -> float:
        return float(self.data[node.index])

    def __setitem__(self, node: MotionStatechartNode, value: float) -> None:
        self.data[node.index] = value

    def __delitem__(self, node: MotionStatechartNode) -> None:
        self.data = np.delete(self.data, node.index)

    def __iter__(self) -> iter:
        return iter(self.data)

    def __len__(self) -> int:
        return self.data.shape[0]

    def keys(self) -> List[MotionStatechartNode]:
        return self.motion_statechart.nodes

    def items(self) -> List[tuple[MotionStatechartNode, float]]:
        return [(node, self[node]) for node in self.motion_statechart.nodes]

    def values(self) -> List[float]:
        return [self[node] for node in self.keys()]

    def __contains__(self, node: MotionStatechartNode) -> bool:
        return node in self.motion_statechart.nodes

    def __deepcopy__(self, memo) -> Self:
        """
        Create a deep copy of the WorldState.
        """
        return self.__class__(
            motion_statechart=self.motion_statechart,
            data=self.data.copy(),
        )

    def to_json(self) -> Dict[str, Any]:
        return {**super().to_json(), "data": self.data.tolist()}

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        motion_statechart = kwargs["motion_statechart"]
        return cls(
            motion_statechart=motion_statechart,
            data=np.array(data["data"], dtype=np.float64),
        )

    def __str__(self) -> str:
        return str({str(symbol.name): value for symbol, value in self.items()})

    def __repr__(self) -> str:
        return str(self)

    def __eq__(self, other: Self) -> bool:
        return str(self) == str(other)


@dataclass(repr=False, eq=False)
class LifeCycleState(State):

    default_value: ClassVar[float] = LifeCycleValues.NOT_STARTED
    _compiled_updater: cas.CompiledFunction = field(init=False)

    def compile(self):
        state_updater = []
        for node in self.motion_statechart.nodes:
            state_symbol = node.life_cycle_variable

            not_started_transitions = cas.if_else(
                condition=node.start_condition == cas.TrinaryTrue,
                if_result=cas.Expression(LifeCycleValues.RUNNING),
                else_result=cas.Expression(LifeCycleValues.NOT_STARTED),
            )
            running_transitions = cas.if_cases(
                cases=[
                    (
                        node.reset_condition == cas.TrinaryTrue,
                        cas.Expression(LifeCycleValues.NOT_STARTED),
                    ),
                    (
                        node.end_condition == cas.TrinaryTrue,
                        cas.Expression(LifeCycleValues.DONE),
                    ),
                    (
                        node.pause_condition == cas.TrinaryTrue,
                        cas.Expression(LifeCycleValues.PAUSED),
                    ),
                ],
                else_result=cas.Expression(LifeCycleValues.RUNNING),
            )
            pause_transitions = cas.if_cases(
                cases=[
                    (
                        node.reset_condition == cas.TrinaryTrue,
                        cas.Expression(LifeCycleValues.NOT_STARTED),
                    ),
                    (
                        node.end_condition == cas.TrinaryTrue,
                        cas.Expression(LifeCycleValues.DONE),
                    ),
                    (
                        node.pause_condition == cas.TrinaryFalse,
                        cas.Expression(LifeCycleValues.RUNNING),
                    ),
                ],
                else_result=cas.Expression(LifeCycleValues.PAUSED),
            )
            ended_transitions = cas.if_else(
                condition=node.reset_condition == cas.TrinaryTrue,
                if_result=cas.Expression(LifeCycleValues.NOT_STARTED),
                else_result=cas.Expression(LifeCycleValues.DONE),
            )

            state_machine = cas.if_eq_cases(
                a=state_symbol,
                b_result_cases=[
                    (LifeCycleValues.NOT_STARTED, not_started_transitions),
                    (LifeCycleValues.RUNNING, running_transitions),
                    (LifeCycleValues.PAUSED, pause_transitions),
                    (LifeCycleValues.DONE, ended_transitions),
                ],
                else_result=cas.Expression(state_symbol),
            )
            state_updater.append(state_machine)
        state_updater = cas.Expression(state_updater)
        self._compiled_updater = state_updater.compile(
            parameters=[self.observation_symbols(), self.life_cycle_symbols()],
            sparse=False,
        )

    def __getitem__(self, node: MotionStatechartNode) -> LifeCycleValues:
        return LifeCycleValues(super().__getitem__(node))

    def update_state(self, observation_state: np.ndarray):
        self.data = self._compiled_updater(observation_state, self.data)

    def __str__(self) -> str:
        return str(
            {
                str(symbol.name): LifeCycleValues(value).name
                for symbol, value in self.items()
            }
        )


@dataclass(repr=False, eq=False)
class ObservationState(State):
    default_value: ClassVar[ObservationStateValues] = ObservationStateValues.UNKNOWN

    _compiled_updater: cas.CompiledFunction = field(init=False)

    def compile(self):
        observation_state_updater = []
        for node in self.motion_statechart.nodes:
            state_f = cas.if_eq_cases(
                a=node.life_cycle_variable,
                b_result_cases=[
                    (
                        int(LifeCycleValues.RUNNING),
                        node._observation_expression,
                    ),
                    (
                        int(LifeCycleValues.NOT_STARTED),
                        cas.TrinaryUnknown,
                    ),
                ],
                else_result=cas.Expression(node.observation_variable),
            )
            observation_state_updater.append(state_f)
        self._compiled_updater = cas.Expression(observation_state_updater).compile(
            parameters=[
                self.observation_symbols(),
                self.life_cycle_symbols(),
                self.motion_statechart.world.state.get_variables(),
                self.motion_statechart.auxiliary_variable_manager.variables,
            ],
            sparse=False,
        )

    def update_state(
        self,
        life_cycle_state: np.ndarray,
        world_state: np.ndarray,
        auxiliar_variables: np.ndarray,
    ):
        self.data = self._compiled_updater(
            self.data, life_cycle_state, world_state, auxiliar_variables
        )


@dataclass(repr=False, eq=False)
class StateHistoryItem:
    control_cycle: int
    life_cycle_state: LifeCycleState
    observation_state: ObservationState

    def __post_init__(self):
        self.life_cycle_state = deepcopy(self.life_cycle_state)
        self.observation_state = deepcopy(self.observation_state)

    def __eq__(self, other: StateHistoryItem) -> bool:
        has_life_cycle_changed = np.any(
            other.life_cycle_state.data != self.life_cycle_state.data
        )
        has_observation_changed = np.any(
            other.observation_state.data != self.observation_state.data
        )
        return not has_life_cycle_changed and not has_observation_changed

    def __repr__(self) -> str:
        merged = {
            node.name.name: f"{self.observation_state[node]} | {life_cycle.name}"
            for node, life_cycle in self.life_cycle_state.items()
        }
        return str(merged)


@dataclass
class StateHistory:
    history: List[StateHistoryItem] = field(default_factory=list)

    def append(self, next_item: StateHistoryItem):
        if len(self.history) != 0:
            if next_item == self.history[-1]:
                return
        self.history.append(next_item)

    def get_life_cycle_history_of_node(
        self, node: MotionStatechartNode
    ) -> list[LifeCycleValues]:
        return [history_item.life_cycle_state[node] for history_item in self.history]

    def get_observation_history_of_node(
        self, node: MotionStatechartNode
    ) -> list[LifeCycleValues]:
        return [history_item.observation_state[node] for history_item in self.history]

    def __len__(self) -> int:
        return len(self.history)


@dataclass
class MotionStatechart(SubclassJSONSerializer):
    """
    Represents a motion statechart.
    A motion statechart is a directed graph of nodes and edges.
    Nodes have two states: observation state and life cycle state.
    Life cycle states indicate the current state in the life cycle of the node:
        - NOT_STARTED: the node has not started yet.
        - RUNNING: the node is running.
        - PAUSED: the node is paused.
        - DONE: the node has ended.
    Out of these 4 states, nodes are only "active" if they are in the RUNNING state.
    Observation states indicate the current observation of the node:
        - TrinaryFalse: the thing the node is observing is not True.
        - TrinaryUnknown: the node has not yet made an observation or it cannot determine its truth value yet.
        - TrinaryTrue: the thing the node is observing is True.
    Nodes are connected with edges, or transitions.
    There are 4 types of transitions:
        - start condition: If True, the node transitions from NOT_STARTED to RUNNING.
        - pause condition: If True, the node transitions from RUNNING to PAUSED.
                           If False, the node transitions from PAUSED to RUNNING.
        - end condition: If True, the node transitions from RUNNING or PAUSED to DONE.
        - reset condition: If True, the node transitions from any state to NOT_STARTED.
    If multiple conditions are met, the following order is used:
        1. reset condition
        2. end condition
        3. pause condition
        4. start condition
    How to use this class:
        1. initialized with a world
        2. add nodes.
        3. set the transition conditions of nodes
        4. compile the motion statechart.
        5. call tick() to update the observation state and life cycle state.
            tick() will raise an exception if the cancel motion condition is met.
        6. call is_end_motion() to check if the motion is done.
    """

    world: World
    """
    Reference to the world, where the motion statechart is defined.
    Symbols to the degree of freedom of the world can be used by nodes.
    """

    auxiliary_variable_manager: AuxiliaryVariableManager = field(
        default_factory=AuxiliaryVariableManager, init=False
    )

    rx_graph: rx.PyDiGraph[MotionStatechartNode] = field(
        default_factory=lambda: rx.PyDAG(multigraph=True), init=False, repr=False
    )
    """
    The underlying graph of the motion statechart.
    """

    observation_state: ObservationState = field(init=False)
    """
    Combined representation of the observation state of the motion statechart, to enable an efficient tick().
    """

    life_cycle_state: LifeCycleState = field(init=False)
    """
    Combined representation of the life cycle state of the motion statechart, to enable an efficient tick().
    """

    qp_controller: Optional[QPController] = field(default=None, init=False)

    control_cycle_counter: int = field(default=0, init=False)
    history: StateHistory = field(default_factory=StateHistory, init=False)

    def __post_init__(self):
        self.life_cycle_state = LifeCycleState(self)
        self.observation_state = ObservationState(self)

    @property
    def build_context(self) -> BuildContext:
        return BuildContext(
            world=self.world,
            auxiliary_variable_manager=self.auxiliary_variable_manager,
        )

    @property
    def execution_context(self) -> ExecutionContext:
        return ExecutionContext(world=self.world)

    @property
    def nodes(self) -> List[MotionStatechartNode]:
        return list(self.rx_graph.nodes())

    @property
    def edges(self) -> List[TrinaryCondition]:
        return self.rx_graph.edges()

    @property
    def unique_edges(self) -> List[TrinaryCondition]:
        return list(set(self.edges))

    def add_node(self, node: MotionStatechartNode):
        if self.has_node(node):
            raise ValueError(f"Node {node.name} already exists.")
        node.motion_statechart = self
        node.index = self.rx_graph.add_node(node)
        self.life_cycle_state.grow()
        self.observation_state.grow()

    def has_node(self, node: MotionStatechartNode) -> bool:
        try:
            self.get_node_by_name(node.name)
            return True
        except Exception:
            return False

    def get_node_by_name(self, name: PrefixedName) -> MotionStatechartNode:
        return next(node for node in self.nodes if node.name == name)

    def _add_transitions(self):
        for node in self.nodes:
            self._create_edge_for_condition(node, node._start_condition)
            self._create_edge_for_condition(node, node._pause_condition)
            self._create_edge_for_condition(node, node._end_condition)
            self._create_edge_for_condition(node, node._reset_condition)

    def _create_edge_for_condition(
        self, owner: MotionStatechartNode, condition: TrinaryCondition
    ):
        for parent_node in condition.variables:
            self.rx_graph.add_edge(owner.index, parent_node.index, condition)

    def _build_nodes(self):
        for node in self.nodes:
            artifacts = node.build(context=self.build_context)
            node._constraint_collection = artifacts.constraints
            node._constraint_collection.link_to_motion_statechart_node(node)
            node._observation_expression = artifacts.observation
            node._debug_expressions = artifacts.debug_expressions

    def _apply_goal_conditions_to_their_children(self):
        for goal in self.get_nodes_by_type(Goal):
            goal.apply_goal_conditions_to_children()

    def compile(self, controller_config: Optional[QPControllerConfig] = None):
        """

        :param controller_config: If not None, the QP controller will be compiled.
        """
        self._apply_goal_conditions_to_their_children()
        self._build_nodes()
        self._add_transitions()
        self.observation_state.compile()
        self.life_cycle_state.compile()
        if controller_config is not None:
            self._compile_qp_controller(controller_config)
        self.history.append(
            next_item=StateHistoryItem(
                control_cycle=self.control_cycle_counter,
                life_cycle_state=self.life_cycle_state,
                observation_state=self.observation_state,
            )
        )

    def _combine_constraint_collections_of_nodes(self) -> ConstraintCollection:
        combined_constraint_collection = ConstraintCollection()
        for node in self.nodes:
            combined_constraint_collection.merge(node._constraint_collection)
        return combined_constraint_collection

    def _compile_qp_controller(self, controller_config: QPControllerConfig):
        ordered_dofs = sorted(
            self.world.active_degrees_of_freedom,
            key=lambda dof: self.world.state._index[dof.name],
        )
        constraint_collection = self._combine_constraint_collections_of_nodes()
        if len(constraint_collection.constraints) == 0:
            self.qp_controller = None
            # to not build controller, if there are no constraints
            return
        self.qp_controller = QPController(
            config=controller_config,
            degrees_of_freedom=ordered_dofs,
            constraint_collection=constraint_collection,
            world_state_symbols=self.world.state.get_variables(),
            life_cycle_variables=self.life_cycle_state.life_cycle_symbols(),
            auxiliary_variables=self.auxiliary_variable_manager.variables,
        )
        if self.qp_controller.has_not_free_variables():
            raise EmptyProblemException(
                "Tried to compile a QPController without free variables."
            )

    def _update_observation_state(self):
        self.observation_state.update_state(
            self.life_cycle_state.data,
            self.world.state.data,
            self.auxiliary_variable_manager.resolve_auxiliary_variables(),
        )
        for node in self.nodes:
            if self.life_cycle_state[node] == LifeCycleValues.RUNNING:
                observation_overwrite = node.on_tick(context=self.execution_context)
                if observation_overwrite is not None:
                    self.observation_state[node] = observation_overwrite

    def _update_life_cycle_state(self):
        previous = self.life_cycle_state.data.copy()
        self.life_cycle_state.update_state(self.observation_state.data)
        self._trigger_life_cycle_callbacks(previous, self.life_cycle_state.data)

    def _trigger_life_cycle_callbacks(
        self,
        previous_state: np.ndarray,
        current_state: np.ndarray,
    ) -> None:
        for node in self.nodes:
            prev = LifeCycleValues(int(previous_state[node.index]))
            curr = LifeCycleValues(int(current_state[node.index]))

            if prev == curr:
                continue

            match (prev, curr):
                case (_, LifeCycleValues.NOT_STARTED):
                    node.on_reset(context=self.execution_context)
                case (LifeCycleValues.NOT_STARTED, LifeCycleValues.RUNNING):
                    node.on_start(context=self.execution_context)
                case (LifeCycleValues.RUNNING, LifeCycleValues.PAUSED):
                    node.on_pause(context=self.execution_context)
                case (LifeCycleValues.PAUSED, LifeCycleValues.RUNNING):
                    node.on_unpause(context=self.execution_context)
                case (
                    (LifeCycleValues.RUNNING | LifeCycleValues.PAUSED),
                    LifeCycleValues.DONE,
                ):
                    node.on_end(context=self.execution_context)
                case _:
                    pass

    def tick(self):
        self.control_cycle_counter += 1
        self._update_observation_state()
        self._update_life_cycle_state()
        self._raise_if_cancel_motion()
        self.history.append(
            next_item=StateHistoryItem(
                control_cycle=self.control_cycle_counter,
                life_cycle_state=self.life_cycle_state,
                observation_state=self.observation_state,
            )
        )
        if self.qp_controller is None:
            return
        next_cmd = self.qp_controller.get_cmd(
            world_state=self.world.state.data,
            life_cycle_state=self.life_cycle_state.data,
            external_collisions=np.array([], dtype=np.float64),
            self_collisions=np.array([], dtype=np.float64),
            auxiliary_variables=self.auxiliary_variable_manager.resolve_auxiliary_variables(),
        )
        self.world.apply_control_commands(
            next_cmd,
            self.qp_controller.config.control_dt or self.qp_controller.config.mpc_dt,
            self.qp_controller.config.max_derivative,
        )

    def tick_until_end(self, timeout: int = 1_000):
        """
        Calls tick until is_end_motion() returns True.
        :param timeout: Max number of ticks to perform.
        """
        for i in range(timeout):
            self.tick()
            if self.is_end_motion():
                return
        raise TimeoutError("Timeout reached while waiting for end of motion.")

    def get_nodes_by_type(
        self, node_type: Type[GenericMotionStatechartNode]
    ) -> List[GenericMotionStatechartNode]:
        return [node for node in self.nodes if isinstance(node, node_type)]

    def is_end_motion(self) -> bool:
        return any(
            self.observation_state[node] == ObservationStateValues.TRUE
            for node in self.get_nodes_by_type(EndMotion)
        )

    def _raise_if_cancel_motion(self):
        for node in self.get_nodes_by_type(CancelMotion):
            if self.observation_state[node] == ObservationStateValues.TRUE:
                raise node.exception

    def draw(self, file_name: str):
        MotionStatechartGraphviz(self).to_dot_graph_pdf(file_name=file_name)

    def to_json(self) -> Dict[str, Any]:
        self._add_transitions()
        result = super().to_json()
        result["world_name"] = self.world.name
        result["nodes"] = [node.to_json() for node in self.nodes]
        result["unique_edges"] = [edge.to_json() for edge in self.unique_edges]
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Self:
        world = kwargs["world"]
        assert world.name == data["world_name"]
        motion_statechart = cls(world=world)
        for json_data in data["nodes"]:
            node = MotionStatechartNode.from_json(json_data, **kwargs)
            motion_statechart.add_node(node)
        for json_data in data["unique_edges"]:
            transition = TrinaryCondition.from_json(
                json_data, motion_statechart=motion_statechart, **kwargs
            )
            transition.owner.set_transition(transition)
        return motion_statechart
