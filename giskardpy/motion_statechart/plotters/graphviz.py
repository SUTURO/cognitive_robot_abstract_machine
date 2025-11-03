from __future__ import annotations

import re
from dataclasses import dataclass, field

import pydot
from typing_extensions import List, Dict, Optional, Union, Set, TYPE_CHECKING

from giskardpy.motion_statechart.data_types import LifeCycleValues, ObservationState
from giskardpy.motion_statechart.graph_node import (
    Goal,
    TransitionKind,
    TrinaryCondition,
)

from giskardpy.motion_statechart.graph_node import (
    EndMotion,
    CancelMotion,
    MotionStatechartNode,
)

if TYPE_CHECKING:
    from giskardpy.motion_statechart.motion_statechart import MotionStatechart


def extract_node_names_from_condition(condition: str) -> Set[str]:
    matches = re.findall(r'"(.*?)"|\'(.*?)\'', condition)
    return set(match for group in matches for match in group if match)


def format_condition(condition: str) -> str:
    condition = condition.replace(" and ", "<BR/>       and ")
    condition = condition.replace(" or ", "<BR/>       or ")
    condition = condition.replace("1.0", "True")
    condition = condition.replace("0.0", "False")
    return condition


NotStartedColor = "#9F9F9F"
MyBLUE = "#0000DD"
MyGREEN = "#006600"
MyORANGE = "#996900"
MyRED = "#993000"
MyGRAY = "#E0E0E0"

ChatGPTGreen = "#28A745"
ChatGPTOrange = "#E6AC00"
ChatGPTRed = "#DC3545"
ChatGPTBlue = "#007BFF"
ChatGPTGray = "#8F959E"

StartCondColor = ChatGPTGreen
PauseCondColor = ChatGPTOrange
EndCondColor = ChatGPTRed
ResetCondColor = ChatGPTGray

MonitorTrueGreen = "#B6E5A0"
MonitorFalseRed = "#FF5024"
FONT = "sans-serif"
LineWidth = 4
NodeSep = 1
RankSep = 1
ArrowSize = 1
Fontsize = 15
GoalNodeStyle = "filled"
GoalNodeShape = "none"
GoalClusterStyle = "filled"
MonitorStyle = "filled, rounded"
MonitorShape = "rectangle"
TaskStyle = "filled, diagonals"
TaskShape = "rectangle"
ConditionFont = "monospace"

ResetSymbol = "⟲"

ObservationStateToColor: Dict[ObservationState, str] = {
    ObservationState.unknown: ResetCondColor,
    ObservationState.true: MonitorTrueGreen,
    ObservationState.false: MonitorFalseRed,
}

ObservationStateToSymbol: Dict[ObservationState, str] = {
    ObservationState.unknown: "?",
    ObservationState.true: "True",
    ObservationState.false: "False",
}

ObservationStateToEdgeStyle: Dict[ObservationState, Dict[str, str]] = {
    ObservationState.unknown: {
        "penwidth": (LineWidth * 1.5) / 2,
        # 'label': '<<FONT FACE="monospace"><B>?</B></FONT>>',
        "fontsize": Fontsize * 1.333,
    },
    ObservationState.true: {"penwidth": LineWidth * 1.5},
    ObservationState.false: {"style": "dashed", "penwidth": LineWidth * 1.5},
}

LiftCycleStateToColor: Dict[LifeCycleValues, str] = {
    LifeCycleValues.NOT_STARTED: ResetCondColor,
    LifeCycleValues.RUNNING: StartCondColor,
    LifeCycleValues.PAUSED: PauseCondColor,
    LifeCycleValues.DONE: EndCondColor,
    LifeCycleValues.FAILED: "red",
}

LiftCycleStateToSymbol: Dict[LifeCycleValues, str] = {
    # LifeCycleState.not_started: '○',
    LifeCycleValues.NOT_STARTED: "—",
    LifeCycleValues.RUNNING: "▶",
    # LifeCycleState.paused: '⏸',
    LifeCycleValues.PAUSED: "<B>||</B>",
    LifeCycleValues.DONE: "■",
    LifeCycleValues.FAILED: "red",
}


@dataclass
class MotionStatechartGraphviz:
    motion_statechart: MotionStatechart
    graph: pydot.Graph = field(init=False)
    compact: bool = False
    _cluster_map: Dict[MotionStatechartNode, pydot.Cluster] = field(
        init=False, default_factory=dict
    )

    def __post_init__(self):
        self.graph = pydot.Dot(
            graph_type="digraph",
            graph_name="",
            ranksep=RankSep if not self.compact else RankSep * 0.5,
            nodesep=NodeSep if not self.compact else NodeSep * 0.5,
            compound=True,
            ratio="compress",
        )

    def format_motion_graph_node(
        self,
        node: MotionStatechartNode,
    ) -> str:
        obs_state = self.motion_statechart.observation_state[node]
        life_cycle_state = self.motion_statechart.life_cycle_state[node]
        obs_color = ObservationStateToColor[obs_state]
        obs_text = ObservationStateToSymbol[obs_state]
        life_color = LiftCycleStateToColor[life_cycle_state]
        life_symbol = LiftCycleStateToSymbol[life_cycle_state]
        label = (
            f'<<TABLE  BORDER="0" CELLBORDER="0" CELLSPACING="0">'
            f"<TR>"
            f'  <TD WIDTH="100%" HEIGHT="{LineWidth}"></TD>'
            f"</TR>"
            f"<TR>"
            f"  <TD><B> {node.name} </B></TD>"
            f"</TR>"
            f"<TR>"
            f'  <TD CELLPADDING="0">'
            f'    <TABLE BORDER="0" CELLBORDER="2" CELLSPACING="0" WIDTH="100%">'
            f"      <TR>"
            f'        <TD BGCOLOR="{obs_color}" WIDTH="50%" FIXEDSIZE="FALSE"><FONT FACE="monospace">{obs_text}</FONT></TD>'
            f"        <VR/>"
            f'        <TD BGCOLOR="{life_color}" WIDTH="50%" FIXEDSIZE="FALSE"><FONT FACE="monospace">{life_symbol}</FONT></TD>'
            f"      </TR>"
            f"    </TABLE>"
            f"  </TD>"
            f"</TR>"
        )
        if self.compact:
            label += (
                f"<TR>" f'  <TD WIDTH="100%" HEIGHT="{LineWidth*2.5}"></TD>' f"</TR>"
            )
        else:
            label += self._build_condition_block(node)
        label += f"</TABLE>>"
        return label

    def _build_condition_block(
        self, node: MotionStatechartNode, line_color="black"
    ) -> str:
        start_condition = format_condition(str(node._start_condition))
        pause_condition = format_condition(str(node._pause_condition))
        end_condition = format_condition(str(node._end_condition))
        reset_condition = format_condition(str(node._reset_condition))
        label = (
            f'<TR><TD WIDTH="100%" BGCOLOR="{line_color}" HEIGHT="{LineWidth}"></TD></TR>'
            f'<TR><TD ALIGN="LEFT" BALIGN="LEFT" CELLPADDING="{LineWidth}"><FONT FACE="{ConditionFont}">start:{start_condition}</FONT></TD></TR>'
        )
        if not isinstance(node, (EndMotion, CancelMotion)):
            label += (
                f'<TR><TD WIDTH="100%" BGCOLOR="{line_color}" HEIGHT="{LineWidth}"></TD></TR>'
                f'<TR><TD ALIGN="LEFT" BALIGN="LEFT" CELLPADDING="{LineWidth}"><FONT FACE="{ConditionFont}">pause:{pause_condition}</FONT></TD></TR>'
            )
            label += (
                f'<TR><TD WIDTH="100%" BGCOLOR="{line_color}" HEIGHT="{LineWidth}"></TD></TR>'
                f'<TR><TD ALIGN="LEFT" BALIGN="LEFT" CELLPADDING="{LineWidth}"><FONT FACE="{ConditionFont}">end  :{end_condition}</FONT></TD></TR>'
            )
            label += (
                f'<TR><TD WIDTH="100%" BGCOLOR="{line_color}" HEIGHT="{LineWidth}"></TD></TR>'
                f'<TR><TD ALIGN="LEFT" BALIGN="LEFT" CELLPADDING="{LineWidth}"><FONT FACE="{ConditionFont}">reset:{reset_condition}</FONT></TD></TR>'
            )
        return label

    def escape_name(self, name: str) -> str:
        return f'"{name}"'

    def get_cluster_of_node(
        self, node_name: str, graph: Union[pydot.Graph, pydot.Cluster]
    ) -> Optional[pydot.Cluster]:
        node_cluster = None
        for cluster in graph.get_subgraphs():
            if (
                len(cluster.get_node(self.escape_name(node_name))) == 1
                or len(cluster.get_node(node_name)) == 1
            ):
                node_cluster = cluster
                break
        return node_cluster

    def _add_node(
        self,
        graph: pydot.Graph,
        node: MotionStatechartNode,
    ) -> pydot.Node:
        pydot_node = self._create_pydot_node(node)
        if len(node._plot_extra_boarder_styles) == 0:
            graph.add_node(pydot_node)
            return pydot_node
        child = pydot_node
        for index, style in enumerate(node._plot_extra_boarder_styles):
            c = pydot.Cluster(
                graph_name=f"{node.name}",
                penwidth=LineWidth,
                style=node._plot_extra_boarder_styles[index],
                color="black",
            )
            if index == 0:
                c.add_node(child)
            else:
                c.add_subgraph(child)
            child = c
        if len(node._plot_extra_boarder_styles) > 0:
            graph.add_subgraph(c)
        return pydot_node

    def _create_pydot_node(self, node: MotionStatechartNode) -> pydot.Node:
        label = self.format_motion_graph_node(node=node)
        pydot_node = pydot.Node(
            str(node.name),
            label=label,
            shape=node._plot_shape,
            color="black",
            style=node._plot_style,
            margin=0,
            fillcolor="white",
            fontname=FONT,
            fontsize=Fontsize,
            penwidth=LineWidth,
        )
        return pydot_node

    def to_dot_graph(self) -> pydot.Graph:
        self._cluster_map[None] = self.graph
        top_level_nodes = [
            node for node in self.motion_statechart.nodes if not node.parent_node
        ]
        self.add_nodes(self.graph, top_level_nodes)
        self.add_edges()
        return self.graph

    def add_nodes(
        self,
        parent_cluster: Union[pydot.Graph, pydot.Cluster],
        nodes: List[MotionStatechartNode],
    ):
        for i, node in enumerate(nodes):
            if isinstance(node, Goal):
                goal_cluster = self._add_cluster(node, parent_cluster)
                self._add_node(
                    graph=goal_cluster,
                    node=node,
                )
                self.add_nodes(goal_cluster, node.nodes)

            self._add_node(
                parent_cluster,
                node=node,
            )

    def _add_cluster(
        self,
        node: MotionStatechartNode,
        parent_cluster: Union[pydot.Graph, pydot.Cluster],
    ):
        goal_cluster = pydot.Cluster(
            graph_name=str(node.name),
            fontname=FONT,
            fontsize=Fontsize,
            style=GoalClusterStyle,
            color="black",
            fillcolor="white",
            penwidth=LineWidth,
        )
        parent_cluster.add_subgraph(goal_cluster)
        self._cluster_map[node] = goal_cluster
        return goal_cluster

    def add_edges(self):
        transition: TrinaryCondition
        for edge_index, (
            parent_node_index,
            child_node_index,
            transition,
        ) in self.motion_statechart.rx_graph.edge_index_map().items():
            parent_node = self.motion_statechart.rx_graph.get_node_data(
                parent_node_index
            )
            child_node = self.motion_statechart.rx_graph.get_node_data(child_node_index)
            if not self._are_nodes_in_same_cluster(parent_node, child_node):
                continue
            if transition.kind == TransitionKind.START:
                self.add_start_condition_edge(parent_node, child_node)
            if transition.kind == TransitionKind.PAUSE:
                self.add_pause_condition_edge(parent_node, child_node)
            if transition.kind == TransitionKind.END:
                self.add_end_condition_edge(parent_node, child_node)
            if transition.kind == TransitionKind.RESET:
                self.add_reset_condition_edge(parent_node, child_node)

    def _are_nodes_in_same_cluster(
        self, parent_node: MotionStatechartNode, child_node: MotionStatechartNode
    ) -> bool:
        if parent_node.parent_node is None and child_node.parent_node is None:
            return True
        if (parent_node.parent_node is None and child_node.parent_node is not None) or (
            parent_node.parent_node is not None and child_node.parent_node is None
        ):
            return False
        return parent_node.parent_node.name == child_node.parent_node.name

    def add_start_condition_edge(
        self,
        parent_node: MotionStatechartNode,
        child_node: MotionStatechartNode,
    ):
        graph = self._cluster_map[parent_node.parent_node]
        destination_node = child_node
        source_node = parent_node
        source_node_name = str(destination_node.name)
        destination_node_name = str(source_node.name)
        node_cluster = self.get_cluster_of_node(destination_node_name, graph)
        sub_node_cluster = self.get_cluster_of_node(source_node_name, graph)
        kwargs = {}
        if node_cluster is not None:
            kwargs["lhead"] = node_cluster.get_name()
        if sub_node_cluster is not None:
            kwargs["ltail"] = sub_node_cluster.get_name()
        source_observation_state = self.motion_statechart.observation_state[
            destination_node
        ]
        kwargs.update(ObservationStateToEdgeStyle[source_observation_state])
        graph.add_edge(
            pydot.Edge(
                src=source_node_name,
                dst=destination_node_name,
                color=StartCondColor,
                arrowsize=ArrowSize,
                **kwargs,
            )
        )

    def add_pause_condition_edge(
        self,
        parent_node: MotionStatechartNode,
        child_node: MotionStatechartNode,
    ):
        graph = self._cluster_map[parent_node.parent_node]
        destination_node = child_node
        source_node = parent_node
        source_node_name = str(destination_node.name)
        destination_node_name = str(source_node.name)
        node_cluster = self.get_cluster_of_node(destination_node_name, graph)
        sub_node_cluster = self.get_cluster_of_node(source_node_name, graph)
        kwargs = {}
        if node_cluster is not None:
            kwargs["lhead"] = node_cluster.get_name()
        if sub_node_cluster is not None:
            kwargs["ltail"] = sub_node_cluster.get_name()
        source_observation_state = self.motion_statechart.observation_state[
            destination_node
        ]
        kwargs.update(ObservationStateToEdgeStyle[source_observation_state])
        graph.add_edge(
            pydot.Edge(
                src=source_node_name,
                dst=destination_node_name,
                color=PauseCondColor,
                minlen=0,
                arrowsize=ArrowSize,
                **kwargs,
            )
        )

    def add_end_condition_edge(
        self,
        parent_node: MotionStatechartNode,
        child_node: MotionStatechartNode,
    ):
        graph = self._cluster_map[parent_node.parent_node]
        destination_node = child_node
        source_node = parent_node
        source_node_name = str(destination_node.name)
        destination_node_name = str(source_node.name)
        node_cluster = self.get_cluster_of_node(destination_node_name, graph)
        sub_node_cluster = self.get_cluster_of_node(source_node_name, graph)
        kwargs = {}
        if node_cluster is not None:
            kwargs["lhead"] = node_cluster.get_name()
        if sub_node_cluster is not None:
            kwargs["ltail"] = sub_node_cluster.get_name()
        source_observation_state = self.motion_statechart.observation_state[
            destination_node
        ]
        kwargs.update(ObservationStateToEdgeStyle[source_observation_state])
        graph.add_edge(
            pydot.Edge(
                src=source_node_name,
                dst=destination_node_name,
                color=EndCondColor,
                arrowhead="none",
                arrowtail="normal",
                dir="both",
                arrowsize=ArrowSize,
                **kwargs,
            )
        )

    def add_reset_condition_edge(
        self,
        parent_node: MotionStatechartNode,
        child_node: MotionStatechartNode,
    ):
        graph = self._cluster_map[parent_node.parent_node]
        destination_node = child_node
        source_node = parent_node
        source_node_name = str(destination_node.name)
        destination_node_name = str(source_node.name)
        node_cluster = self.get_cluster_of_node(destination_node_name, graph)
        sub_node_cluster = self.get_cluster_of_node(source_node_name, graph)
        kwargs = {}
        if node_cluster is not None:
            kwargs["lhead"] = node_cluster.get_name()
        if sub_node_cluster is not None:
            kwargs["ltail"] = sub_node_cluster.get_name()
        source_observation_state = self.motion_statechart.observation_state[
            destination_node
        ]
        kwargs.update(ObservationStateToEdgeStyle[source_observation_state])
        graph.add_edge(
            pydot.Edge(
                src=source_node_name,
                dst=destination_node_name,
                color=ResetCondColor,
                arrowhead="none",
                arrowtail="normal",
                minlen=0,
                dir="both",
                arrowsize=ArrowSize,
                **kwargs,
            )
        )
