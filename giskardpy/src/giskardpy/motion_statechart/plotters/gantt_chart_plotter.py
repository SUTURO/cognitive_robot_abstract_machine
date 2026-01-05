from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, TYPE_CHECKING, List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from giskardpy.middleware import get_middleware
from giskardpy.motion_statechart.data_types import (
    LifeCycleValues,
    ObservationStateValues,
)
from giskardpy.motion_statechart.graph_node import Goal, MotionStatechartNode
from giskardpy.motion_statechart.plotters.styles import (
    LiftCycleStateToColor,
    ObservationStateToColor,
)
from giskardpy.utils.utils import create_path

if TYPE_CHECKING:
    from giskardpy.motion_statechart.motion_statechart import (
        MotionStatechart,
    )


@dataclass
class HistoryGanttChartPlotter:
    """
    Plot a hierarchy-aware Gantt chart of node states.

    Shows parent-child relationships of Goals by ordering rows in
    preorder and by prefixing labels with tree glyphs (├─, └─, │).
    Optional background bands and goal outlines emphasize grouping.
    """

    motion_statechart: MotionStatechart

    def plot_gantt_chart(self, file_name: str) -> None:
        """
        Render the Gantt chart and save it.

        The chart shows life cycle (top half) and observation state (bottom half)
        per node over control cycles and emphasizes hierarchical Goals.
        """

        nodes = self.motion_statechart.nodes
        if len(nodes) == 0:
            get_middleware().logwarn(
                "Gantt chart skipped: no nodes in motion statechart."
            )
            return

        history = self.motion_statechart.history.history
        if len(history) == 0:
            get_middleware().logwarn("Gantt chart skipped: empty StateHistory.")
            return

        ordered = self._iter_hierarchy()

        last_cycle = max(item.control_cycle for item in history)
        num_bars = len(self.motion_statechart.history.history[0].life_cycle_state)
        figure_width, figure_height = self._compute_figure_size(num_bars, last_cycle)

        plt.figure(figsize=(figure_width, figure_height))
        plt.grid(True, axis="x", zorder=-1)

        self._iterate_history_and_draw(ordered_nodes=ordered)

        self._format_axes(ordered_nodes=ordered)
        self._save_figure(file_name=file_name)

    def _iter_hierarchy(self) -> List[Tuple[MotionStatechartNode, int, bool]]:
        """
        Traverse nodes in preorder, yielding each node with its depth.
        """

        def walk(n: MotionStatechartNode, d: int):
            yield n, d, False
            if isinstance(n, Goal):
                for c in n.nodes:
                    yield from walk(c, d + 1)

        ordered_: List[Tuple[MotionStatechartNode, int, bool]] = []
        for root in self.motion_statechart.top_level_nodes:
            sub_list = list(walk(root, 0))
            sub_list[-1] = sub_list[-1][0], sub_list[-1][1], True
            ordered_.extend(sub_list)
        return list(reversed(ordered_))

    def _compute_figure_size(
        self, num_bars: int, last_cycle: int
    ) -> tuple[float, float]:
        figure_height = 0.7 + num_bars * 0.25
        figure_width = max(4.0, 0.5 * float(last_cycle + 1))
        return figure_width, figure_height

    def _iterate_history_and_draw(
        self,
        ordered_nodes: List[Tuple[MotionStatechartNode, int, bool]],
    ) -> None:
        for node_idx, (node, idx, final) in enumerate(ordered_nodes):
            self._plot_lifecycle_bar(node=node, node_idx=node_idx)
            self._plot_observation_bar(node=node, node_idx=node_idx)

    def _plot_lifecycle_bar(
        self,
        node: MotionStatechartNode,
        node_idx: int,
    ):
        life_cycle_history = (
            self.motion_statechart.history.get_life_cycle_history_of_node(node)
        )
        self._plot_node_bar(
            node_idx=node_idx,
            history=life_cycle_history,
            color_map=LiftCycleStateToColor,
            top=True,
        )

    def _plot_observation_bar(
        self,
        node: MotionStatechartNode,
        node_idx: int,
    ):
        obs_history = self.motion_statechart.history.get_observation_history_of_node(
            node
        )
        self._plot_node_bar(
            node_idx=node_idx,
            history=obs_history,
            color_map=ObservationStateToColor,
            top=False,
        )

    def _plot_node_bar(
        self,
        node_idx: int,
        history: List[LifeCycleValues | ObservationStateValues],
        color_map: Dict[LifeCycleValues | ObservationStateValues, str],
        top: bool,
    ) -> None:
        current_state = history[0]
        start_idx = 0
        for idx, next_state in enumerate(history[1:]):
            if current_state != next_state:
                life_cycle_width = idx + 1 - start_idx
                self._draw_block(
                    node_idx=node_idx,
                    block_start=start_idx,
                    block_width=life_cycle_width,
                    color=color_map[current_state],
                    top=top,
                )
                start_idx = idx + 1
                current_state = next_state
        last_idx = len(self.motion_statechart.history)
        life_cycle_width = last_idx - start_idx
        self._draw_block(
            node_idx=node_idx,
            block_start=start_idx,
            block_width=life_cycle_width,
            color=color_map[current_state],
            top=top,
        )

    def _draw_block(
        self,
        node_idx,
        block_start,
        block_width,
        color,
        top: bool,
        bar_height: float = 0.8,
    ):
        if top:
            y = node_idx + bar_height / 4
        else:
            y = node_idx - bar_height / 4
        plt.barh(
            y,
            block_width,
            height=bar_height / 2,
            left=block_start,
            color=color,
            zorder=2,
        )

    def _format_axes(
        self,
        ordered_nodes: List[Tuple[MotionStatechartNode, int, bool]],
    ) -> None:
        last_cycle = len(self.motion_statechart.history)
        plt.xlabel("Control cycle")
        plt.xlim(0, len(self.motion_statechart.history))
        plt.xticks(
            np.arange(
                0,
                last_cycle + 1,
                max(1, (last_cycle - 0 + 1) // 10),
            )
        )
        plt.ylabel("Nodes")
        num_bars = len(self.motion_statechart.history.history[0].life_cycle_state)
        plt.ylim(-0.8, num_bars - 1 + 0.8)

        def make_label(node: MotionStatechartNode, depth: int, final: bool) -> str:
            if depth == 0:
                return node.unique_name
            if final:
                return "└─" * (depth - 1) + "└─ " + node.unique_name
            else:
                return "│  " * (depth - 1) + "├─ " + node.unique_name

        node_names = [make_label(n, depth, final) for n, depth, final in ordered_nodes]
        node_idx = list(range(len(node_names)))
        plt.yticks(node_idx, node_names)
        plt.gca().yaxis.tick_right()
        plt.tight_layout()

    def _save_figure(self, file_name: str) -> None:
        create_path(file_name)
        plt.savefig(file_name)
        plt.close()
        get_middleware().loginfo(f"Saved gantt chart to {file_name}.")
