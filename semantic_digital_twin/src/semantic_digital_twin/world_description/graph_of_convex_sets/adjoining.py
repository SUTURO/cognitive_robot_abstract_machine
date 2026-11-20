"""
Paths through the free spaces of regions that border each other, such as the rooms of
one storey, each of which has a free space of its own.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from functools import cached_property

import numpy as np
import rustworkx as rx

from krrood_spatial_math.spatial_types import Point2
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import PlanarBoundingBox
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    PlanarGraphOfBoundingBoxes,
)

# %% places and the legs between them


@dataclass(kw_only=True)
class PlaceInFreeSpace:
    """
    A point within one of the free spaces of :class:`AdjoiningFreeSpaces`.
    """

    free_space: PlanarGraphOfBoundingBoxes
    """
    The free space the point lies in.
    """

    position: Point2
    """
    The point, which lies in a node of :attr:`free_space`.
    """


@dataclass(kw_only=True)
class Leg:
    """
    The waypoints leading from one place to another.
    """

    waypoints: list[Point2]
    """
    The waypoints, beginning at the place the leg leaves and ending at the place it
    reaches.
    """

    length: float
    """
    The distance travelled along :attr:`waypoints`, in m.
    """

    @classmethod
    def from_waypoints(cls, *, waypoints: list[Point2], world: World) -> Leg:
        """
        :param waypoints: The waypoints of the leg, in any reference frames.
        :param world: The world the reference frames of the waypoints belong to.
        :return: The leg along the waypoints, with the length of the line through them.
        """
        positions = np.array(
            [
                world.transform(waypoint, world.root).to_np()[:2]
                for waypoint in waypoints
            ]
        )
        return cls(
            waypoints=waypoints,
            length=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
        )

    def reversed(self) -> Leg:
        """
        :return: The same leg, travelled the other way.
        """
        return Leg(waypoints=self.waypoints[::-1], length=self.length)


# %% joining the free spaces


@dataclass(kw_only=True)
class AdjoiningFreeSpaces:
    """
    The free spaces of regions that border each other, joined wherever a node of one
    touches a node of another, so that a path can lead through several of them.
    """

    world: World
    """
    The world the free spaces lie in.
    """

    free_spaces: list[PlanarGraphOfBoundingBoxes]
    """
    The free spaces that are joined.
    """

    gap_tolerance: float = 0.01
    """
    How far apart nodes of two free spaces may lie and still count as touching, in m.
    """

    def path_from_to(
        self, *, start: PlaceInFreeSpace, goal: PlaceInFreeSpace
    ) -> list[Point2] | None:
        """
        :param start: Where the path begins.
        :param goal: Where the path ends.
        :return: The shortest path from *start* to *goal* through the free
            spaces, each waypoint in the frame of the free space it lies in, or None if
            no path exists. Where the path passes from one free space into another, it
            holds one waypoint on either side of the border.
        """
        if start.free_space is goal.free_space:
            return start.free_space.path_from_to(start.position, goal.position)

        graph = rx.PyDiGraph(multigraph=False)
        start_index = graph.add_node(start)
        goal_index = graph.add_node(goal)
        for exit_place, entry_place in self._crossings:
            self._add_leg(
                graph=graph,
                source=graph.add_node(exit_place),
                target=graph.add_node(entry_place),
                leg=Leg(
                    waypoints=[exit_place.position, entry_place.position],
                    length=0.0,
                ),
            )
        self._connect_places_within_each_free_space(graph=graph)

        paths = rx.dijkstra_shortest_paths(
            graph, start_index, target=goal_index, weight_fn=lambda leg: leg.length
        )
        if goal_index not in paths:
            return None

        waypoints = [
            self.world.transform(
                start.position, start.free_space.search_space.reference_frame
            )
        ]
        for source, target in itertools.pairwise(paths[goal_index]):
            waypoints.extend(graph.get_edge_data(source, target).waypoints[1:])
        return waypoints

    @cached_property
    def _crossings(self) -> list[tuple[PlaceInFreeSpace, PlaceInFreeSpace]]:
        """
        :return: For every pair of touching nodes of two different free spaces, a place
            in each node next to where they touch, the place in the first node first.
        """
        crossings = []
        for free_space, other_free_space in itertools.combinations(self.free_spaces, 2):
            for node, other_node in itertools.product(
                free_space.graph.nodes(), other_free_space.graph.nodes()
            ):
                entry_region = self._border_region(into=other_node, beside=node)
                if entry_region is None:
                    continue
                exit_region = self._border_region(into=node, beside=other_node)
                crossings.append(
                    (
                        PlaceInFreeSpace(
                            free_space=free_space, position=exit_region.center
                        ),
                        PlaceInFreeSpace(
                            free_space=other_free_space,
                            position=entry_region.center,
                        ),
                    )
                )
        return crossings

    def _border_region(
        self, *, into: PlanarBoundingBox, beside: PlanarBoundingBox
    ) -> PlanarBoundingBox | None:
        """
        :param into: The node whose part is returned.
        :param beside: The node that part borders on.
        :return: The part of *into* within :attr:`gap_tolerance` of *beside*, or None
            if the two nodes do not touch.
        """
        return beside.bloat(self.gap_tolerance, self.gap_tolerance).intersection_with(
            into
        )

    def _connect_places_within_each_free_space(self, *, graph: rx.PyDiGraph) -> None:
        """
        Add a leg between every two places of `graph` that lie in the same free space
        and are connected through it.
        """
        for free_space in self.free_spaces:
            indices = [
                index
                for index in graph.node_indices()
                if graph[index].free_space is free_space
            ]
            for source, target in itertools.combinations(indices, 2):
                waypoints = free_space.path_from_to(
                    graph[source].position, graph[target].position
                )
                if waypoints is None:
                    continue
                self._add_leg(
                    graph=graph,
                    source=source,
                    target=target,
                    leg=Leg.from_waypoints(waypoints=waypoints, world=self.world),
                )

    @staticmethod
    def _add_leg(*, graph: rx.PyDiGraph, source: int, target: int, leg: Leg) -> None:
        """
        Add `leg` from `source` to `target` to `graph`, and the leg back.
        """
        graph.add_edge(source, target, leg)
        graph.add_edge(target, source, leg.reversed())
