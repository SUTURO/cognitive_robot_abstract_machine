from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.graph_node import NodeArtifacts, Task
from giskardpy.motion_statechart.monitors.payload_monitors import (
    CountSimulationTimeSeconds,
    Print,
)
from giskardpy.motion_statechart.ros_context import RosContextExtension
from typing_extensions import Optional

from coraplex.datastructures.enums import ExecutionType
from coraplex.exceptions import QueryWithoutLocation
from coraplex.perception import PerceptionInterface, PerceptionQuery
from coraplex.plans.executables import GiskardExecutable
from coraplex.robot_plans.motions.base import BaseMotion
from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)

# %% perceiving inside the motion chart


@dataclass(eq=False, repr=False)
class WorldWritingPerceptionTask(Task, ABC):
    """
    Motion statechart node that answers a perception query and writes what it saw into
    the world.

    The node adds no motion constraints. The query is answered on the node's first tick,
    so the whole perception takes one tick however long the source needs to reply, and
    the node then observes ``TRUE`` so the surrounding sequence continues.

    ..warning:: That tick blocks until the source replies, which on the real robot holds
        up the control loop for as long as the pipeline takes to answer.
    """

    query: PerceptionQuery = field(kw_only=True)
    """
    What to look for and where.
    """

    execution_type: ExecutionType | None = field(kw_only=True)
    """
    Which source answers the query.

    Carried by the node rather than read from the execution environment, because on the
    real robot the chart is answered in the controller's process, where that environment
    does not exist. None when the chart was built without one, which :meth:`build`
    rejects.
    """

    perception_source: PerceptionInterface | None = field(init=False, default=None)
    """
    The source answering the query, resolved during :meth:`build`.
    """

    _answer_written: bool = field(init=False, default=False, repr=False)
    """
    Whether the query has already been answered and written into the world.
    """

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        self.perception_source = PerceptionInterface.for_execution_type(
            self.execution_type,
            context.require_extension(RosContextExtension).ros_node,
        )
        return NodeArtifacts()

    def on_start(self, context: MotionStatechartContext) -> None:
        self._answer_written = False

    def on_tick(
        self, context: MotionStatechartContext
    ) -> ObservationStateValues | None:
        if self._answer_written:
            return ObservationStateValues.TRUE
        self.write_answer()
        self._answer_written = True
        return ObservationStateValues.TRUE

    @abstractmethod
    def write_answer(self) -> None:
        """
        Ask :attr:`perception_source` the query and write what it saw into the world.
        """


@dataclass(eq=False, repr=False)
class PerceptionTask(WorldWritingPerceptionTask):
    """
    Perception that moves the one object the query asks for to where it was seen.
    """

    accept_first_if_multiple: bool = False
    """
    Whether several candidates may be resolved by taking the first one.

    When False, several candidates raise
    :class:`~coraplex.exceptions.UnidentifiedDetections` instead of being chosen between.
    """

    def write_answer(self) -> None:
        detection = self.perception_source.detect(
            self.query, self.accept_first_if_multiple
        )
        detection.apply_to(
            self.query.world, trust_orientation=self.query.trust_detected_orientation
        )


@dataclass(eq=False, repr=False)
class OccupantPerceptionTask(WorldWritingPerceptionTask):
    """
    Perception that makes the people in the world within the query's
    :attr:`~coraplex.perception.PerceptionQuery.location` the ones the source saw there.

    Every :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Human`
    whose root lies in the location is removed, and one is added at the pose of each
    detection, so a person who left is no longer in the world.
    """

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        """
        :raises QueryWithoutLocation: If the query names no location.
        """
        if self.query.location is None:
            raise QueryWithoutLocation(self.query)
        return super().build(context)

    def write_answer(self) -> None:
        detections = self.perception_source.detect_all(self.query)
        world = self.query.world
        location = self.query.location
        with world.modify_world():
            for occupant in location.semantic_annotations_within(Human):
                world.remove_semantic_annotation(occupant)
                world.remove_connection(occupant.root.parent_connection)
                world.remove_kinematic_structure_entity(occupant.root)
            for index, detection in enumerate(detections):
                name = f"{location.name.name}_occupant_{index}"
                Human.get_annotation_specification(name, BodySpecification(name)).spawn(
                    world,
                    parent_T_self=world.transform(
                        detection.pose, world.root
                    ).homogeneous_matrix,
                )


@dataclass
class DetectingMotion(BaseMotion):
    """
    Detects the object a perception query asks for and moves it to where it was seen.

    The detection runs inside the motion statechart, so it merges with the surrounding
    motions and a motion planned after it binds the corrected pose when it starts.
    """

    query: PerceptionQuery
    """
    Query for the perception system that should be answered.
    """

    accept_first_if_multiple: bool = False
    """
    Whether several candidates may be resolved by taking the first one.

    When False, several candidates raise
    :class:`~coraplex.exceptions.UnidentifiedDetections` instead of being chosen between.
    """

    @property
    def _motion_chart(self) -> Task:
        return PerceptionTask(
            query=self.query,
            execution_type=GiskardExecutable.execution_type,
            accept_first_if_multiple=self.accept_first_if_multiple,
        )


@dataclass
class DetectingOccupantsMotion(BaseMotion):
    """
    Updates the people in the world within a region to the ones perceived there.
    """

    query: PerceptionQuery
    """
    Query for the people within its
    :attr:`~coraplex.perception.PerceptionQuery.location`.
    """

    @property
    def _motion_chart(self) -> Task:
        return OccupantPerceptionTask(
            query=self.query, execution_type=GiskardExecutable.execution_type
        )


# %% reporting and waiting inside the motion chart


@dataclass
class PrintMotion(BaseMotion):
    """
    Prints a message when the motion chart reaches it, between the surrounding motions.
    """

    message: str
    """
    The message to print.
    """

    @property
    def _motion_chart(self) -> Print:
        return Print(message=self.message)


@dataclass
class WaitMotion(BaseMotion):
    """
    Lets the motion chart wait before it continues with the next motion.
    """

    seconds: float
    """
    How long to wait, in seconds of control time: the control cycles that pass, times
    the control period.
    """

    @property
    def _motion_chart(self) -> CountSimulationTimeSeconds:
        return CountSimulationTimeSeconds(seconds=self.seconds)
