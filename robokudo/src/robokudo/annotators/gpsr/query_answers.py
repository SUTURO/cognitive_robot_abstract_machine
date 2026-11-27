"""
The kinds of things GPSR perception queries ask about, what every answer to them builds
on, and the answers about postures, which need no model besides the pose detector.
"""

from __future__ import annotations

# %% Imports
from enum import StrEnum

from builtin_interfaces.msg import Time
from py_trees.common import Status
from robokudo_msgs.action import Query
from robokudo_msgs.msg import ObjectDesignator
from std_msgs.msg import Header
from typing_extensions import Any

from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.gpsr.belief_state import BeliefEntry
from robokudo.annotators.gpsr.lying import HumanLyingAnnotator
from robokudo.annotators.gpsr.sitting import HumanSittingAnnotator
from robokudo.annotators.gpsr.standing import HumanStandingAnnotator
from robokudo.cas import CAS, CASViews
from robokudo.types.annotation import PositionAnnotation
from robokudo.types.human import ActivityAnnotation
from robokudo.types.scene import HumanHypothesis
from robokudo.utils.annotation_conversion import Position2ODConverter
from robokudo.utils.query import QueryHandler


# %% query types
class QueryType(StrEnum):
    """
    Types accepted by GPSR perception queries.
    """

    HUMAN = "human"
    """
    A query about persons.
    """
    HUMANS = "humans"
    """
    A query about several persons.
    """
    OBJECT = "object"
    """
    A query about objects.
    """
    RECALL = "recall"
    """
    A query about remembered persons and objects.
    """
    FACE = "face"
    """
    A query about faces.
    """
    DOOR = "door"
    """
    A query about doors, answered with the trained door detector.
    """
    DOOR_OPEN_VOCABULARY = "door_open_vocabulary"
    """
    A query about doors, answered with open vocabulary detection.
    """


# %% camera header
def create_camera_header(cas: CAS) -> Header:
    """
    :param cas: The CAS holding the camera observation.
    :return: The header of the current camera image: its acquisition time and the frame
        of the camera.
    """
    observation = cas.require_camera_observation()
    return Header(
        frame_id=str(observation.camera.root.name),
        stamp=Time(
            sec=observation.timestamp_nanoseconds // 1_000_000_000,
            nanosec=observation.timestamp_nanoseconds % 1_000_000_000,
        ),
    )


# %% person answers
class GenerateResult(BaseAnnotator):
    """
    Builds answers for detected hypotheses.
    """

    def add_identity(
        self,
        object_designator: ObjectDesignator,
        human_hypothesis: HumanHypothesis,
    ) -> None:
        """
        Add the person's belief identifier to the answer when available.

        :param object_designator: The answer that receives the identifier.
        :param human_hypothesis: The person of the answer.
        """
        belief_entries = self.get_cas().filter_by_type(
            BeliefEntry, human_hypothesis.annotations
        )
        if belief_entries:
            object_designator.uid = str(belief_entries[0].id)

    def add_positions(
        self, object_designator: ObjectDesignator, positions: list[PositionAnnotation]
    ) -> None:
        """
        Add observed positions and their sources to the answer, in the world frame the
        camera pose refers to.

        :param object_designator: The answer that receives the positions.
        :param positions: The observed positions, relative to the camera.
        """
        converter = Position2ODConverter()
        for position in positions:
            converter.convert(position, self.get_cas(), object_designator)


# %% posture answers
class GenerateHumanPostureResult(GenerateResult):
    """
    Answers queries for persons whose unambiguous posture matches the query.
    """

    def __init__(
        self, name: str = "GenerateHumanPostureResult", *args: Any, **kwargs: Any
    ) -> None:
        """
        :param name: Name of this annotator in the tree.
        """
        super().__init__(*args, **kwargs)
        self.name = name

    def update(self) -> Status:
        """
        Overrides :meth:`BaseAnnotator.update` to answer with every person whose only
        posture is the queried one.

        :return: :attr:`Status.SUCCESS`.
        """
        query = self.get_cas().get(CASViews.QUERY)

        human_hypotheses = self.get_cas().filter_annotations_by_type(HumanHypothesis)
        query_result = []
        result = Query.Result()

        keypoint_rule_sources = {
            HumanSittingAnnotator.__name__,
            HumanStandingAnnotator.__name__,
            HumanLyingAnnotator.__name__,
        }

        for human_hypothesis in human_hypotheses:
            object_designator = ObjectDesignator()
            object_designator.type = query.obj.type
            activities = self.get_cas().filter_by_type(
                ActivityAnnotation, human_hypothesis.annotations
            )
            positions = self.get_cas().filter_by_type(
                PositionAnnotation, human_hypothesis.annotations
            )

            keypoint_postures = {
                activity.type
                for activity in activities
                if activity.source in keypoint_rule_sources
            }
            if len(keypoint_postures) != 1:
                self.rk_logger.info(
                    f"Posture of human hypothesis {human_hypothesis.id}: undecided, rules fired: {sorted(keypoint_postures)}"
                )
                continue
            posture = keypoint_postures.pop()
            if posture == HumanLyingAnnotator.posture:
                posture = HumanLyingAnnotator.query_word
            self.rk_logger.info(
                f"Posture of human hypothesis {human_hypothesis.id}: {posture}"
            )
            if posture == query.obj.attribute[0]:
                object_designator.attribute.append(posture)

            if object_designator.attribute:
                self.add_identity(object_designator, human_hypothesis)
                self.add_positions(object_designator, positions)

                query_result.append(object_designator)

        result.res = query_result
        QueryHandler.send_answer(result)
        self.feedback_message = "Query result generated."
        return Status.SUCCESS
