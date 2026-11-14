"""Answer human detection and posture queries using the HSR camera."""

from __future__ import annotations

# %% imports
from enum import StrEnum

from py_trees.composites import Sequence
from typing_extensions import TYPE_CHECKING

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.cluster_position import ClusterPositionAnnotator
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.gpsr.gpsr_results import GenerateHumanPostureResult, QueryType
from robokudo.annotators.gpsr.lying import HumanLyingAnnotator
from robokudo.annotators.gpsr.sitting import HumanPosture, HumanSittingAnnotator
from robokudo.annotators.gpsr.standing import HumanStandingAnnotator
from robokudo.annotators.human_and_pose import HumanAndPoseAnnotator
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.query import GenerateQueryResult, QueryAnnotator
from robokudo.descriptors.factories.cr_descriptor_factory import (
    CollectionReaderDescriptorFactory,
)
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline
from robokudo.tree_components.query_based_task_scheduler import QueryBasedScheduler

if TYPE_CHECKING:
    from robokudo_msgs.action import Query


# %% query routes
class HumanRoute(StrEnum):
    """The supported kinds of human perception queries."""

    DETECTION = "detection"
    """Find persons without filtering their posture."""

    POSTURE = "posture"
    """Find persons matching a requested posture."""


# %% analysis engine
class AnalysisEngine(AnalysisEngineInterface):
    """Detect humans and answer queries about their posture."""

    def name(self) -> str:
        return "human_detection"

    def implementation(self) -> Pipeline:
        camera = CollectionReaderAnnotator(
            descriptor=CollectionReaderDescriptorFactory.create_descriptor("hsr_ros2")
        )
        human_descriptor = HumanAndPoseAnnotator.Descriptor()
        human_descriptor.parameters.global_with_depth = True

        def select_route(query: Query.Goal) -> HumanRoute | None:
            """Select a route for a supported human query.

            :param query: The requested object and its properties.
            :return: The matching route, or None if the query is unsupported.
            """
            if query.obj.type != QueryType.HUMAN:
                return None
            if query.obj.color or query.obj.description or query.obj.location:
                return None
            if not query.obj.attribute:
                return HumanRoute.DETECTION
            if len(query.obj.attribute) == 1 and query.obj.attribute[0] in HumanPosture:
                return HumanRoute.POSTURE
            return None

        detection = Sequence("Human detection", memory=True)
        detection.add_child(GenerateQueryResult())
        posture = Sequence("HumanPosture", memory=True)
        posture.add_children(
            [
                HumanSittingAnnotator(),
                HumanStandingAnnotator(),
                HumanLyingAnnotator(),
                GenerateHumanPostureResult(),
            ]
        )
        task_scheduling = Sequence("Task Scheduling", memory=True)
        task_scheduling.add_child(
            QueryBasedScheduler(
                tasks={HumanRoute.DETECTION: detection, HumanRoute.POSTURE: posture},
                filter_fn=select_route,
            )
        )
        pipeline = Pipeline("HumanDetection")
        pipeline.add_children(
            [
                pipeline_init(),
                QueryAnnotator(),
                camera,
                ImagePreprocessorAnnotator(),
                HumanAndPoseAnnotator(descriptor=human_descriptor),
                ClusterPositionAnnotator(),
                task_scheduling,
            ]
        )
        return pipeline
