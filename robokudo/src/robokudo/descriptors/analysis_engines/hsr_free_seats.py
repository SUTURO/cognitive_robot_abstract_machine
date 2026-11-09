"""
Analysis engine answering which people the HSR sees sitting in a region of the kitchen
lab, such as one of the seat places of its sofa.

A query naming a region by its ``location`` is answered with one object designator per
person sitting in that region, typed ``human`` and positioned at their cloud's center.
"""

from robokudo.analysis_engine import AnalysisEngineInterface
from robokudo.annotators.cluster_position import ClusterPositionAnnotator
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.human_and_pose import HumanAndPoseAnnotator
from robokudo.annotators.human_sitting import HumanSittingAnnotator
from robokudo.annotators.human_standing import HumanStandingAnnotator
from robokudo.annotators.image_preprocessor import ImagePreprocessorAnnotator
from robokudo.annotators.query import GenerateQueryResult, QueryAnnotator
from robokudo.annotators.region_occupant_filter import RegionOccupantFilter
from robokudo.annotators.world_descriptor_bootstrap import (
    WorldDescriptorBootstrapAnnotator,
)
from robokudo.descriptors.factories.cr_descriptor_factory import (
    CollectionReaderDescriptorFactory,
)
from robokudo.idioms import pipeline_init
from robokudo.pipeline import Pipeline


class AnalysisEngine(AnalysisEngineInterface):
    """
    Query-driven detection of the people within a region of the kitchen lab.
    """

    def name(self) -> str:
        return "hsr_free_seats"

    def implementation(self) -> Pipeline:
        """
        Build the pipeline that waits for a query, reads a frame of the HSR's head
        camera, detects the people in it and their posture, keeps those sitting within
        the queried region and replies with their positions.

        :return: The configured pipeline.
        """
        world_descriptor = WorldDescriptorBootstrapAnnotator.Descriptor()
        world_descriptor.parameters.world_descriptor_name = "world_kitchen_lab"

        human_descriptor = HumanAndPoseAnnotator.Descriptor()
        human_descriptor.parameters.global_with_depth = True
        human_descriptor.parameters.person_class_name = "human"

        pipeline = Pipeline("HsrFreeSeatsPipeline")
        pipeline.add_children(
            [
                pipeline_init(),
                QueryAnnotator(),
                CollectionReaderAnnotator(
                    descriptor=CollectionReaderDescriptorFactory.create_descriptor(
                        "hsr_ros2"
                    )
                ),
                ImagePreprocessorAnnotator("ImagePreprocessor"),
                WorldDescriptorBootstrapAnnotator(descriptor=world_descriptor),
                HumanAndPoseAnnotator(descriptor=human_descriptor),
                ClusterPositionAnnotator(),
                HumanStandingAnnotator(),
                HumanSittingAnnotator(),
                RegionOccupantFilter(),
                GenerateQueryResult(),
            ]
        )
        return pipeline
