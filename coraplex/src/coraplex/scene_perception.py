"""Completed scene observations and furniture-grounded spoken reports."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import Counter
from copy import deepcopy
from dataclasses import dataclass, field
from enum import StrEnum
from uuid import UUID

from std_msgs.msg import String
from typing_extensions import TYPE_CHECKING
from coraplex.perception import Detection
from coraplex.scene_description import SceneDescription
from cramph.composites import ChildChooser, CompositeNodeChoosingItsChild
from cramph.node import NodeArtifacts, StatechartNode, SucceedsOnObservingTrue
from krrood.patterns.field_metadata import JSONMetadata
from krrood.symbolic_math.symbolic_math import Scalar
from krrood.entity_query_language.verbalization.fragments.base import (
    NounPhrase,
    WordFragment,
    oxford_comma,
)
from krrood.entity_query_language.verbalization.fragments.features import (
    Definiteness,
    GrammaticalNumber,
)
from krrood.entity_query_language.verbalization.rendering.realization import (
    realize_subtree,
)
from krrood.entity_query_language.verbalization.vocabulary.parts_of_speech import Noun
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.reasoning.predicates import SupportedBy, InsideOf
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.utils import camel_case_split

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from semantic_digital_twin.world_description.world_entity import SemanticAnnotation
    from krrood_spatial_math.spatial_types import Pose
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.geometry import (
        Scale,
        VolumetricBoundingBox,
    )

# %% observation contracts


@dataclass
class SceneQuery:
    """A scene request with no object-category or cardinality restriction."""

    world: World
    """Current map supplying frames and furniture geometry."""
    region: VolumetricBoundingBox | None = None
    """Optional observation area; known out-of-area positions are excluded."""
    furniture: HasSupportingSurface | None = None
    """Optional target supplied by the task rather than selected by a fixed name."""
    contact_tolerance: float = 0.03
    """
    Metres a detected box may float above a surface and still count as resting on it.

    Detected boxes are less exact than simulated placements, so this is wider than
    the default of `SupportedBy`.
    """


@dataclass
class ObservedObject:
    """One hypothesis from a completed perception batch, independent of map entries."""

    category: str = ""
    """Reported classification, empty when the object was not classified."""
    identity: str | None = None
    """Batch-local hypothesis identity; absent identities never imply duplicates."""
    pose: Pose | None = None
    """Measured pose with its reference frame, when localization succeeded."""
    dimensions: Scale | None = None
    """Measured box extents; absent dimensions cannot establish contact."""


class SceneSentence(StrEnum):
    """Sentence forms of a spoken scene report."""

    SINGLE = "There is {objects}{location}."
    """Report exactly one object at a location."""
    SEVERAL = "There are {objects}{location}."
    """Report more than one object at a location."""
    NOTHING = "I did not detect any objects."
    """Report an empty batch without claiming that the room is empty."""
    NOTHING_ON_FURNITURE = "I did not detect any objects on {furniture}."
    """Report an empty batch for a furniture target supplied by the task."""


class SceneRelation(StrEnum):
    """Spatial claims that the report can establish from the available geometry."""

    UNKNOWN = ""
    """No furniture location established."""
    ON = "on"
    """Supported by a furniture surface."""
    IN = "in"
    """Supported and fully enclosed by the furniture bounding box."""


@dataclass(frozen=True)
class SceneLocation:
    """A furniture instance and relation used to group a spoken report."""

    identity: UUID | None = None
    """Furniture root identity, keeping same-category furniture instances distinct."""
    phrase: str = ""
    """Rendered furniture noun phrase."""
    relation: SceneRelation = SceneRelation.UNKNOWN
    """Established relationship to that furniture."""


@dataclass
class SceneObservation:
    """The complete hypotheses returned by one successful perception request."""

    objects: list[ObservedObject] = field(default_factory=list)
    """Hypotheses for this request only, including unclassified ones."""

    def detections(
        self,
        annotation_types_by_category: dict[str, type[SemanticAnnotation]],
    ) -> list[Detection]:
        """
        :param annotation_types_by_category: The annotation each reported class stands
            for; objects of other classes are left out.
        :return: One detection per annotation type, at the pose of the first object of
            a class standing for it that was located.
        """
        detections: dict[type[SemanticAnnotation], Detection] = {}
        for observed_object in self.objects:
            annotation_type = annotation_types_by_category.get(observed_object.category)
            if (
                annotation_type is None
                or observed_object.pose is None
                or annotation_type in detections
            ):
                continue
            detections[annotation_type] = Detection(
                semantic_annotation=annotation_type, pose=observed_object.pose
            )
        return list(detections.values())

    def describe(self, query: SceneQuery) -> str:
        """Group distinct hypotheses by their supported locations without altering the map."""
        snapshot = deepcopy(query.world)
        groups: dict[SceneLocation, Counter[str]] = {}
        seen: set[str] = set()
        for index, observation in enumerate(self.objects):
            if observation.identity:
                if observation.identity in seen:
                    continue
                seen.add(observation.identity)
            if observation.pose is not None and query.region is not None:
                position = query.world.transform(
                    observation.pose, query.world.root
                ).position
                if not query.region.contains(position):
                    continue
            location = self.locate(observation, query, snapshot, index)
            if (
                query.furniture is not None
                and location.identity is not None
                and location.identity != query.furniture.root.id
            ):
                continue
            category = (
                " ".join(
                    part
                    for word in observation.category.replace("_", " ").split()
                    for part in camel_case_split(word)
                ).lower()
                or "unidentified object"
            )
            groups.setdefault(location, Counter())[category] += 1
        if not groups:
            if query.furniture is None:
                return SceneSentence.NOTHING
            furniture = SceneDescription.render_noun(
                [query.furniture], Definiteness.DEFINITE
            )
            return SceneSentence.NOTHING_ON_FURNITURE.format(furniture=furniture)
        sentences = []
        for location, counts in groups.items():
            nouns = []
            for category, count in counts.items():
                if count == 1:
                    nouns.append(Noun(category).as_fragment())
                else:
                    nouns.append(
                        NounPhrase(
                            head=WordFragment(text=category),
                            number=GrammaticalNumber.PLURAL,
                            definiteness=Definiteness.BARE,
                            pre_head=WordFragment(text=str(count)),
                        )
                    )
            phrase = realize_subtree(oxford_comma(nouns, WordFragment(text="and")))
            suffix = (
                f" {location.relation} {location.phrase}"
                if location.identity is not None
                else ""
            )
            sentence = (
                SceneSentence.SINGLE
                if sum(counts.values()) == 1
                else SceneSentence.SEVERAL
            )
            sentences.append(sentence.format(objects=phrase, location=suffix))
        return " ".join(sentences)

    def locate(
        self,
        observation: ObservedObject,
        query: SceneQuery,
        snapshot: World,
        index: int,
    ) -> SceneLocation:
        """Evaluate measured box support in an isolated world; unknown geometry stays unlocated."""
        if observation.pose is None or observation.dimensions is None:
            return SceneLocation()
        if any(
            dimension <= 0
            for dimension in (
                observation.dimensions.x,
                observation.dimensions.y,
                observation.dimensions.z,
            )
        ):
            return SceneLocation()
        root_T_object = query.world.transform(
            observation.pose, query.world.root
        ).homogeneous_matrix
        body = BodySpecification.box(
            name=f"observation_{index}",
            scale=observation.dimensions,
            parent_T_self=HomogeneousTransformationMatrix(
                root_T_object.to_np(), reference_frame=snapshot.root
            ),
        ).spawn(snapshot)
        surfaces = [
            surface
            for surface in snapshot.semantic_annotations
            if isinstance(surface, HasSupportingSurface)
            and SupportedBy(
                body, surface.root, contact_tolerance=query.contact_tolerance
            )()
        ]
        if len({surface.root.id for surface in surfaces}) != 1:
            return SceneLocation()
        surface = surfaces[0]
        relation = (
            SceneRelation.IN
            if InsideOf(body=body, other=surface.root, minimum_containment_ratio=1.0)()
            else SceneRelation.ON
        )
        return SceneLocation(
            identity=surface.root.id,
            phrase=SceneDescription.render_noun(surfaces, Definiteness.DEFINITE),
            relation=relation,
        )


# %% completed perception and speech


@dataclass
class ScenePerception(ABC):
    """A source of completed, unrestricted scene observations."""

    @abstractmethod
    def observe(self, query: SceneQuery) -> SceneObservation:
        """Return a successful final observation or raise; feedback is not a completed batch."""


@dataclass
class SceneReport:
    """Publish one report only after a scene source completes successfully."""

    source: ScenePerception
    """Backend returning the final batch or raising its perception error."""
    publisher: Publisher
    """Text publisher connected to speech input."""

    def announce(self, query: SceneQuery) -> None:
        """Request the scene and publish its completed description exactly once."""
        batch = self.source.observe(query)
        sentence = batch.describe(query)
        self.publisher.publish(String(data=sentence))


# %% observing the scene as a step of a plan


@dataclass(eq=False, repr=False, kw_only=True)
class SceneObservationNode(CompositeNodeChoosingItsChild):
    """
    A step of a plan that asks the source of the :class:`SceneObserver` of its context
    what the robot sees, and keeps the whole answer in :attr:`observation` of the copy
    of the statechart the plan was built in.
    """

    observation: SceneObservation | None = field(
        default=None, init=False, metadata=JSONMetadata(serialize=False).as_dict()
    )
    """
    Everything the source answered the latest time this step ran, unfiltered, and
    ``None`` before that.
    """


@dataclass(eq=False, repr=False, kw_only=True)
class ObservationReceived(SucceedsOnObservingTrue, StatechartNode):
    """
    The child of a :class:`SceneObservationNode` whose answer arrived, which succeeds at
    once so the plan goes on.
    """

    def build_artifacts(self) -> NodeArtifacts:
        return NodeArtifacts(observation=Scalar.const_true())


@dataclass
class SceneObserver(ChildChooser[SceneObservationNode]):
    """
    Answers every :class:`SceneObservationNode` of a plan with :attr:`source`, in the
    process the plan was built in, and moves the annotations of
    :attr:`annotation_types_by_category` in the world to where they were seen.
    """

    source: ScenePerception
    """
    The scene source asked each time a :class:`SceneObservationNode` starts.
    """

    annotation_types_by_category: dict[str, type[SemanticAnnotation]] = field(
        default_factory=dict
    )
    """
    The annotation of the world each reported class stands for. Only the body of such an
    annotation is moved, see :meth:`SceneObservation.detections`; the rest of the world
    stays as it is.
    """

    def choose_child(self, node: SceneObservationNode) -> StatechartNode:
        world = node.context.world
        node.observation = self.source.observe(SceneQuery(world=world))
        for detection in node.observation.detections(self.annotation_types_by_category):
            detection.apply_to(world)
        return ObservationReceived()
