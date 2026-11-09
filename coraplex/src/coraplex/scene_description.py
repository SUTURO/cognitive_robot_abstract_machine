"""
Spoken descriptions of observed objects and their current supporting surfaces.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from krrood.entity_query_language.verbalization.fragments.features import Definiteness
from krrood.entity_query_language.verbalization.rendering.realization import (
    realize_subtree,
)
from std_msgs.msg import String
from krrood.entity_query_language.verbalization.vocabulary.parts_of_speech import Noun
from semantic_digital_twin.reasoning.predicates import InsideOf, SupportedBy
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    HasSupportingSurface,
)
from semantic_digital_twin.utils import camel_case_split
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.perception import PerceptionInterface, PerceptionQuery
    from rclpy.publisher import Publisher
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.world_entity import (
        Body,
        SemanticAnnotation,
    )

# %% spoken claims


class SceneSentence(StrEnum):
    """
    Sentence forms for established observations and support relationships.
    """

    EMPTY = "I do not see any objects."
    """Report an empty observation set without claiming that the room is empty."""
    VISIBLE = "I see {object_phrase}."
    """Report an object whose furniture location has not been established."""
    ON_SURFACE = "There is {object_phrase} on {surface_phrase}."
    """Report support without enclosure."""
    IN_CONTAINER = "There is {object_phrase} in {surface_phrase}."
    """Report support with enclosure under the configured containment criterion."""


# %% scene description


@dataclass
class SceneDescription:
    """
    Describe only the supplied observed bodies using the world's current state.
    """

    world: World
    """
    World whose semantic annotations and collision geometry establish locations.
    """

    containment_fraction: float = 1.0
    """
    Fraction required to say inside; by default the entire object must be enclosed.
    """

    def describe(self, observed_bodies: list[Body]) -> str:
        """
        Return one sentence per distinct observed body, without guessing its support.
        """
        if not observed_bodies:
            return SceneSentence.EMPTY
        return " ".join(
            self.describe_body(body) for body in dict.fromkeys(observed_bodies)
        )

    def describe_body(self, body: Body) -> str:
        """
        Describe an observed body, withholding locations with multiple possible
        supports.
        """
        annotations = [
            annotation
            for annotation in self.world.semantic_annotations
            if isinstance(annotation, HasRootBody) and annotation.root is body
        ]
        object_phrase = self.render_noun(annotations, Definiteness.INDEFINITE)
        surfaces = [
            annotation
            for annotation in self.world.semantic_annotations
            if isinstance(annotation, HasSupportingSurface)
            and annotation.root is not body
            and SupportedBy(body, annotation.root)()
        ]
        support_bodies = {surface.root for surface in surfaces}
        if len(support_bodies) != 1:
            return SceneSentence.VISIBLE.format(object_phrase=object_phrase)
        surface_phrase = self.render_noun(surfaces, Definiteness.DEFINITE)
        supporting_body = next(iter(support_bodies))
        sentence = (
            SceneSentence.IN_CONTAINER
            if InsideOf(
                body=body,
                other=supporting_body,
                minimum_containment_ratio=self.containment_fraction,
            )()
            else SceneSentence.ON_SURFACE
        )
        return sentence.format(
            object_phrase=object_phrase, surface_phrase=surface_phrase
        )

    @staticmethod
    def render_noun(
        annotations: list[SemanticAnnotation], definiteness: Definiteness
    ) -> str:
        """
        Render the most specific unambiguous annotation category with its article.
        """
        categories = {type(annotation) for annotation in annotations}
        specific = [
            category
            for category in categories
            if not any(
                other is not category and issubclass(other, category)
                for other in categories
            )
        ]
        label = (
            " ".join(camel_case_split(specific[0].__name__)).lower()
            if len(specific) == 1
            else "object"
        )
        return realize_subtree(Noun(label, definiteness=definiteness).as_fragment())


# %% speech publication


@dataclass
class SceneAnnouncement:
    """
    Publish one scene report per explicit request to a supplied text publisher.
    """

    description: SceneDescription
    """
    Describer sharing the world whose bodies were observed.
    """

    publisher: Publisher
    """
    String publisher connected to the configured speech input topic.
    """

    def announce(self, observed_bodies: list[Body]) -> None:
        """
        Publish a complete report without claiming that audio playback finished.
        """
        sentence = self.description.describe(observed_bodies)
        self.publisher.publish(String(data=sentence))


# %% perception reports


@dataclass
class PerceptionAnnouncement:
    """Report the fresh result of an explicit perception query through speech input."""

    source: PerceptionInterface
    """Perception backend answering the request, including RoboKudo on hardware."""

    publisher: Publisher
    """String publisher connected to the speech input topic."""

    def announce(self, query: PerceptionQuery) -> None:
        """Update detected poses and announce only the bodies identified by this request.

        Unresolvable detections propagate their perception error without publishing.
        """
        detections = self.source.look_for(query)
        for detection in detections:
            detection.resolve_annotations(query.world)
        observed_bodies = []
        for detection in detections:
            annotations = detection.apply_to(
                query.world, trust_orientation=query.trust_detected_orientation
            )
            observed_bodies.extend(annotation.root for annotation in annotations)
        SceneAnnouncement(
            description=SceneDescription(world=query.world), publisher=self.publisher
        ).announce(observed_bodies)
