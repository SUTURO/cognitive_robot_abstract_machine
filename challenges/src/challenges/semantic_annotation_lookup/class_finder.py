"""
Looking up the semantic annotation class a spoken or written name refers to.
"""

from __future__ import annotations

# %% Imports
import inspect
from dataclasses import dataclass

from krrood.utils import recursive_subclasses

from challenges.semantic_annotation_lookup.exceptions import (
    AmbiguousSemanticAnnotationName,
    UnknownSemanticAnnotationName,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Room
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation


# %% Name lookup


@dataclass
class SemanticAnnotationClassFinder:
    """
    Finds the semantic annotation class a spoken or written object name refers to, such
    as ``"cola"`` for a class ``Cola`` or ``"pringles can"`` for ``PringlesCan``.

    Names match regardless of case and of spaces, underscores and hyphens between words.
    A name no class has exactly falls back to the concrete classes whose name contains
    it, such as ``"counter"`` for ``CounterTop``.
    """

    word_separators: str = " _-"
    """
    The characters that separate the words of a name and are ignored when matching.
    """

    def find(self, name: str) -> type[SemanticAnnotation]:
        """
        :param name: The name of an object, such as ``"cola"`` or ``"pringles_can"``.
        :return: The semantic annotation class with that name, or else the one concrete
            class whose name contains it.
        :raises UnknownSemanticAnnotationName: If no class has or contains that name.
        :raises AmbiguousSemanticAnnotationName: If several classes have that name, or
            none has it and several concrete classes contain it.
        """
        return self.select_single_match(
            name, self.classes_named(name) or self.concrete_classes_containing(name)
        )

    def find_place(self, name: str) -> type[SemanticAnnotation]:
        """
        Differs from :meth:`find` in that a name no class has exactly, but that contains
        the name of a room, refers to that room, such as ``"living room table"`` for
        :class:`LivingRoom`.

        :param name: The name of a place, such as ``"kitchen"`` or ``"living room
            table"``.
        :return: The class with that name, else the room the name contains, else the one
            concrete class whose name contains it.
        :raises UnknownSemanticAnnotationName: If no class matches the name.
        :raises AmbiguousSemanticAnnotationName: If the name contains several room
            names.
        """
        return self.select_single_match(
            name,
            self.classes_named(name)
            or self.rooms_mentioned(name)
            or self.concrete_classes_containing(name),
        )

    def classes_named(self, name: str) -> set[type[SemanticAnnotation]]:
        """
        :param name: The name of an object.
        :return: The semantic annotation classes with exactly that name.
        """
        comparable_name = self.comparable_name(name)
        return {
            annotation_class
            for annotation_class in recursive_subclasses(SemanticAnnotation)
            if self.comparable_name(annotation_class.__name__) == comparable_name
        }

    def concrete_classes_containing(self, name: str) -> set[type[SemanticAnnotation]]:
        """
        :param name: The name of an object.
        :return: The concrete semantic annotation classes whose name contains it.
        """
        comparable_name = self.comparable_name(name)
        return {
            annotation_class
            for annotation_class in recursive_subclasses(SemanticAnnotation)
            if comparable_name in self.comparable_name(annotation_class.__name__)
            and not inspect.isabstract(annotation_class)
        }

    def rooms_mentioned(self, name: str) -> set[type[Room]]:
        """
        :param name: The name of a place.
        :return: The concrete room classes whose name the place's name contains.
        """
        comparable_name = self.comparable_name(name)
        return {
            room_class
            for room_class in recursive_subclasses(Room)
            if self.comparable_name(room_class.__name__) in comparable_name
            and not inspect.isabstract(room_class)
        }

    def select_single_match(
        self, name: str, matches: set[type[SemanticAnnotation]]
    ) -> type[SemanticAnnotation]:
        """
        :param name: The name the classes were found for.
        :param matches: The classes found for the name.
        :return: The only class found.
        :raises UnknownSemanticAnnotationName: If no class was found.
        :raises AmbiguousSemanticAnnotationName: If several classes were found.
        """
        if not matches:
            raise UnknownSemanticAnnotationName(name=name)
        if len(matches) > 1:
            raise AmbiguousSemanticAnnotationName(name=name, matches=list(matches))
        return next(iter(matches))

    def comparable_name(self, name: str) -> str:
        """
        :param name: A class name or object name.
        :return: The name in lower case without word separators.
        """
        return name.translate(str.maketrans("", "", self.word_separators)).lower()
