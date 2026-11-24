"""
Tests for finding the semantic annotation class a spoken or written name refers to.
"""

from __future__ import annotations

# %% Imports
from abc import ABC, abstractmethod
from dataclasses import dataclass

import pytest

from challenges.semantic_annotation_lookup.exceptions import (
    AmbiguousSemanticAnnotationName,
    UnknownSemanticAnnotationName,
)
from challenges.semantic_annotation_lookup.class_finder import (
    SemanticAnnotationClassFinder,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    DrinkingContainer,
    KitchenIsland,
    LivingRoom,
    Milk,
    Room,
)
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation


# %% Fixtures


@pytest.fixture
def finder() -> SemanticAnnotationClassFinder:
    """
    A finder with the default word separators.
    """
    return SemanticAnnotationClassFinder()


# %% Matching


def test_lower_case_name_finds_class(finder: SemanticAnnotationClassFinder) -> None:
    """
    A name in lower case finds the class of that name.
    """
    assert finder.find("milk") is Milk


@pytest.mark.parametrize(
    "name", ["drinking container", "drinking_container", "Drinking-Container"]
)
def test_name_with_word_separators_finds_class(
    finder: SemanticAnnotationClassFinder, name: str
) -> None:
    """
    Spaces, underscores and hyphens between the words of a name are ignored.
    """
    assert finder.find(name) is DrinkingContainer


# %% Matching part of a name


def test_name_contained_in_one_class_name_finds_that_class(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A name no class has finds the one class whose name contains it.
    """

    @dataclass(eq=False)
    class ContainedNameWidget(SemanticAnnotation): ...

    assert finder.find("contained name") is ContainedNameWidget


def test_exact_name_wins_over_names_containing_it(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A class with exactly the name wins over classes whose names contain it.
    """

    @dataclass(eq=False)
    class ExactNameProbe(SemanticAnnotation): ...

    @dataclass(eq=False)
    class ExactNameProbeHolder(SemanticAnnotation): ...

    assert finder.find("exact name probe") is ExactNameProbe


def test_abstract_class_containing_the_name_is_skipped(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    Abstract classes whose names contain the name are not found.
    """

    @dataclass(eq=False)
    class SkippedAbstractGadget(SemanticAnnotation, ABC):
        @abstractmethod
        def describe_gadget(self) -> str: ...

    @dataclass(eq=False)
    class SkippedConcreteGadget(SemanticAnnotation): ...

    assert finder.find("gadget") is SkippedConcreteGadget


def test_name_contained_in_two_class_names_raises(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A name contained in the names of two classes is ambiguous.
    """

    @dataclass(eq=False)
    class TwiceContainedLeft(SemanticAnnotation): ...

    @dataclass(eq=False)
    class TwiceContainedRight(SemanticAnnotation): ...

    with pytest.raises(AmbiguousSemanticAnnotationName) as raised:
        finder.find("twice contained")
    assert set(raised.value.matches) == {TwiceContainedLeft, TwiceContainedRight}


# %% Places named after their room


def test_place_containing_a_room_name_finds_the_room(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A place whose name contains a room's name refers to that room.
    """
    assert finder.find_place("living room table") is LivingRoom


def test_exact_name_of_a_place_wins_over_the_room_it_contains(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A place with exactly the name wins over the room its name contains.
    """
    assert finder.find_place("kitchen island") is KitchenIsland


def test_place_without_a_room_name_finds_class_like_find(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A place whose name contains no room's name is found as :meth:`find` finds it.
    """
    assert finder.find_place("milk") is Milk


def test_place_containing_two_room_names_raises(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A place whose name contains the names of two rooms is ambiguous.
    """

    @dataclass(eq=False)
    class FirstProbeSuite(Room): ...

    @dataclass(eq=False)
    class SecondProbeSuite(Room): ...

    with pytest.raises(AmbiguousSemanticAnnotationName) as raised:
        finder.find_place("first probe suite second probe suite")
    assert set(raised.value.matches) == {FirstProbeSuite, SecondProbeSuite}


# %% Failures


def test_unknown_name_raises(finder: SemanticAnnotationClassFinder) -> None:
    """
    A name no class has or contains is unknown.
    """
    with pytest.raises(UnknownSemanticAnnotationName):
        finder.find("no such object")


def test_name_shared_by_two_classes_raises(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A name two classes have is ambiguous.
    """

    @dataclass(eq=False)
    class SharedName(SemanticAnnotation): ...

    first_class = SharedName

    @dataclass(eq=False)
    class SharedName(SemanticAnnotation): ...

    with pytest.raises(AmbiguousSemanticAnnotationName) as raised:
        finder.find("shared name")
    assert set(raised.value.matches) == {first_class, SharedName}


# %% Edge cases of the lookup


def test_abstract_class_with_exactly_the_name_is_found(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A class with exactly the name is found even if it is abstract.
    """

    @dataclass(eq=False)
    class AbstractExactProbe(SemanticAnnotation, ABC):
        @abstractmethod
        def describe_probe(self) -> str: ...

    assert finder.find("abstract exact probe") is AbstractExactProbe


def test_custom_word_separators_are_ignored() -> None:
    """
    A finder ignores the word separators it is given.
    """
    finder = SemanticAnnotationClassFinder(word_separators=".")
    assert finder.find("drinking.container") is DrinkingContainer


def test_place_without_a_room_name_falls_back_to_the_class_containing_it(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    A place no class has exactly and that names no room finds the one class whose name
    contains it.
    """

    @dataclass(eq=False)
    class ProbeShelfUnit(SemanticAnnotation): ...

    assert finder.find_place("probe shelf") is ProbeShelfUnit


def test_abstract_room_in_a_place_name_is_skipped(
    finder: SemanticAnnotationClassFinder,
) -> None:
    """
    An abstract room whose name the place's name contains is not the place.
    """

    @dataclass(eq=False)
    class AbstractProbeHall(Room, ABC):
        @abstractmethod
        def describe_hall(self) -> str: ...

    with pytest.raises(UnknownSemanticAnnotationName):
        finder.find_place("abstract probe hall table")


def test_unknown_place_raises(finder: SemanticAnnotationClassFinder) -> None:
    """
    A place no class has, contains or names a room of is unknown.
    """
    with pytest.raises(UnknownSemanticAnnotationName):
        finder.find_place("no such place")
