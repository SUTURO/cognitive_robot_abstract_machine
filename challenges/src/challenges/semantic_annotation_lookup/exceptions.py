"""
Exceptions raised when a name refers to no semantic annotation class or to several.
"""

from __future__ import annotations

# %% Imports
from dataclasses import dataclass

from krrood.exceptions import DataclassException
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from semantic_digital_twin.world_description.world_entity import (
        SemanticAnnotation,
    )


# %% Name lookup


@dataclass
class UnknownSemanticAnnotationName(DataclassException):
    """
    Raised when no semantic annotation class has or contains the requested name.
    """

    name: str
    """
    The object name no class was found for.
    """

    def error_message(self) -> str:
        return f"No semantic annotation class is named or contains '{self.name}'."

    def suggest_correction(self) -> str:
        return (
            f"Define a semantic annotation class for '{self.name}' "
            f"or check the spelling of the name."
        )


@dataclass
class AmbiguousSemanticAnnotationName(DataclassException):
    """
    Raised when several semantic annotation classes have the requested name, or none has
    it and several concrete classes contain it.
    """

    name: str
    """
    The object name several classes were found for.
    """

    matches: list[type[SemanticAnnotation]]
    """
    The classes that have or contain the name.
    """

    def error_message(self) -> str:
        return (
            f"The name '{self.name}' matches several semantic annotation classes: "
            f"{[f'{match.__module__}.{match.__qualname__}' for match in self.matches]}."
        )

    def suggest_correction(self) -> str:
        return "Rename one of the classes so that their names differ."
